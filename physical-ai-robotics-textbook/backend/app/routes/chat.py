"""
API routes for conversation management with comprehensive error handling.
"""

import asyncio
import time
import re
from fastapi import APIRouter, HTTPException, Request, Query
from typing import List, Optional
from datetime import datetime

from app.models import (
    ChatRequest,
    ChatResponse,
    ConversationResponse,
    MessageBase,
    SourceCitation,
)
from app.services.postgres_service import postgres_service
from app.services.agent_service import agent_service
from app.services.vector_search_service import vector_search_service
from app.utils.logger import get_logger, log_api_request, log_performance, log_error
from app.utils.error_handler import (
    error_handler,
    handle_and_log_exception,
    log_sensitive_operation,
    ServiceError,
    ValidationError,
    NotFoundError,
    AIServiceError,
)


router = APIRouter()
logger = get_logger("api_routes")


@router.post("/chat", response_model=ChatResponse)
@error_handler
async def chat_endpoint(request: Request, chat_request: ChatRequest):
    """
    Main chat endpoint that handles user messages and returns AI responses.
    Creates new conversations if needed and stores all messages in the database.
    """
    start_time = time.time()

    # Log API request with full context
    log_api_request(
        endpoint="/api/chat",
        method=request.method,
        query=chat_request.message[:100] if chat_request.message else "",
        conversation_id=chat_request.conversation_id,
        user_id=(
            getattr(request.state, "user_id", None)
            if hasattr(request, "state")
            else None
        ),
    )

    try:
        # Validate the input
        if len(chat_request.message.strip()) < 1:
            raise ValidationError("Message cannot be empty", {"field": "message"})

        # Validate conversation_id format if provided
        if chat_request.conversation_id:
            # Basic validation for conversation_id format (UUID-like)
            import re

            if not re.match(r"^[a-zA-Z0-9_-]+$", chat_request.conversation_id):
                raise ValidationError(
                    "Invalid conversation ID format", {"field": "conversation_id"}
                )

        # Pass conversation_id to agent service, or None if new conversation
        # The agent service handles conversation creation and message storage
        agent_result = await agent_service.generate_response(
            query=chat_request.message,
            conversation_id=chat_request.conversation_id,  # Pass the original conversation_id
            selected_text=chat_request.selected_text,
        )

        # Extract response and sources from agent result
        agent_response = agent_result["response"]
        sources = agent_result["sources"]

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "chat_endpoint",
            duration,
            {
                "conversation_id": agent_result["conversation_id"],
                "message_length": len(chat_request.message),
            },
        )

        # Create the response with all required fields
        response = ChatResponse(
            response=agent_response,
            conversation_id=agent_result["conversation_id"],
            sources=sources,
            timestamp=datetime.utcnow(),
        )

        # Log successful request completion
        logger.info(
            f"Chat request completed successfully in {duration:.2f}s for conversation {response.conversation_id}"
        )

        return response

    except ValidationError as ve:
        logger.warning(f"Validation error in chat endpoint: {ve.message}")
        raise HTTPException(status_code=400, detail=ve.message)
    except AIServiceError as ae:
        logger.error(f"AI service error in chat endpoint: {ae.message}")
        raise HTTPException(
            status_code=ae.status_code,
            detail="I am currently unavailable. Please try again later.",
        )
    except Exception as e:
        # Log the full exception details including traceback
        error_info = handle_and_log_exception(e, "chat_endpoint")
        logger.error(f"Unexpected error in chat endpoint: {str(e)}", exc_info=True)

        # Return user-friendly error message
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.get("/conversations/{conversation_id}", response_model=ConversationResponse)
@error_handler
async def get_conversation(request: Request, conversation_id: str):
    """
    Retrieve a conversation with all its messages.
    """
    start_time = time.time()

    # Log API request
    log_api_request(
        endpoint=f"/api/conversations/{conversation_id}",
        method=request.method,
        conversation_id=conversation_id,
    )

    try:
        # Get conversation
        conversation = await postgres_service.get_conversation(conversation_id)
        if not conversation:
            raise NotFoundError(f"Conversation {conversation_id} not found")

        # Get messages for this conversation
        messages = await postgres_service.get_messages(conversation_id)

        # Format the response
        message_objects = [
            MessageBase(
                id=msg["id"],
                conversation_id=msg["conversation_id"],
                role=msg["role"],
                content=msg["content"],
                timestamp=msg["timestamp"],
                sources=msg.get("sources", []),
            )
            for msg in messages
        ]

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "get_conversation", duration, {"conversation_id": conversation_id}
        )

        return ConversationResponse(
            id=conversation["id"],
            user_id=conversation["user_id"],
            title=conversation["title"],
            created_at=conversation["created_at"],
            updated_at=conversation["updated_at"],
            messages=message_objects,
        )

    except NotFoundError:
        raise
    except Exception as e:
        error_info = handle_and_log_exception(e, "get_conversation")
        logger.error(f"Unexpected error in get_conversation: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.get("/conversations", response_model=List[ConversationResponse])
@error_handler
async def get_conversations(
    request: Request,
    user_id: Optional[str] = Query(None, description="Filter by user ID"),
    limit: int = Query(
        50, ge=1, le=100, description="Number of conversations to return"
    ),
):
    """
    Retrieve all conversations for a user (or all conversations if no user_id provided).
    """
    start_time = time.time()

    # Log API request
    log_api_request(
        endpoint="/api/conversations", method=request.method, user_id=user_id
    )

    try:
        conversations_data = await postgres_service.get_conversations(
            user_id=user_id, limit=limit
        )

        conversations = []
        for conv in conversations_data:
            # Get message count for each conversation
            messages = await postgres_service.get_messages(
                conv["id"], limit=1
            )  # Just get first message for metadata
            conversations.append(
                ConversationResponse(
                    id=conv["id"],
                    user_id=conv["user_id"],
                    title=conv["title"],
                    created_at=conv["created_at"],
                    updated_at=conv["updated_at"],
                    messages=[],  # Don't include full messages in list view for performance
                )
            )

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "get_conversations",
            duration,
            {"user_id": user_id, "limit": limit, "returned_count": len(conversations)},
        )

        return conversations

    except Exception as e:
        error_info = handle_and_log_exception(e, "get_conversations")
        logger.error(f"Unexpected error in get_conversations: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.get(
    "/conversations/{conversation_id}/messages", response_model=List[MessageBase]
)
@error_handler
async def get_conversation_messages(
    request: Request,
    conversation_id: str,
    limit: int = Query(50, ge=1, le=100, description="Number of messages to return"),
    recent: bool = Query(
        False, description="Return most recent messages instead of oldest"
    ),
):
    """
    Retrieve messages for a specific conversation.
    """
    start_time = time.time()

    # Log API request
    log_api_request(
        endpoint=f"/api/conversations/{conversation_id}/messages",
        method=request.method,
        conversation_id=conversation_id,
    )

    try:
        # Verify conversation exists
        conversation = await postgres_service.get_conversation(conversation_id)
        if not conversation:
            raise NotFoundError(f"Conversation {conversation_id} not found")

        # Get messages
        if recent:
            # Get recent messages (useful for context in agent service)
            messages_data = await postgres_service.get_recent_messages(
                conversation_id, limit
            )
        else:
            messages_data = await postgres_service.get_messages(conversation_id, limit)

        messages = [
            MessageBase(
                id=msg["id"],
                conversation_id=msg["conversation_id"],
                role=msg["role"],
                content=msg["content"],
                timestamp=msg["timestamp"],
                sources=msg.get("sources", []),
            )
            for msg in messages_data
        ]

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "get_conversation_messages",
            duration,
            {
                "conversation_id": conversation_id,
                "limit": limit,
                "recent": recent,
                "returned_count": len(messages),
            },
        )

        return messages

    except NotFoundError:
        raise
    except Exception as e:
        error_info = handle_and_log_exception(e, "get_conversation_messages")
        logger.error(f"Unexpected error in get_conversation_messages: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.delete("/conversations/{conversation_id}")
@error_handler
async def delete_conversation(request: Request, conversation_id: str):
    """
    Delete a conversation and all its messages.
    """
    start_time = time.time()

    # Log API request
    log_api_request(
        endpoint=f"/api/conversations/{conversation_id}",
        method=request.method,
        conversation_id=conversation_id,
    )

    try:
        success = await postgres_service.delete_conversation(conversation_id)
        if not success:
            raise NotFoundError(f"Conversation {conversation_id} not found")

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "delete_conversation", duration, {"conversation_id": conversation_id}
        )

        return {"message": "Conversation deleted successfully"}

    except NotFoundError:
        raise
    except Exception as e:
        error_info = handle_and_log_exception(e, "delete_conversation")
        logger.error(f"Unexpected error in delete_conversation: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.post("/ingest")
@error_handler
async def ingest_documents(request: Request):
    """
    Endpoint to trigger document ingestion into the vector database.
    """
    start_time = time.time()

    # Log API request
    log_api_request(endpoint="/api/ingest", method=request.method)

    try:
        from app.scripts.ingest_documents import main as run_ingestion

        # Run the ingestion process
        # Note: In a real implementation, you'd want to run this in the background
        result = await run_ingestion()

        # Log performance
        duration = time.time() - start_time
        log_performance(
            "ingest_documents",
            duration,
            {
                "documents_processed": (
                    result.get("files_processed", 0) if isinstance(result, dict) else 0
                ),
                "chunks_created": (
                    result.get("total_chunks_created", 0)
                    if isinstance(result, dict)
                    else 0
                ),
            },
        )

        return {
            "status": "success",
            "message": f"Documents ingested successfully",
            "documents_processed": (
                result.get("files_processed", 0) if isinstance(result, dict) else 0
            ),
            "chunks_created": (
                result.get("total_chunks_created", 0) if isinstance(result, dict) else 0
            ),
            "processing_time": (
                result.get("total_processing_time", 0)
                if isinstance(result, dict)
                else 0
            ),
        }
    except Exception as e:
        error_info = handle_and_log_exception(e, "ingest_documents")
        logger.error(f"Unexpected error in ingest_documents: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])


@router.post("/conversations", response_model=dict)
@error_handler
async def create_conversation(request: Request, user_id: Optional[str] = None):
    """
    Create a new conversation.
    """
    start_time = time.time()
    
    log_api_request(
        endpoint="/api/conversations",
        method=request.method,
        user_id=user_id,
    )
    
    try:
        import secrets
        import string
        
        # Generate unique conversation ID
        conversation_id = f"conv_{''.join(secrets.choice(string.ascii_lowercase + string.digits) for _ in range(15))}"
        
        # Create conversation in database
        await postgres_service.create_conversation(
            conversation_id=conversation_id,
            user_id=user_id,
            title="New Conversation"
        )
        
        # Log performance
        duration = time.time() - start_time
        log_performance(
            "create_conversation",
            duration,
            {"conversation_id": conversation_id}
        )
        
        logger.info(f"Created new conversation: {conversation_id}")
        
        return {
            "conversation_id": conversation_id,
            "created_at": datetime.utcnow().isoformat()
        }
        
    except Exception as e:
        error_info = handle_and_log_exception(e, "create_conversation")
        logger.error(f"Error creating conversation: {str(e)}")
        raise HTTPException(status_code=500, detail=error_info["detail"])