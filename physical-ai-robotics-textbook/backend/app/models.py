from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4
import json


# Request Models
class ChatRequest(BaseModel):
    """
    Request model for chat endpoint.
    """

    message: str = Field(
        ..., min_length=1, max_length=2000, description="User message to the chatbot"
    )
    conversation_id: Optional[str] = Field(
        None, description="Existing conversation ID, if any"
    )
    selected_text: Optional[str] = Field(
        None, max_length=2000, description="Text selected by user for context"
    )
    user_id: Optional[str] = Field(None, description="User identifier for tracking")


class IngestRequest(BaseModel):
    """
    Request model for document ingestion endpoint.
    """

    force_refresh: bool = Field(False, description="Whether to re-ingest all documents")
    source_path: Optional[str] = Field(None, description="Path to documents to ingest")


class QueryRequest(BaseModel):
    """
    Request model for vector search endpoint.
    """

    query: str = Field(..., min_length=1, max_length=1000, description="Search query")
    conversation_id: Optional[str] = Field(
        None, description="Conversation ID for context"
    )
    top_k: Optional[int] = Field(
        5, ge=1, le=20, description="Number of results to return"
    )
    filters: Optional[Dict[str, Any]] = Field(
        None, description="Optional filters for search"
    )


# Response Models
class SourceCitation(BaseModel):
    """
    Model for source citations in responses.
    """

    source_file: str
    module: str
    chapter: str
    chunk_index: int
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    content: str


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint.
    """

    response: str
    conversation_id: str
    sources: List[SourceCitation] = Field(default_factory=list)
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class IngestResponse(BaseModel):
    """
    Response model for ingestion endpoint.
    """

    success: bool
    message: str
    files_processed: int = 0
    chunks_created: int = 0
    processing_time: float = 0.0  # in seconds


class QueryResponse(BaseModel):
    """
    Response model for vector search endpoint.
    """

    query: str
    results: List[Dict[str, Any]]
    relevance_scores: List[float]
    processing_time: float = 0.0  # in seconds


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint.
    """

    status: str = "healthy"
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    services: Dict[str, bool] = Field(default_factory=dict)


# Database Models (for internal use)
class ConversationBase(BaseModel):
    """
    Base model for conversation data.
    """

    id: Optional[str] = Field(default_factory=lambda: str(uuid4()))
    user_id: Optional[str] = None
    title: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class MessageBase(BaseModel):
    """
    Base model for message data.
    """

    id: Optional[str] = Field(default_factory=lambda: str(uuid4()))
    conversation_id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    sources: Optional[List[SourceCitation]] = Field(default_factory=list)


class ConversationCreate(ConversationBase):
    """
    Model for creating a new conversation.
    """

    pass


class MessageCreate(MessageBase):
    """
    Model for creating a new message.
    """

    pass


class ConversationResponse(ConversationBase):
    """
    Response model for conversation data.
    """

    messages: List[MessageBase] = Field(default_factory=list)


# Vector Database Models
class DocumentChunk(BaseModel):
    """
    Model for document chunks stored in vector database.
    """

    id: str = Field(default_factory=lambda: str(uuid4()))
    content: str
    module: str
    chapter: str
    source_file: str
    chunk_index: int
    embedding: Optional[List[float]] = None
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)


class RetrievedChunk(BaseModel):
    """
    Model for chunks retrieved from vector database.
    """

    content: str
    module: str
    chapter: str
    source_file: str
    chunk_index: int
    relevance_score: float
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)
