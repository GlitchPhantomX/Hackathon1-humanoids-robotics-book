"""
AI Agent service for generating responses using Google Generative AI (Gemini).
Implements the RAG (Retrieval-Augmented Generation) pattern.
"""

from typing import List, Dict, Any, Optional
import google.generativeai as genai
from app.config import settings
from app.utils.logger import get_logger
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.services.postgres_service import postgres_service
from app.services.vector_search_service import vector_search_service
from app.models import RetrievedChunk, SourceCitation
import time
import asyncio
import re


class AgentService:
    """
    Service class for AI agent that combines retrieval and generation.
    Implements the RAG pattern: Retrieve → Augment → Generate
    """

    def __init__(self):
        self.api_key = settings.google_api_key
        self.model_name = settings.llm_model
        self.temperature = settings.temperature
        self.max_tokens = settings.max_tokens
        self.max_chunks = settings.max_retrieved_chunks
        self.logger = get_logger("agent_service")

        # Configure the Google Generative AI API
        genai.configure(api_key=self.api_key)

        # Initialize the generative model
        self.model = genai.GenerativeModel(self.model_name)

    async def generate_response(
        self,
        query: str,
        conversation_id: Optional[str] = None,
        selected_text: Optional[str] = None,
        user_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Generate a response to the user query using RAG pattern.

        Args:
            query: User's query
            conversation_id: Existing conversation ID (optional)
            selected_text: Text selected by user for context (optional)
            user_id: User identifier (optional)

        Returns:
            Dictionary containing response and metadata
        """
        start_time = time.time()
        self.logger.info(f"Generating response for query: '{query[:50]}...'")

        try:
            # ✅ Detect greetings and casual queries
            greeting_patterns = [
                r'^(hi|hello|hey|hola|greetings?|good\s+(morning|afternoon|evening)|howdy)[\s\!\?]*$',
                r'^(how\s+are\s+you|what\'?s\s+up|wassup|sup)[\s\!\?]*$',
                r'^(thanks?|thank\s+you|thx|ty)[\s\!\?]*$',
                r'^(bye|goodbye|see\s+you|cya)[\s\!\?]*$',
            ]
            
            query_lower = query.strip().lower()
            is_greeting = any(re.match(pattern, query_lower, re.IGNORECASE) for pattern in greeting_patterns)
            
            # Handle greetings without searching the database
            if is_greeting:
                self.logger.info(f"Detected greeting/casual query: '{query}'")
                
                # Generate friendly response based on query type
                if re.match(r'^(hi|hello|hey|hola|greetings?|good\s+(morning|afternoon|evening)|howdy)', query_lower):
                    casual_response = (
                        "Hello! 👋 I'm your Physical AI & Robotics textbook assistant. "
                        "I'm here to help you understand concepts from the course.\n\n"
                        "You can ask me about:\n"
                        "• Physical AI fundamentals and applications\n"
                        "• Robot components (sensors, actuators, control systems)\n"
                        "• Kinematics and dynamics\n"
                        "• Computer vision for robotics\n"
                        "• Specific chapters or topics from the textbook\n\n"
                        "What would you like to learn about today?"
                    )
                elif re.match(r'^(how\s+are\s+you|what\'?s\s+up|wassup|sup)', query_lower):
                    casual_response = (
                        "I'm doing great, thanks for asking! 😊\n\n"
                        "I'm ready to help you with any questions about Physical AI and Robotics. "
                        "Feel free to ask about specific topics, concepts, or chapters from the textbook!"
                    )
                elif re.match(r'^(thanks?|thank\s+you|thx|ty)', query_lower):
                    casual_response = (
                        "You're welcome! Happy to help! 😊\n\n"
                        "If you have any more questions about the textbook, feel free to ask!"
                    )
                elif re.match(r'^(bye|goodbye|see\s+you|cya)', query_lower):
                    casual_response = (
                        "Goodbye! Happy learning! 📚\n\n"
                        "Come back anytime you need help with Physical AI & Robotics concepts!"
                    )
                else:
                    casual_response = (
                        "Hello! I'm here to help you with the Physical AI & Robotics textbook. "
                        "What would you like to learn about?"
                    )
                
                # Save to conversation if ID exists
                if conversation_id:
                    await postgres_service.create_message(
                        conversation_id=conversation_id, role="user", content=query
                    )
                    await postgres_service.create_message(
                        conversation_id=conversation_id,
                        role="assistant",
                        content=casual_response,
                        sources=[],
                    )
                else:
                    # Create new conversation
                    new_conv = await postgres_service.create_conversation(
                        user_id=user_id,
                        title="Chat Session",
                    )
                    conversation_id = new_conv["id"]
                    await postgres_service.create_message(
                        conversation_id=conversation_id, role="user", content=query
                    )
                    await postgres_service.create_message(
                        conversation_id=conversation_id,
                        role="assistant",
                        content=casual_response,
                        sources=[],
                    )
                
                return {
                    "response": casual_response,
                    "conversation_id": conversation_id,
                    "sources": [],
                    "retrieval_time": time.time() - start_time,
                    "retrieved_chunks_count": 0,
                }
            
            # ✅ Preprocess query if it looks like a heading/topic
            original_query = query
            query_for_search = query
            
            # Detect if user is asking about a topic/heading (short query without question mark)
            if len(query.split()) <= 6 and not query.endswith('?') and not selected_text:
                # Expand the query for better retrieval
                query_for_search = f"What is {query}? Explain {query} in detail. Key concepts of {query}."
                self.logger.info(f"Expanded topic query: '{query}' -> '{query_for_search}'")
            
            # Step 2: Retrieve relevant chunks from vector database
            # If selected text is provided, use enhanced search
            if selected_text:
                retrieved_chunks = (
                    await vector_search_service.search_with_selected_text(
                        query=query_for_search,  # ✅ Use expanded query
                        selected_text=selected_text, 
                        top_k=self.max_chunks
                    )
                )
            else:
                retrieved_chunks = await vector_search_service.search(
                    query=query_for_search,  # ✅ Use expanded query
                    top_k=self.max_chunks
                )
            self.logger.debug(f"Retrieved {len(retrieved_chunks)} relevant chunks")

            # ✅ Handle zero results with helpful guidance
            if len(retrieved_chunks) == 0:
                self.logger.warning(f"No relevant chunks found for query: '{original_query}'")
                
                helpful_response = (
                    f"I couldn't find specific information about \"{original_query}\" in the textbook sections I have access to.\n\n"
                    "Here are some ways I can help:\n"
                    "• Try asking a more specific question (e.g., 'What are the key components of robot kinematics?')\n"
                    "• Ask about related topics in Physical AI and Robotics\n"
                    "• Browse the textbook chapters and select text to ask about specific sections\n\n"
                    "Some topics I can help with:\n"
                    "• Physical AI fundamentals\n"
                    "• Robot sensors and actuators\n"
                    "• Kinematics and dynamics\n"
                    "• Control systems\n"
                    "• Computer vision for robotics\n\n"
                    "What would you like to explore?"
                )
                
                # Still save to conversation if ID exists
                if conversation_id:
                    await postgres_service.create_message(
                        conversation_id=conversation_id, role="user", content=original_query
                    )
                    await postgres_service.create_message(
                        conversation_id=conversation_id,
                        role="assistant",
                        content=helpful_response,
                        sources=[],
                    )
                else:
                    # Create new conversation
                    new_conv = await postgres_service.create_conversation(
                        user_id=user_id,
                        title=original_query[:50],
                    )
                    conversation_id = new_conv["id"]
                    await postgres_service.create_message(
                        conversation_id=conversation_id, role="user", content=original_query
                    )
                    await postgres_service.create_message(
                        conversation_id=conversation_id,
                        role="assistant",
                        content=helpful_response,
                        sources=[],
                    )
                
                return {
                    "response": helpful_response,
                    "conversation_id": conversation_id,
                    "sources": [],
                    "retrieval_time": time.time() - start_time,
                    "retrieved_chunks_count": 0,
                }

            # Step 3: Get recent conversation context if available
            conversation_context = []
            if conversation_id:
                recent_messages = await postgres_service.get_recent_messages(
                    conversation_id=conversation_id,
                    limit=6,  # Get last 6 messages for context
                )
                # Format messages for context
                for msg in recent_messages:
                    role = "User" if msg["role"] == "user" else "Assistant"
                    conversation_context.append(f"{role}: {msg['content']}")

            # Step 4: Prepare context for the LLM
            context_parts = []

            # Add selected text context if provided
            if selected_text:
                context_parts.append(
                    f"USER SELECTED TEXT FOR CONTEXT:\n{selected_text}\n"
                )

            # Add retrieved document chunks
            if retrieved_chunks:
                context_parts.append("RETRIEVED DOCUMENT CONTEXT:")
                for i, chunk in enumerate(retrieved_chunks):
                    context_parts.append(
                        f"[Source {i+1}] Module: {chunk['module']}, "
                        f"Chapter: {chunk['chapter']}, "
                        f"File: {chunk['source_file']}\n"
                        f"Content: {chunk['content'][:500]}..."  # Truncate long content
                    )
                context_parts.append("")  # Empty line for separation

            # Add conversation history if available
            if conversation_context:
                context_parts.append("CONVERSATION HISTORY:")
                context_parts.extend(conversation_context)
                context_parts.append("")  # Empty line for separation

            # Step 5: Create system prompt with context
            system_prompt = self._create_system_prompt(context_parts)

            # Step 6: Generate response using the LLM
            response = await self._generate_llm_response(
                system_prompt=system_prompt, user_query=original_query  # ✅ Use original query
            )

            # Step 7: Create source citations
            sources = self._create_source_citations(retrieved_chunks)

            # Step 8: Save the interaction to conversation history if conversation_id provided
            if conversation_id:
                # Save user message
                await postgres_service.create_message(
                    conversation_id=conversation_id, role="user", content=original_query
                )
                # Save assistant response
                await postgres_service.create_message(
                    conversation_id=conversation_id,
                    role="assistant",
                    content=response,
                    sources=sources,
                )
            else:
                # Create new conversation
                new_conv = await postgres_service.create_conversation(
                    user_id=user_id,
                    title=original_query[:50],  # Use first 50 chars of query as title
                )
                conversation_id = new_conv["id"]

                # Save messages to new conversation
                await postgres_service.create_message(
                    conversation_id=conversation_id, role="user", content=original_query
                )
                await postgres_service.create_message(
                    conversation_id=conversation_id,
                    role="assistant",
                    content=response,
                    sources=sources,
                )

            duration = time.time() - start_time
            self.logger.info(f"Response generated in {duration:.2f}s")

            return {
                "response": response,
                "conversation_id": conversation_id,
                "sources": sources,
                "retrieval_time": duration,
                "retrieved_chunks_count": len(retrieved_chunks),
            }

        except Exception as e:
            # Log the error with context
            error_msg = f"Error generating response: {str(e)}"
            self.logger.error(error_msg, exc_info=True)

            # Log performance metrics even for failures
            duration = time.time() - start_time
            self.logger.info(f"Response generation failed in {duration:.2f}s")

            # ✅ Better fallback response
            fallback_response = (
                "I apologize, but I'm having trouble accessing the textbook information right now. "
                "\n\nHere are some things you can try:\n"
                "• Make sure your question is specific (e.g., 'What are the types of robot sensors?')\n"
                "• Try asking about a related topic\n"
                "• Check if the topic is covered in the textbook chapters\n\n"
                "What would you like to know more about?"
            )

            # Return a user-friendly error message
            return {
                "response": fallback_response,
                "conversation_id": conversation_id,
                "sources": [],
                "error": str(e),
            }

    def _create_system_prompt(self, context_parts: List[str]) -> str:
        """
        Create a system prompt that includes the context and instructions.

        Args:
            context_parts: List of context parts to include

        Returns:
            Formatted system prompt
        """
        system_prompt = (
            "You are an expert AI tutor for the Physical AI & Robotics textbook. "
            "Your role is to help students understand complex concepts in an engaging, clear, and professional manner.\n\n"
        )

        # Add context parts
        for part in context_parts:
            system_prompt += part + "\n"

        system_prompt += (
            "\nINSTRUCTIONS:\n"
            "1. If the user asks about a topic/heading without specific questions:\n"
            "   - Provide a comprehensive overview of that topic\n"
            "   - Break down key concepts in an easy-to-understand way\n"
            "   - Give examples when helpful\n"
            "   - Suggest what they might want to explore further\n\n"
            
            "2. If context is limited but you know the general topic:\n"
            "   - Acknowledge what you found in the textbook\n"
            "   - Provide helpful general information about the topic area\n"
            "   - Guide them to ask more specific questions\n"
            "   - Never just say 'I cannot answer' without offering guidance\n\n"
            
            "3. Communication style:\n"
            "   - Be conversational, encouraging, and professional\n"
            "   - Avoid robotic phrases like 'the provided context'\n"
            "   - Instead use 'In the textbook...' or 'According to the course materials...'\n"
            "   - Break down complex topics into digestible explanations\n\n"
            
            "4. Always cite sources using [Source X] format when information comes from retrieved documents\n\n"
            
            "5. If the user selected specific text, prioritize addressing that content first\n\n"
            
            "6. Format responses clearly with paragraphs and bullet points when appropriate\n\n"
            
            "Remember: You're here to make learning easier and more engaging!\n\n"
            "Now, please respond to the user's query:"
        )

        return system_prompt

    async def _generate_llm_response(self, system_prompt: str, user_query: str) -> str:
        """
        Generate response using the LLM with proper error handling and retry logic.

        Args:
            system_prompt: System prompt with context
            user_query: User's query

        Returns:
            Generated response text
        """
        max_retries = 3
        retry_count = 0

        while retry_count < max_retries:
            try:
                # Create the content for the model
                contents = [
                    {"role": "user", "parts": [system_prompt]},
                    {
                        "role": "model",
                        "parts": [
                            "Understood. I will follow your instructions and respond based only on the provided context."
                        ],
                    },
                    {"role": "user", "parts": [user_query]},
                ]

                # Generate content with specified parameters
                response = await self.model.generate_content_async(
                    contents,
                    generation_config={
                        "temperature": self.temperature,
                        "max_output_tokens": self.max_tokens,
                        "candidate_count": 1,
                    },
                )

                # Extract text from response
                if response.candidates and response.candidates[0].content.parts:
                    text_response = ""
                    for part in response.candidates[0].content.parts:
                        if hasattr(part, "text"):
                            text_response += part.text
                    return text_response.strip()
                else:
                    return "I couldn't generate a response. Please try rephrasing your question."

            except Exception as e:
                retry_count += 1
                self.logger.warning(
                    f"LLM generation attempt {retry_count} failed: {str(e)}"
                )

                if retry_count < max_retries:
                    # Wait before retrying (exponential backoff)
                    await asyncio.sleep(2**retry_count)
                    continue
                else:
                    # All retries failed
                    self.logger.error(
                        f"LLM generation failed after {max_retries} attempts: {str(e)}"
                    )
                    return "I'm currently experiencing difficulties generating a response. Please try again in a moment."

    def _create_source_citations(
        self, retrieved_chunks: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Create source citations from retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved chunks from Qdrant

        Returns:
            List of source citation dictionaries
        """
        citations = []
        for i, chunk in enumerate(retrieved_chunks):
            citation = {
                "source_file": chunk.get("source_file", ""),
                "module": chunk.get("module", ""),
                "chapter": chunk.get("chapter", ""),
                "chunk_index": chunk.get("chunk_index", 0),
                "relevance_score": chunk.get("relevance_score", 0.0),
                "content": (
                    chunk.get("content", "")[:200] + "..."
                    if len(chunk.get("content", "")) > 200
                    else chunk.get("content", "")
                ),
            }
            citations.append(citation)

        return citations

    async def validate_query(self, query: str) -> Dict[str, Any]:
        """
        Validate a query before processing.

        Args:
            query: Query to validate

        Returns:
            Dictionary with validation results
        """
        validation_result = {"is_valid": True, "errors": [], "warnings": []}

        # Check query length
        if len(query.strip()) == 0:
            validation_result["is_valid"] = False
            validation_result["errors"].append("Query cannot be empty")
        elif len(query.strip()) < 3:
            validation_result["warnings"].append("Query is very short (< 3 characters)")
        elif len(query.strip()) > 2000:
            validation_result["is_valid"] = False
            validation_result["errors"].append("Query is too long (> 2000 characters)")

        # Check for potentially problematic content
        if re.search(r"(joke|fun|tell me|make me laugh)", query, re.IGNORECASE):
            validation_result["warnings"].append(
                "Query appears to be requesting non-textbook content"
            )

        return validation_result

    async def get_conversation_context(
        self, conversation_id: str, limit: int = 6
    ) -> List[Dict[str, Any]]:
        """
        Get conversation context for a given conversation.

        Args:
            conversation_id: ID of the conversation
            limit: Number of recent messages to retrieve

        Returns:
            List of recent messages in the conversation
        """
        try:
            messages = await postgres_service.get_recent_messages(
                conversation_id=conversation_id, limit=limit
            )
            return messages
        except Exception as e:
            self.logger.error(f"Error retrieving conversation context: {str(e)}")
            return []

    async def generate_embedding_for_query(self, query: str) -> List[float]:
        """
        Generate embedding specifically for a query (for external use).

        Args:
            query: Query text to embed

        Returns:
            Embedding vector
        """
        return await embedding_service.generate_query_embedding(query)


# Global service instance
agent_service = AgentService()