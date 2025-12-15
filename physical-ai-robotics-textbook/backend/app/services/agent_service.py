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
            # Detect simple greetings (only very basic ones)
            greeting_patterns = [
                r'^(hi|hello|hey)[\s\!\?]*$',
                r'^(good\s+(morning|afternoon|evening))[\s\!\?]*$',
            ]
            
            query_lower = query.strip().lower()
            is_simple_greeting = any(re.match(pattern, query_lower, re.IGNORECASE) for pattern in greeting_patterns)
            
            # Handle only simple greetings briefly
            if is_simple_greeting:
                self.logger.info(f"Detected simple greeting: '{query}'")
                
                casual_response = (
                    "Welcome to the Physical AI & Robotics Textbook Assistant. "
                    "I can help you understand concepts from the course material. "
                    "Please ask your question about any topic covered in the textbook."
                )
                
                # Save to conversation
                if not conversation_id:
                    new_conv = await postgres_service.create_conversation(
                        user_id=user_id,
                        title="New Session",
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
            
            # Preprocess query for better retrieval
            original_query = query
            query_for_search = query
            
            # Expand short topic queries
            if len(query.split()) <= 5 and not query.endswith('?') and not selected_text:
                query_for_search = f"Explain {query}. What is {query}? Key concepts and details about {query}."
                self.logger.info(f"Expanded query for better retrieval: '{query}' -> '{query_for_search}'")
            
            # Retrieve relevant chunks from vector database
            if selected_text:
                retrieved_chunks = (
                    await vector_search_service.search_with_selected_text(
                        query=query_for_search,
                        selected_text=selected_text, 
                        top_k=self.max_chunks
                    )
                )
            else:
                retrieved_chunks = await vector_search_service.search(
                    query=query_for_search,
                    top_k=self.max_chunks
                )
            
            self.logger.debug(f"Retrieved {len(retrieved_chunks)} relevant chunks")

            # Handle insufficient results
            if len(retrieved_chunks) == 0:
                self.logger.warning(f"No relevant chunks found for query: '{original_query}'")
                
                helpful_response = (
                    f"The requested information about '{original_query}' was not found in the available textbook sections. "
                    "This may occur if:\n\n"
                    "• The topic is not covered in the current textbook chapters\n"
                    "• The query needs to be more specific\n"
                    "• The terminology differs from what is used in the textbook\n\n"
                    "Please reformulate your question or ask about related topics that are covered in the Physical AI & Robotics curriculum."
                )
                
                # Save to conversation
                if not conversation_id:
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

            # Get conversation context if available
            conversation_context = []
            if conversation_id:
                recent_messages = await postgres_service.get_recent_messages(
                    conversation_id=conversation_id,
                    limit=4,  # Keep context concise
                )
                for msg in recent_messages:
                    role = "User" if msg["role"] == "user" else "Assistant"
                    conversation_context.append(f"{role}: {msg['content']}")

            # Prepare context for LLM
            context_parts = []

            if selected_text:
                context_parts.append(
                    f"SELECTED TEXT CONTEXT:\n{selected_text}\n"
                )

            if retrieved_chunks:
                context_parts.append("TEXTBOOK REFERENCE MATERIAL:")
                for i, chunk in enumerate(retrieved_chunks):
                    context_parts.append(
                        f"\n[Reference {i+1}]\n"
                        f"Source: Module {chunk['module']}, Chapter {chunk['chapter']}, File: {chunk['source_file']}\n"
                        f"Content: {chunk['content'][:600]}..."
                    )
                context_parts.append("")

            if conversation_context:
                context_parts.append("PREVIOUS CONVERSATION:")
                context_parts.extend(conversation_context[-4:])  # Last 4 messages only
                context_parts.append("")

            # Create system prompt
            system_prompt = self._create_system_prompt(context_parts)

            # Generate response using LLM
            response = await self._generate_llm_response(
                system_prompt=system_prompt, user_query=original_query
            )

            # Create source citations
            sources = self._create_source_citations(retrieved_chunks)

            # Save conversation
            if not conversation_id:
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
            error_msg = f"Error generating response: {str(e)}"
            self.logger.error(error_msg, exc_info=True)

            duration = time.time() - start_time
            self.logger.info(f"Response generation failed in {duration:.2f}s")

            fallback_response = (
                "An error occurred while processing your request. "
                "Please try rephrasing your question or contact support if the issue persists."
            )

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
            "You are an expert educational assistant for the Physical AI & Robotics textbook. "
            "Your purpose is to provide accurate, comprehensive, and well-structured responses based on the course material.\n\n"
        )

        # Add context parts
        for part in context_parts:
            system_prompt += part + "\n"

        system_prompt += (
            "\nRESPONSE GUIDELINES:\n\n"
            
            "1. CONTENT ACCURACY:\n"
            "   - Base your responses strictly on the provided textbook material\n"
            "   - Use information from the reference material to construct comprehensive answers\n"
            "   - Cite sources using [Reference X] notation when drawing from specific documents\n"
            "   - If the reference material lacks sufficient detail, acknowledge this professionally\n\n"
            
            "2. RESPONSE STRUCTURE:\n"
            "   - Provide clear, well-organized explanations\n"
            "   - Use proper paragraphs and logical flow\n"
            "   - Include relevant examples and details from the textbook\n"
            "   - Break down complex concepts into understandable components\n\n"
            
            "3. PROFESSIONAL TONE:\n"
            "   - Maintain a formal, academic tone appropriate for university-level education\n"
            "   - Avoid casual language, emojis, or overly friendly expressions\n"
            "   - Do not use phrases like 'I apologize' or 'I'm having trouble'\n"
            "   - Frame limitations professionally: 'The available material does not cover...' rather than 'I cannot...'\n\n"
            
            "4. HANDLING QUESTIONS:\n"
            "   - For topic queries: Provide comprehensive overviews with key concepts and details\n"
            "   - For specific questions: Give precise, well-supported answers\n"
            "   - For broad questions: Structure information logically with appropriate depth\n"
            "   - For out-of-scope queries: State professionally that the topic is not covered in the current material\n\n"
            
            "5. PERSONAL INFORMATION:\n"
            "   - Do not engage with personal questions about the user\n"
            "   - Redirect focus to the educational material\n"
            "   - Maintain professional boundaries\n\n"
            
            "6. IDENTITY QUESTIONS:\n"
            "   - If asked who you are: 'I am an educational assistant designed to help you understand the Physical AI & Robotics textbook content.'\n"
            "   - If asked about your purpose: 'My role is to provide accurate information and explanations based on the course material.'\n"
            "   - Keep responses concise and redirect to the educational content\n\n"
            
            "Now respond to the user's query based on these guidelines and the provided reference material."
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
                contents = [
                    {"role": "user", "parts": [system_prompt]},
                    {
                        "role": "model",
                        "parts": [
                            "Understood. I will provide accurate, professional responses based on the textbook material provided."
                        ],
                    },
                    {"role": "user", "parts": [user_query]},
                ]

                response = await self.model.generate_content_async(
                    contents,
                    generation_config={
                        "temperature": self.temperature,
                        "max_output_tokens": self.max_tokens,
                        "candidate_count": 1,
                    },
                )

                if response.candidates and response.candidates[0].content.parts:
                    text_response = ""
                    for part in response.candidates[0].content.parts:
                        if hasattr(part, "text"):
                            text_response += part.text
                    return text_response.strip()
                else:
                    return "Unable to generate a response. Please reformulate your question."

            except Exception as e:
                retry_count += 1
                self.logger.warning(
                    f"LLM generation attempt {retry_count} failed: {str(e)}"
                )

                if retry_count < max_retries:
                    await asyncio.sleep(2**retry_count)
                    continue
                else:
                    self.logger.error(
                        f"LLM generation failed after {max_retries} attempts: {str(e)}"
                    )
                    return "The system is currently unavailable. Please try again later."

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

        if len(query.strip()) == 0:
            validation_result["is_valid"] = False
            validation_result["errors"].append("Query cannot be empty")
        elif len(query.strip()) < 2:
            validation_result["warnings"].append("Query is very short")
        elif len(query.strip()) > 2000:
            validation_result["is_valid"] = False
            validation_result["errors"].append("Query exceeds maximum length")

        return validation_result

    async def get_conversation_context(
        self, conversation_id: str, limit: int = 4
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