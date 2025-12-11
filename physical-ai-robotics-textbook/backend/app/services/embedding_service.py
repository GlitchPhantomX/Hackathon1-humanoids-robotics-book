"""
Embedding service for generating text embeddings using Google's text embedding models.
"""

from typing import List, Union
import google.generativeai as genai
from app.config import settings
from app.utils.logger import get_logger
import time


class EmbeddingService:
    """
    Service class for generating text embeddings using Google's embedding models.
    """

    def __init__(self):
        self.api_key = settings.google_api_key
        self.model_name = settings.embedding_model
        self.logger = get_logger("embedding_service")

        # Configure the Google Generative AI API
        genai.configure(api_key=self.api_key)

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for the given text.

        Args:
            text: Input text to generate embedding for

        Returns:
            List of float values representing the embedding vector
        """
        try:
            start_time = time.time()

            # Generate embedding using Google's embedding API
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="RETRIEVAL_DOCUMENT",  # Optimal for document retrieval
            )

            embedding = result["embedding"]

            duration = time.time() - start_time
            self.logger.info(
                f"Generated embedding for text of length {len(text)} in {duration:.2f}s"
            )

            return embedding

        except Exception as e:
            self.logger.error(f"Error generating embedding: {str(e)}")
            raise

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of input texts to generate embeddings for

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        embeddings = []

        for i, text in enumerate(texts):
            try:
                embedding = await self.generate_embedding(text)
                embeddings.append(embedding)

                # Log progress for larger batches
                if len(texts) > 5:
                    self.logger.info(f"Processed {i+1}/{len(texts)} embeddings")

            except Exception as e:
                self.logger.error(f"Error generating embedding for text {i}: {str(e)}")
                # Return a zero vector in case of error, or raise exception
                # For now, we'll raise to be strict about quality
                raise

        return embeddings

    async def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate an embedding for a query (optimized for retrieval).

        Args:
            query: Query text to generate embedding for

        Returns:
            List of float values representing the embedding vector
        """
        try:
            start_time = time.time()

            # Generate embedding optimized for query/retrieval tasks
            result = genai.embed_content(
                model=self.model_name,
                content=query,
                task_type="RETRIEVAL_QUERY",  # Optimal for query retrieval
            )

            embedding = result["embedding"]

            duration = time.time() - start_time
            self.logger.info(
                f"Generated query embedding for '{query[:50]}...' in {duration:.2f}s"
            )

            return embedding

        except Exception as e:
            self.logger.error(f"Error generating query embedding: {str(e)}")
            raise

    def get_model_info(self) -> dict:
        """
        Get information about the embedding model being used.

        Returns:
            Dictionary with model information
        """
        try:
            # Get the model details
            model = genai.get_model(self.model_name)
            return {
                "name": model.name,
                "version": getattr(model, "version", "unknown"),
                "display_name": getattr(model, "display_name", self.model_name),
                "description": getattr(
                    model, "description", "No description available"
                ),
            }
        except Exception as e:
            self.logger.error(f"Error getting model info: {str(e)}")
            return {"name": self.model_name, "error": str(e)}

    async def validate_text_length(self, text: str, max_length: int = 30720) -> bool:
        """
        Validate if text length is within the model's limits.

        Args:
            text: Text to validate
            max_length: Maximum allowed length (default based on Google's limits)

        Returns:
            True if text length is acceptable, False otherwise
        """
        # Google's embedding models can handle quite large texts
        # But we'll set a reasonable limit to avoid issues
        if len(text) > max_length:
            self.logger.warning(
                f"Text length {len(text)} exceeds recommended limit {max_length}"
            )
            return False
        return True

    async def embed_document_chunk(self, content: str, metadata: dict = None) -> dict:
        """
        Generate embedding for a document chunk with associated metadata.

        Args:
            content: Content of the document chunk
            metadata: Optional metadata about the chunk

        Returns:
            Dictionary containing embedding and metadata
        """
        if metadata is None:
            metadata = {}

        # Validate content length
        if not await self.validate_text_length(content):
            raise ValueError(f"Document chunk too long: {len(content)} characters")

        embedding = await self.generate_embedding(content)

        return {
            "embedding": embedding,
            "content": content,
            "metadata": metadata,
            "content_length": len(content),
        }


# Global service instance
embedding_service = EmbeddingService()
