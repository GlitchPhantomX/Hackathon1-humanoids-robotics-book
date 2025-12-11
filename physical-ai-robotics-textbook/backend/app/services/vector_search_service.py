"""
Vector search service for implementing query embedding generation and similarity search.
"""

from typing import List, Dict, Any, Optional
from app.services.embedding_service import embedding_service
from app.services.qdrant_service import qdrant_service
from app.utils.logger import get_logger
import time


class VectorSearchService:
    """
    Service class for vector search functionality.
    Handles query embedding generation and similarity search in Qdrant.
    """

    def __init__(self):
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        self.logger = get_logger("vector_search_service")

    async def search(
        self,
        query: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None,
        selected_text: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        Perform vector search with the given query.

        Args:
            query: The search query text
            top_k: Number of top results to return (default 5, configurable)
            filters: Optional filters for metadata (e.g., {"module": "kinematics"})
            selected_text: Optional selected text to boost relevance

        Returns:
            List of relevant chunks with content, metadata, and relevance scores
        """
        start_time = time.time()

        # Validate inputs
        if not query or len(query.strip()) < 3:
            raise ValueError("Query must be at least 3 characters long")

        # Generate embedding for the query using the same model as documents
        query_embedding = await embedding_service.generate_query_embedding(query)

        # If selected text is provided, we might want to combine it with the query
        if selected_text:
            # For now, we'll just log that we received selected text
            # In future, we might combine the query and selected text for enhanced search
            self.logger.info(
                f"Processing query with selected text: '{selected_text[:50]}...'"
            )

        # Perform similarity search in Qdrant
        search_results = await qdrant_service.search_similar(
            query_embedding=query_embedding, top_k=top_k, filters=filters
        )

        # Ensure we return only top_k results in case Qdrant returns more
        if len(search_results) > top_k:
            search_results = search_results[:top_k]

        # Add relevance scores (already included from Qdrant as cosine similarity)
        for result in search_results:
            # Ensure relevance score is between 0 and 1
            score = result.get("relevance_score", 0.0)
            # Qdrant cosine similarity is already between -1 and 1, but should be 0-1 for our use case
            # In practice, cosine similarity for text embeddings is usually 0-1
            result["relevance_score"] = max(0.0, min(1.0, float(score)))

        duration = time.time() - start_time
        self.logger.info(
            f"Vector search completed in {duration:.2f}s, returned {len(search_results)} results"
        )

        # Check performance target
        if duration > 0.5:
            self.logger.warning(f"Search took {duration:.2f}s, exceeding 500ms target")

        return search_results

    async def search_with_selected_text(
        self,
        query: str,
        selected_text: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        """
        Perform vector search with special handling for selected text.

        Args:
            query: The search query text
            selected_text: The selected text to boost relevance
            top_k: Number of top results to return (default 5, configurable)
            filters: Optional filters for metadata

        Returns:
            List of relevant chunks with content, metadata, and relevance scores
        """
        # For now, implement a basic approach where we enhance the query with selected text context
        # In the future, we might want to implement more sophisticated approaches like:
        # 1. Dual encoding (separate embeddings for query and selected text)
        # 2. Cross-attention mechanisms
        # 3. Re-ranking based on selected text similarity

        enhanced_query = f"{query} [CONTEXT: {selected_text}]"

        self.logger.info(
            f"Searching with enhanced query combining user query and selected text"
        )
        return await self.search(
            query=enhanced_query,
            top_k=top_k,
            filters=filters,
            selected_text=selected_text,
        )

    async def validate_search_query(self, query: str) -> bool:
        """
        Validate if a search query is appropriate for processing.

        Args:
            query: The search query to validate

        Returns:
            True if query is valid, False otherwise
        """
        if not query:
            return False

        # Check minimum length
        if len(query.strip()) < 3:
            self.logger.warning(f"Query too short: '{query}'")
            return False

        # Check maximum length to prevent abuse
        if len(query) > 500:
            self.logger.warning(f"Query too long: {len(query)} characters")
            return False

        # Check for nonsensical input (too many special characters relative to text)
        text_chars = sum(1 for c in query if c.isalnum() or c.isspace())
        if len(query) > 0 and text_chars / len(query) < 0.3:
            self.logger.warning(f"Query appears nonsensical: '{query[:50]}...'")
            return False

        return True

    async def search_by_module_or_chapter(
        self,
        query: str,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        top_k: int = 5,
    ) -> List[Dict[str, Any]]:
        """
        Search with optional filtering by module or chapter.

        Args:
            query: The search query text
            module: Optional module to filter by
            chapter: Optional chapter to filter by
            top_k: Number of top results to return (default 5, configurable)

        Returns:
            List of relevant chunks with content, metadata, and relevance scores
        """
        filters = {}
        if module:
            filters["module"] = module
        if chapter:
            filters["chapter"] = chapter

        return await self.search(
            query=query, top_k=top_k, filters=filters if filters else None
        )

    async def get_search_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the search system.

        Returns:
            Dictionary with search system statistics
        """
        collection_info = await qdrant_service.get_collection_info()

        return {
            "total_documents": collection_info.get("point_count", 0),
            "vector_size": collection_info.get("vector_size", 0),
            "collection_name": collection_info.get("name", ""),
            "status": (
                "healthy"
                if collection_info.get("point_count") is not None
                else "unavailable"
            ),
        }


# Global service instance
vector_search_service = VectorSearchService()
