"""
Vector search service for querying Qdrant vector database.
Handles semantic search and retrieval of relevant document chunks.
"""

from typing import List, Dict, Any, Optional
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.config import settings
from app.utils.logger import get_logger


class VectorSearchService:
    """
    Service for performing vector similarity search in Qdrant.
    """

    def __init__(self):
        self.logger = get_logger("vector_search_service")
        self.min_score = 0.25  # Lowered from default (was probably 0.7)
        
    async def search(
        self,
        query: str,
        top_k: int = 5,
        min_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search for relevant document chunks.

        Args:
            query: Search query text
            top_k: Number of results to return
            min_score: Minimum relevance score (0.0 to 1.0)

        Returns:
            List of retrieved chunks with metadata
        """
        try:
            self.logger.info(f"Searching for: '{query}' (top_k={top_k})")
            
            # Use provided min_score or default
            score_threshold = min_score if min_score is not None else self.min_score
            
            # Generate embedding for the query
            query_embedding = await embedding_service.generate_query_embedding(query)
            self.logger.debug(f"Generated query embedding (dim: {len(query_embedding)})")
            
            # Search in Qdrant
            search_results = await qdrant_service.search(
                query_vector=query_embedding,
                top_k=top_k * 2,  # Get more results then filter
                score_threshold=0.0  # Get all results, filter later
            )
            
            self.logger.info(f"Qdrant returned {len(search_results)} initial results")
            
            # Process and format results
            formatted_results = []
            for result in search_results:
                # Apply score threshold after retrieval
                if result['relevance_score'] < score_threshold:
                    continue
                    
                formatted_result = {
                    'id': result.get('id'),
                    'content': result.get('content', ''),
                    'source_file': result.get('source_file', ''),
                    'module': result.get('module', ''),
                    'chapter': result.get('chapter', ''),
                    'chunk_index': result.get('chunk_index', 0),
                    'relevance_score': result.get('relevance_score', 0.0),
                }
                formatted_results.append(formatted_result)
            
            # Limit to top_k after filtering
            formatted_results = formatted_results[:top_k]
            
            self.logger.info(f"Returning {len(formatted_results)} results after filtering (score >= {score_threshold})")
            
            # Log top results for debugging
            if formatted_results:
                for i, res in enumerate(formatted_results[:3], 1):
                    self.logger.debug(
                        f"Result {i}: score={res['relevance_score']:.3f}, "
                        f"chapter={res['chapter']}, "
                        f"content={res['content'][:50]}..."
                    )
            else:
                self.logger.warning(f"No results found for query: '{query}'")
            
            return formatted_results
            
        except Exception as e:
            self.logger.error(f"Error in vector search: {str(e)}", exc_info=True)
            return []

    async def search_with_selected_text(
        self,
        query: str,
        selected_text: str,
        top_k: int = 5,
        min_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform enhanced search using both query and selected text context.

        Args:
            query: User's query
            selected_text: Text selected by user for context
            top_k: Number of results to return
            min_score: Minimum relevance score

        Returns:
            List of retrieved chunks with metadata
        """
        try:
            self.logger.info(f"Enhanced search with selected text (length: {len(selected_text)})")
            
            # Combine query with selected text for better context
            enhanced_query = f"{query}\n\nContext: {selected_text[:500]}"
            
            # Use regular search with enhanced query
            results = await self.search(
                query=enhanced_query,
                top_k=top_k,
                min_score=min_score
            )
            
            return results
            
        except Exception as e:
            self.logger.error(f"Error in enhanced search: {str(e)}", exc_info=True)
            # Fallback to regular search
            return await self.search(query=query, top_k=top_k, min_score=min_score)

    async def search_by_filters(
        self,
        query: str,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        top_k: int = 5,
        min_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Search with metadata filters (module, chapter).

        Args:
            query: Search query
            module: Filter by module name
            chapter: Filter by chapter name
            top_k: Number of results
            min_score: Minimum relevance score

        Returns:
            Filtered search results
        """
        try:
            self.logger.info(f"Filtered search: module={module}, chapter={chapter}")
            
            # Generate query embedding
            query_embedding = await embedding_service.generate_query_embedding(query)
            
            # Build filter conditions
            filter_conditions = []
            if module:
                filter_conditions.append({
                    "key": "module",
                    "match": {"value": module}
                })
            if chapter:
                filter_conditions.append({
                    "key": "chapter",
                    "match": {"value": chapter}
                })
            
            # Search with filters
            search_results = await qdrant_service.search(
                query_vector=query_embedding,
                top_k=top_k,
                score_threshold=min_score or self.min_score,
                filters=filter_conditions if filter_conditions else None
            )
            
            # Format results
            formatted_results = []
            for result in search_results:
                formatted_result = {
                    'id': result.get('id'),
                    'content': result.get('content', ''),
                    'source_file': result.get('source_file', ''),
                    'module': result.get('module', ''),
                    'chapter': result.get('chapter', ''),
                    'chunk_index': result.get('chunk_index', 0),
                    'relevance_score': result.get('relevance_score', 0.0),
                }
                formatted_results.append(formatted_result)
            
            self.logger.info(f"Filtered search returned {len(formatted_results)} results")
            return formatted_results
            
        except Exception as e:
            self.logger.error(f"Error in filtered search: {str(e)}", exc_info=True)
            return []

    async def multi_query_search(
        self,
        queries: List[str],
        top_k: int = 5,
        min_score: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform search with multiple query variations for better coverage.

        Args:
            queries: List of query variations
            top_k: Number of results per query
            min_score: Minimum relevance score

        Returns:
            Deduplicated and ranked results
        """
        try:
            self.logger.info(f"Multi-query search with {len(queries)} variations")
            
            all_results = []
            seen_ids = set()
            
            for query in queries:
                results = await self.search(
                    query=query,
                    top_k=top_k,
                    min_score=min_score
                )
                
                # Deduplicate by ID
                for result in results:
                    result_id = result.get('id')
                    if result_id not in seen_ids:
                        all_results.append(result)
                        seen_ids.add(result_id)
            
            # Sort by relevance score
            all_results.sort(key=lambda x: x['relevance_score'], reverse=True)
            
            # Return top_k overall
            final_results = all_results[:top_k]
            
            self.logger.info(f"Multi-query search returned {len(final_results)} unique results")
            return final_results
            
        except Exception as e:
            self.logger.error(f"Error in multi-query search: {str(e)}", exc_info=True)
            return []


# Global service instance
vector_search_service = VectorSearchService()