"""
Qdrant service for vector storage and similarity search operations.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from app.config import settings
from app.utils.logger import get_logger
import uuid
import time


class QdrantService:
    """
    Service class for Qdrant vector database operations.
    Handles document storage, retrieval, and similarity search.
    """

    def __init__(self):
        if settings.qdrant_url:
            self.client = AsyncQdrantClient(
                url=settings.qdrant_url, api_key=settings.qdrant_api_key, timeout=30
            )
        else:
            raise ValueError(
                "Qdrant URL is required. Please set QDRANT_URL in your environment variables."
            )
        self.collection_name = settings.qdrant_collection_name
        self.logger = get_logger("qdrant_service")

    async def init_collection(self):
        """
        Initialize the Qdrant collection with proper configuration.
        """
        try:
            # Check if collection already exists
            collections = await self.client.get_collections()
            collection_exists = any(
                col.name == self.collection_name for col in collections.collections
            )

            if not collection_exists:
                # Create collection with appropriate vector configuration
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=768,  # Size for text-embedding-004 model
                        distance=models.Distance.COSINE,
                    ),
                    # Configure payload schema for metadata
                    optimizers_config=models.OptimizersConfigDiff(
                        memmap_threshold=20000, indexing_threshold=20000
                    ),
                )
                self.logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                self.logger.info(
                    f"Qdrant collection already exists: {self.collection_name}"
                )

            # Create payload index for efficient filtering
            await self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="module",
                field_schema=models.PayloadSchemaType.KEYWORD,
            )

            await self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chapter",
                field_schema=models.PayloadSchemaType.KEYWORD,
            )

            await self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source_file",
                field_schema=models.PayloadSchemaType.KEYWORD,
            )

            self.logger.info("Qdrant collection initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"Error initializing Qdrant collection: {str(e)}")
            raise

    async def store_embedding(
        self,
        content: str,
        embedding: List[float],
        module: str,
        chapter: str,
        source_file: str,
        chunk_index: int,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> str:
        """
        Store a single document embedding in Qdrant.

        Args:
            content: The original text content
            embedding: The embedding vector
            module: Module name from textbook structure
            chapter: Chapter name from textbook structure
            source_file: Original source file path
            chunk_index: Sequential index of the chunk
            metadata: Additional metadata to store

        Returns:
            ID of the stored vector
        """
        try:
            start_time = time.time()

            if metadata is None:
                metadata = {}

            # Generate a unique ID for the point
            point_id = str(uuid.uuid4())

            # Prepare the payload with all required metadata
            payload = {
                "content": content,
                "module": module,
                "chapter": chapter,
                "source_file": source_file,
                "chunk_index": chunk_index,
                "created_at": time.time(),
                **metadata,  # Include any additional metadata
            }

            # Store the vector in Qdrant
            await self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(id=point_id, vector=embedding, payload=payload)
                ],
            )

            duration = time.time() - start_time
            self.logger.info(
                f"Stored embedding for '{source_file}' chunk {chunk_index} in {duration:.2f}s"
            )

            return point_id

        except Exception as e:
            self.logger.error(f"Error storing embedding: {str(e)}")
            raise

    async def store_embeddings_batch(
        self, documents: List[Dict[str, Any]]
    ) -> List[str]:
        """
        Store multiple document embeddings in Qdrant efficiently.

        Args:
            documents: List of dictionaries containing content, embedding, and metadata

        Returns:
            List of IDs of stored vectors
        """
        try:
            start_time = time.time()
            point_ids = []

            points = []
            for doc in documents:
                point_id = str(uuid.uuid4())
                point_ids.append(point_id)

                payload = {
                    "content": doc["content"],
                    "module": doc["module"],
                    "chapter": doc["chapter"],
                    "source_file": doc["source_file"],
                    "chunk_index": doc["chunk_index"],
                    "created_at": time.time(),
                    **doc.get("metadata", {}),
                }

                points.append(
                    models.PointStruct(
                        id=point_id, vector=doc["embedding"], payload=payload
                    )
                )

            # Batch upsert all points
            await self.client.upsert(
                collection_name=self.collection_name, points=points
            )

            duration = time.time() - start_time
            self.logger.info(
                f"Stored {len(documents)} embeddings in batch in {duration:.2f}s"
            )

            return point_ids

        except Exception as e:
            self.logger.error(f"Error storing embeddings batch: {str(e)}")
            raise

    async def search_similar(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar documents based on embedding similarity.

        Args:
            query_embedding: The embedding vector to search for
            top_k: Number of top results to return
            filters: Optional filters for metadata (e.g., {"module": "kinematics"})

        Returns:
            List of similar documents with content, metadata, and similarity scores
        """
        try:
            start_time = time.time()

            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    if isinstance(value, list):
                        # Handle list of values (OR condition)
                        conditions = [
                            models.FieldCondition(
                                key=key, match=models.MatchValue(value=v)
                            )
                            for v in value
                        ]
                        filter_conditions.append(
                            models.ShouldCondition(should=conditions)
                        )
                    else:
                        # Handle single value
                        filter_conditions.append(
                            models.FieldCondition(
                                key=key, match=models.MatchValue(value=value)
                            )
                        )

                if filter_conditions:
                    qdrant_filters = models.Filter(must=filter_conditions)

            # Perform the search using query_points method (correct for Qdrant client 1.16.1+)
            search_results = await self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False,
                score_threshold=0.0,  # Include all results above 0 similarity
                query_filter=qdrant_filters,
            )

            # Format results
            results = []
            for (
                result
            ) in (
                search_results.points
            ):  # query_points returns SearchResult with points attribute
                results.append(
                    {
                        "id": result.id,
                        "content": result.payload.get("content", ""),
                        "module": result.payload.get("module", ""),
                        "chapter": result.payload.get("chapter", ""),
                        "source_file": result.payload.get("source_file", ""),
                        "chunk_index": result.payload.get("chunk_index", 0),
                        "relevance_score": float(
                            result.score
                        ),  # Cosine similarity score
                        "metadata": {
                            k: v
                            for k, v in result.payload.items()
                            if k
                            not in [
                                "content",
                                "module",
                                "chapter",
                                "source_file",
                                "chunk_index",
                                "created_at",
                            ]
                        },
                    }
                )

            duration = time.time() - start_time
            self.logger.info(
                f"Search completed in {duration:.2f}s, found {len(results)} results"
            )

            return results

        except Exception as e:
            self.logger.error(f"Error searching for similar documents: {str(e)}")
            raise

    async def get_document_by_id(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific document by its ID.

        Args:
            doc_id: ID of the document to retrieve

        Returns:
            Document content and metadata, or None if not found
        """
        try:
            records = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id],
                with_payload=True,
                with_vectors=False,
            )

            if not records:
                return None

            record = records[0]
            payload = record.payload

            return {
                "id": record.id,
                "content": payload.get("content", ""),
                "module": payload.get("module", ""),
                "chapter": payload.get("chapter", ""),
                "source_file": payload.get("source_file", ""),
                "chunk_index": payload.get("chunk_index", 0),
                "metadata": {
                    k: v
                    for k, v in payload.items()
                    if k
                    not in [
                        "content",
                        "module",
                        "chapter",
                        "source_file",
                        "chunk_index",
                        "created_at",
                    ]
                },
            }

        except Exception as e:
            self.logger.error(f"Error retrieving document by ID: {str(e)}")
            return None

    async def delete_document_by_id(self, doc_id: str) -> bool:
        """
        Delete a document by its ID.

        Args:
            doc_id: ID of the document to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[doc_id]),
            )
            self.logger.info(f"Deleted document with ID: {doc_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error deleting document by ID: {str(e)}")
            return False

    async def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution!).

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            await self.client.delete_collection(self.collection_name)
            self.logger.info(f"Deleted collection: {self.collection_name}")
            return True

        except Exception as e:
            self.logger.error(f"Error deleting collection: {str(e)}")
            return False

    async def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information
        """
        try:
            collection_info = await self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance,
                "point_count": collection_info.points_count,
                "config": {
                    "hnsw_config": collection_info.config.hnsw_config,
                    "optimizer_config": collection_info.config.optimizer_config,
                    "wal_config": collection_info.config.wal_config,
                },
            }
        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            return {"error": str(e)}

    async def count_documents(self) -> int:
        """
        Count the total number of documents in the collection.

        Returns:
            Number of documents in the collection
        """
        try:
            collection_info = await self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            self.logger.error(f"Error counting documents: {str(e)}")
            return 0


# Global service instance
qdrant_service = QdrantService()
