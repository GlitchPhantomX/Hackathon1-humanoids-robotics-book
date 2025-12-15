from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging
import os
from contextlib import asynccontextmanager
import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import openai
import uuid
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize global variables
qdrant_client = None
embedding_model = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize resources on startup and cleanup on shutdown."""
    global qdrant_client, embedding_model

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_client = QdrantClient(url=qdrant_url)

    # Initialize embedding model
    embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

    # Create collection if it doesn't exist
    try:
        qdrant_client.get_collection("documents")
    except:
        qdrant_client.create_collection(
            collection_name="documents",
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
        )

    logger.info("RAG system initialized")
    yield
    logger.info("RAG system shutdown")

# Initialize FastAPI app
app = FastAPI(
    title="RAG System API",
    description="Retrieval-Augmented Generation API for document-based question answering",
    version="1.0.0",
    lifespan=lifespan
)

# Pydantic models
class Document(BaseModel):
    id: Optional[str] = None
    content: str
    metadata: Optional[Dict[str, Any]] = {}
    source: Optional[str] = ""

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    filters: Optional[Dict[str, Any]] = {}

class QueryResponse(BaseModel):
    query: str
    results: List[Dict[str, Any]]
    answer: str

class DocumentIngestionRequest(BaseModel):
    documents: List[Document]
    collection_name: str = "documents"

# API endpoints
@app.post("/ingest", summary="Ingest documents into the RAG system")
async def ingest_documents(request: DocumentIngestionRequest):
    """Ingest documents into the vector database."""
    global qdrant_client, embedding_model

    try:
        points = []
        for doc in request.documents:
            # Generate embedding for the document content
            embedding = embedding_model.encode(doc.content).tolist()

            # Create a point for Qdrant
            point = models.PointStruct(
                id=doc.id or str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "content": doc.content,
                    "metadata": doc.metadata,
                    "source": doc.source,
                    "timestamp": datetime.utcnow().isoformat()
                }
            )
            points.append(point)

        # Upload points to Qdrant
        qdrant_client.upsert(
            collection_name=request.collection_name,
            points=points
        )

        logger.info(f"Ingested {len(points)} documents")
        return {"status": "success", "count": len(points)}

    except Exception as e:
        logger.error(f"Error ingesting documents: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/query", summary="Query the RAG system")
async def query_rag(request: QueryRequest):
    """Query the RAG system and get a response with retrieved context."""
    global qdrant_client, embedding_model

    try:
        # Generate embedding for the query
        query_embedding = embedding_model.encode(request.query).tolist()

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name="documents",
            query_vector=query_embedding,
            limit=request.top_k,
            query_filter=None  # Add filters if needed
        )

        # Extract content from search results
        retrieved_docs = []
        for result in search_results:
            retrieved_docs.append({
                "id": result.id,
                "content": result.payload.get("content", ""),
                "metadata": result.payload.get("metadata", {}),
                "source": result.payload.get("source", ""),
                "score": result.score
            })

        # Generate response using OpenAI (simulated here)
        # In a real implementation, you would use the retrieved docs as context
        context = " ".join([doc["content"] for doc in retrieved_docs[:3]])  # Use top 3 docs
        answer = f"Based on the provided context: {context[:500]}... Answer to query '{request.query}' would be generated here."

        response = QueryResponse(
            query=request.query,
            results=retrieved_docs,
            answer=answer
        )

        logger.info(f"Processed query: {request.query}")
        return response

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health", summary="Health check endpoint")
async def health_check():
    """Health check endpoint to verify the RAG system is running."""
    return {"status": "healthy", "timestamp": datetime.utcnow().isoformat()}

@app.get("/stats", summary="Get RAG system statistics")
async def get_stats():
    """Get statistics about the RAG system."""
    global qdrant_client

    try:
        collection_info = qdrant_client.get_collection("documents")
        stats = {
            "vectors_count": collection_info.points_count,
            "indexed_vectors_count": collection_info.indexed_vectors_count,
            "collection_name": "documents"
        }
        return stats
    except Exception as e:
        logger.error(f"Error getting stats: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

# Example usage and testing
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)