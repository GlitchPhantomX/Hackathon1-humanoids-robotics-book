from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uvicorn
import logging
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    filters: Optional[Dict[str, Any]] = None

class QueryResponse(BaseModel):
    results: List[Dict[str, Any]]
    query_time: float
    sources: List[str]

class Document(BaseModel):
    content: str
    metadata: Dict[str, Any]

class DocumentResponse(BaseModel):
    success: bool
    document_id: str
    message: str

# Global variables for services
vector_db_client = None
embedding_model = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup and cleanup on shutdown."""
    global vector_db_client, embedding_model

    # Initialize vector database client
    logger.info("Initializing vector database client...")
    # Initialize your vector DB client here (e.g., Qdrant, Pinecone, etc.)
    # vector_db_client = initialize_vector_db()

    # Initialize embedding model
    logger.info("Initializing embedding model...")
    # embedding_model = initialize_embedding_model()

    logger.info("Application started successfully")
    yield

    # Cleanup on shutdown
    logger.info("Shutting down application...")
    if vector_db_client:
        # Close vector database connection
        pass
    logger.info("Application shutdown complete")

# Initialize FastAPI app
app = FastAPI(
    title="RAG System API",
    description="Retrieval-Augmented Generation system with vector database integration",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "RAG System API is running", "status": "healthy"}

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "components": {
            "vector_db": "connected" if vector_db_client else "disconnected",
            "embedding_model": "loaded" if embedding_model else "not loaded"
        }
    }

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query the RAG system to retrieve relevant documents and generate responses.

    Args:
        request: Query request with query text and parameters

    Returns:
        QueryResponse with results and metadata
    """
    try:
        logger.info(f"Processing query: {request.query[:50]}...")

        # Generate embedding for the query
        query_embedding = embedding_model.encode(request.query)

        # Perform similarity search in vector database
        search_results = vector_db_client.search(
            collection_name="documents",
            query_vector=query_embedding,
            limit=request.top_k,
            query_filter=request.filters
        )

        # Process results and generate response
        results = []
        sources = []

        for result in search_results:
            doc_data = {
                "content": result.payload.get("content", ""),
                "metadata": result.payload.get("metadata", {}),
                "score": result.score
            }
            results.append(doc_data)
            sources.append(result.payload.get("source", "unknown"))

        response = QueryResponse(
            results=results,
            query_time=0.1,  # Replace with actual timing
            sources=sources
        )

        logger.info(f"Query processed successfully, found {len(results)} results")
        return response

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/documents", response_model=DocumentResponse)
async def add_document(document: Document):
    """
    Add a document to the RAG system for retrieval.

    Args:
        document: Document content and metadata to be indexed

    Returns:
        DocumentResponse with success status and document ID
    """
    try:
        logger.info(f"Adding document with metadata: {document.metadata}")

        # Process document content (chunking, etc.)
        # chunks = chunk_document(document.content)

        # Generate embeddings for document chunks
        # embeddings = embedding_model.encode(chunks)

        # Store in vector database
        # doc_id = vector_db_client.store_document(chunks, embeddings, document.metadata)

        # For now, return a mock success
        doc_id = "doc_" + str(hash(document.content))[:8]

        response = DocumentResponse(
            success=True,
            document_id=doc_id,
            message=f"Document added successfully with ID: {doc_id}"
        )

        logger.info(f"Document added successfully: {doc_id}")
        return response

    except Exception as e:
        logger.error(f"Error adding document: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error adding document: {str(e)}")

@app.delete("/documents/{document_id}", response_model=DocumentResponse)
async def delete_document(document_id: str):
    """
    Delete a document from the RAG system.

    Args:
        document_id: ID of the document to delete

    Returns:
        DocumentResponse with success status
    """
    try:
        logger.info(f"Deleting document: {document_id}")

        # Delete from vector database
        # success = vector_db_client.delete_document(document_id)

        # For now, return a mock success
        success = True

        response = DocumentResponse(
            success=success,
            document_id=document_id,
            message=f"Document {document_id} deleted successfully" if success else f"Document {document_id} not found"
        )

        logger.info(f"Document deletion processed: {document_id}, success: {success}")
        return response

    except Exception as e:
        logger.error(f"Error deleting document {document_id}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error deleting document: {str(e)}")

@app.get("/stats")
async def get_statistics():
    """
    Get statistics about the RAG system.

    Returns:
        Dictionary with system statistics
    """
    try:
        # Get statistics from vector database
        # stats = vector_db_client.get_collection_stats("documents")

        # For now, return mock statistics
        stats = {
            "total_documents": 150,
            "total_chunks": 1247,
            "indexed_size_mb": 45.7,
            "last_updated": "2024-12-13T10:30:00Z",
            "embedding_model": "text-embedding-ada-002",
            "vector_db": "qdrant"
        }

        return stats

    except Exception as e:
        logger.error(f"Error getting statistics: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting statistics: {str(e)}")

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")
    uvicorn.run(app, host=host, port=port)