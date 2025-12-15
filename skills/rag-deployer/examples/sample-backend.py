from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any, Union
from datetime import datetime
import uvicorn
import logging
import asyncio
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
import hashlib
from tenacity import retry, stop_after_attempt, wait_exponential

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize global clients
qdrant_client = None
openai_client = None

# Pydantic models for request/response validation
class Document(BaseModel):
    content: str = Field(..., min_length=1, max_length=10000)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    doc_id: Optional[str] = None

class DocumentResponse(BaseModel):
    success: bool
    doc_id: str
    message: str

class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    top_k: int = Field(default=5, ge=1, le=100)
    filters: Optional[Dict[str, Any]] = None
    include_metadata: bool = True

class QueryResponse(BaseModel):
    query: str
    results: List[Dict[str, Any]]
    query_time: float
    sources: List[str]

class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    context_size: int = Field(default=5, ge=1, le=20)
    temperature: float = Field(default=0.7, ge=0.0, le=2.0)

class ChatResponse(BaseModel):
    query: str
    response: str
    sources: List[str]
    response_time: float

class HealthResponse(BaseModel):
    status: str
    timestamp: datetime
    components: Dict[str, str]

class StatisticsResponse(BaseModel):
    total_documents: int
    total_collections: int
    indexed_size_mb: float
    last_updated: datetime
    embedding_model: str
    vector_db: str

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup and cleanup on shutdown."""
    global qdrant_client, openai_client

    logger.info("Initializing services...")

    # Initialize Qdrant client
    try:
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=10
            )
        else:
            qdrant_client = QdrantClient(host="localhost", port=6333)

        # Test connection
        qdrant_client.get_collections()
        logger.info("Qdrant client initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {e}")
        raise

    # Initialize OpenAI client
    try:
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        openai_client = openai.AsyncOpenAI(api_key=openai_api_key)
        logger.info("OpenAI client initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize OpenAI client: {e}")
        raise

    # Create collection if it doesn't exist
    try:
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if "documents" not in collection_names:
            qdrant_client.create_collection(
                collection_name="documents",
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )
            logger.info("Created 'documents' collection in Qdrant")
        else:
            logger.info("Using existing 'documents' collection")
    except Exception as e:
        logger.error(f"Failed to create or access collection: {e}")
        raise

    logger.info("All services initialized successfully")
    yield

    # Cleanup on shutdown
    logger.info("Shutting down services...")
    if qdrant_client:
        qdrant_client.close()
    logger.info("Services shutdown complete")

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

async def get_embedding(text: str) -> List[float]:
    """Get embedding for text using OpenAI API."""
    try:
        response = await openai_client.embeddings.create(
            input=text,
            model=os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        raise HTTPException(status_code=500, detail=f"Error getting embedding: {e}")

async def generate_response(query: str, context: str) -> str:
    """Generate response using OpenAI API with context."""
    try:
        system_prompt = "You are a helpful assistant that answers questions based on the provided context. Be concise and accurate."

        response = await openai_client.chat.completions.create(
            model=os.getenv("CHAT_MODEL", "gpt-3.5-turbo"),
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}\n\nAnswer:"}
            ],
            temperature=0.7,
            max_tokens=500
        )
        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"Error generating response: {e}")
        raise HTTPException(status_code=500, detail=f"Error generating response: {e}")

@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "RAG System API is running", "status": "healthy", "version": "1.0.0"}

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    try:
        # Check Qdrant health
        qdrant_health = "unavailable"
        try:
            qdrant_client.get_collections()
            qdrant_health = "available"
        except:
            qdrant_health = "unavailable"

        # Check OpenAI health (simple test)
        openai_health = "unavailable"
        try:
            await openai_client.models.list()
            openai_health = "available"
        except:
            openai_health = "unavailable"

        health_status = HealthResponse(
            status="healthy" if qdrant_health == "available" and openai_health == "available" else "degraded",
            timestamp=datetime.utcnow(),
            components={
                "qdrant": qdrant_health,
                "openai": openai_health,
                "api": "available"
            }
        )

        return health_status
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=500, detail=f"Health check failed: {e}")

@app.post("/documents", response_model=DocumentResponse)
async def add_document(document: Document):
    """Add a document to the RAG system for retrieval."""
    try:
        start_time = asyncio.get_event_loop().time()
        logger.info(f"Adding document with metadata: {document.metadata}")

        # Generate embedding for the document content
        embedding = await get_embedding(document.content)

        # Create document ID if not provided
        doc_id = document.doc_id or str(uuid.uuid4())

        # Prepare payload for Qdrant
        payload = {
            "content": document.content,
            "metadata": document.metadata,
            "created_at": datetime.utcnow().isoformat()
        }

        # Store in Qdrant
        qdrant_client.upsert(
            collection_name="documents",
            points=[
                models.PointStruct(
                    id=hashlib.md5(doc_id.encode()).hexdigest(),
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        response_time = asyncio.get_event_loop().time() - start_time
        logger.info(f"Document added successfully: {doc_id} in {response_time:.2f}s")

        return DocumentResponse(
            success=True,
            doc_id=doc_id,
            message=f"Document added successfully with ID: {doc_id}"
        )
    except Exception as e:
        logger.error(f"Error adding document: {e}")
        raise HTTPException(status_code=500, detail=f"Error adding document: {str(e)}")

@app.post("/query", response_model=QueryResponse)
async def query_documents(request: QueryRequest):
    """Query the RAG system to retrieve relevant documents."""
    try:
        start_time = asyncio.get_event_loop().time()
        logger.info(f"Processing query: {request.query[:50]}...")

        # Generate embedding for the query
        query_embedding = await get_embedding(request.query)

        # Perform search in Qdrant
        search_results = qdrant_client.search(
            collection_name="documents",
            query_vector=query_embedding,
            limit=request.top_k,
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="metadata",
                        match=models.MatchAny(
                            any=list(request.filters.values()) if request.filters else []
                        )
                    )
                ]
            ) if request.filters else None
        )

        # Process results
        results = []
        sources = []

        for result in search_results:
            doc_data = {
                "id": result.id,
                "content": result.payload.get("content", ""),
                "metadata": result.payload.get("metadata", {}) if request.include_metadata else {},
                "score": result.score
            }
            results.append(doc_data)

            # Extract source information
            source = result.payload.get("metadata", {}).get("source", f"doc_{result.id}")
            sources.append(source)

        response_time = asyncio.get_event_loop().time() - start_time
        logger.info(f"Query processed successfully: found {len(results)} results in {response_time:.2f}s")

        return QueryResponse(
            query=request.query,
            results=results,
            query_time=response_time,
            sources=sources
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Chat endpoint that retrieves context and generates a response."""
    try:
        start_time = asyncio.get_event_loop().time()
        logger.info(f"Processing chat request: {request.query[:50]}...")

        # First, get relevant documents for context
        query_embedding = await get_embedding(request.query)

        search_results = qdrant_client.search(
            collection_name="documents",
            query_vector=query_embedding,
            limit=request.context_size
        )

        # Build context from search results
        context_parts = []
        sources = []

        for result in search_results:
            context_parts.append(result.payload.get("content", ""))
            source = result.payload.get("metadata", {}).get("source", f"doc_{result.id}")
            sources.append(source)

        context = "\n\n".join(context_parts)

        if not context:
            context = "No relevant context found in the knowledge base."

        # Generate response using the context
        response_text = await generate_response(request.query, context)

        response_time = asyncio.get_event_loop().time() - start_time
        logger.info(f"Chat response generated in {response_time:.2f}s")

        return ChatResponse(
            query=request.query,
            response=response_text,
            sources=sources,
            response_time=response_time
        )
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.get("/stats", response_model=StatisticsResponse)
async def get_statistics():
    """Get statistics about the RAG system."""
    try:
        # Get collection statistics
        collection_info = qdrant_client.get_collection(collection_name="documents")

        # Calculate indexed size (approximate)
        points_count = collection_info.points_count
        # Approximate size: 1536 dimensions * 4 bytes per float * number of points
        approximate_size_mb = (points_count * 1536 * 4) / (1024 * 1024) if points_count > 0 else 0

        return StatisticsResponse(
            total_documents=points_count,
            total_collections=1,  # Simplified
            indexed_size_mb=round(approximate_size_mb, 2),
            last_updated=datetime.utcnow(),
            embedding_model=os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002"),
            vector_db="qdrant"
        )
    except Exception as e:
        logger.error(f"Error getting statistics: {e}")
        raise HTTPException(status_code=500, detail=f"Error getting statistics: {str(e)}")

@app.delete("/documents/{doc_id}", response_model=DocumentResponse)
async def delete_document(doc_id: str):
    """Delete a document from the RAG system."""
    try:
        logger.info(f"Deleting document: {doc_id}")

        # Convert doc_id to the hash used in Qdrant
        point_id = hashlib.md5(doc_id.encode()).hexdigest()

        # Delete from Qdrant
        qdrant_client.delete(
            collection_name="documents",
            points_selector=models.PointIdsList(
                points=[point_id]
            )
        )

        return DocumentResponse(
            success=True,
            doc_id=doc_id,
            message=f"Document {doc_id} deleted successfully"
        )
    except Exception as e:
        logger.error(f"Error deleting document {doc_id}: {e}")
        raise HTTPException(status_code=500, detail=f"Error deleting document: {str(e)}")

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")
    uvicorn.run(app, host=host, port=port, log_level="info")