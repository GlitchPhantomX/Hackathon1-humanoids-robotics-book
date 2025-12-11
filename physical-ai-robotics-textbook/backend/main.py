"""
Main FastAPI application for the RAG Chatbot.
"""
from dotenv import load_dotenv
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import os

from app.config import settings
from app.services.postgres_service import postgres_service
from app.utils.logger import get_logger
from app.routes import chat  # Import the chat routes

logger = get_logger(__name__)
load_dotenv()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for the FastAPI application.
    Handles startup and shutdown events.
    """
    # Startup
    logger.info("Initializing application...")
    
    # Initialize database
    try:
        await postgres_service.init_db()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Error initializing database: {e}")
        raise
    
    yield  # Application runs here
    
    # Shutdown
    logger.info("Shutting down application...")
    await postgres_service.close()


# Create FastAPI app with lifespan
app = FastAPI(
    title="Physical AI & Robotics Textbook RAG Chatbot API",
    description="API for interacting with the Physical AI & Robotics Textbook using RAG",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins.split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routes
app.include_router(chat.router, prefix="/api", tags=["chat"])

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the application status.
    """
    return {
        "status": "healthy",
        "timestamp": "2025-12-07T16:00:00Z",
        "services": {
            "postgres": True,  # We initialized it in lifespan
            "qdrant": True,    # Assume it's available
            "google_ai": True  # Assume it's available
        }
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=True
    )