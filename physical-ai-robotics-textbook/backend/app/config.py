import os
from typing import Optional
from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict
from dotenv import load_dotenv

# Explicitly load .env file
BASE_DIR = Path(__file__).resolve().parent.parent
load_dotenv(BASE_DIR / ".env")


class Settings(BaseSettings):
    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "physical_ai_textbook"

    # Postgres Configuration
    database_url: str

    # Google Gemini Configuration
    google_api_key: str

    # Application Configuration
    environment: str = "development"
    log_level: str = "INFO"
    debug: bool = False

    # RAG / LLM Configuration
    embedding_model: str = "text-embedding-004"
    llm_model: str = "gemini-2.5-flash"
    max_retrieved_chunks: int = 5
    max_context_length: int = 1000
    temperature: float = 0.7
    max_tokens: int = 1000

    # Text Processing
    chunk_size: int = 1000
    chunk_overlap: int = 200

    # API Configuration
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    allowed_origins: str = (
        "http://localhost:3000,http://127.0.0.1:3000,http://localhost:5173,http://127.0.0.1:5173,http://localhost:3002"
    )

    model_config = SettingsConfigDict(
        case_sensitive=False,
        extra="ignore"
    )


# Initialize settings
settings = Settings()