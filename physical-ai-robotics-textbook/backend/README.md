# RAG Chatbot Backend API

This directory contains the backend FastAPI application for the RAG (Retrieval-Augmented Generation) chatbot that answers questions about the Physical AI & Robotics textbook.

## Setup Instructions

### Prerequisites

- Python 3.11+
- pip
- Virtual environment (recommended: `venv`)

### Environment Setup

1. Create a `.env` file in the backend root directory with the following variables:
   ```
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=postgresql+asyncpg://user:password@localhost/dbname
   GOOGLE_API_KEY=your_google_api_key
   ```

2. Install the dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Running the Application

To start the backend server:

```bash
cd backend
uvicorn app.main:app --reload
```

The API will be available at `http://localhost:8000`.

## API Endpoints

### POST /api/chat
Main chat endpoint that handles user messages and returns AI responses.

- Request Body:
  ```json
  {
    "message": "Your message here",
    "conversation_id": "optional conversation ID",
    "selected_text": "optional selected text for context",
    "user_id": "optional user ID"
  }
  ```
- Response:
  ```json
  {
    "response": "AI-generated response",
    "conversation_id": "conversation ID",
    "sources": [
      {
        "source_file": "source file path",
        "module": "module name",
        "chapter": "chapter name",
        "chunk_index": 0,
        "relevance_score": 0.0,
        "content": "relevant content snippet"
      }
    ],
    "timestamp": "ISO timestamp"
  }
  ```

### GET /api/conversations/{conversation_id}
Retrieve a specific conversation with all its messages.

### GET /api/conversations
Retrieve all conversations for a user.

### POST /api/ingest
Trigger document ingestion into the vector database.

### GET /api/health
System health check endpoint.

## Architecture

The backend follows a service-oriented architecture:
- Models: Pydantic models for request/response validation
- Routes: API endpoints
- Services: Business logic (agent, vector search, postgres, embedding)
- Utils: Utilities (logger, error handler, text processing)

## Error Handling

The application uses comprehensive error handling with:
- Proper HTTP status codes
- User-friendly error messages
- Detailed logging with context
- Performance monitoring
- Secure logging that sanitizes sensitive information

## Testing

To run the tests:

```bash
cd backend
python -m pytest tests/ -v
```

## Technologies Used

- FastAPI: Modern, fast web framework for building APIs with Python 3.7+ based on standard Python type hints.
- SQLAlchemy: SQL toolkit and Object Relational Mapping (ORM) library.
- Qdrant: Vector similarity search engine.
- Google Generative AI: For implementing the conversational agent.
- PostgreSQL: For storing conversation history.