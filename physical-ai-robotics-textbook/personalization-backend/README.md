# Chapter Personalization Backend

This service provides AI-driven personalization for textbook chapters based on user profile information.

## Architecture

The personalization backend is designed to work with:
- `auth-backend` for user authentication and profile data
- `rag-chatbot` for AI processing capabilities

## Setup

1. Install dependencies: `uv pip install -e .`
2. Configure environment variables in `.env`
3. Run the service: `python main.py`

## Endpoints

- `GET /health` - Service health check
- `POST /api/personalize/chapter` - Personalize chapter content (to be implemented)

## Environment Variables

- `PERSONALIZATION_SERVICE_PORT` - Port to run the service on (default: 8001)
- `PERSONALIZATION_SERVICE_HOST` - Host to bind to (default: localhost)
- `AUTH_SERVICE_URL` - URL of the auth backend service
- `RAG_SERVICE_URL` - URL of the RAG chatbot service
- `OPENAI_API_KEY` - OpenAI API key for AI processing