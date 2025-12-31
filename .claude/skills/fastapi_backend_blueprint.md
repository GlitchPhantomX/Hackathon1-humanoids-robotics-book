# FastAPI Backend Blueprint Skill

---
name: fastapi_backend_blueprint
description: Generate a reusable FastAPI backend structure for Physical AI & Humanoid Robotics project
version: 1.0.0
project: physical-ai-robotics-textbook
parameters:
  - name: service_name
    description: Name of the backend service (e.g., rag-chatbot, robotics-api)
    required: false
  - name: features
    description: Comma-separated features to include (chatbot,chapter-help,robotics-explain)
    required: false
---

## Purpose

Generate a clean, production-ready FastAPI backend structure that:
- Separates concerns (routes, services, AI logic)
- Exposes AI-powered endpoints for educational robotics content
- Remains reusable across the entire textbook project
- Follows industry best practices

## Core Principles

**MUST FOLLOW:**
1. **Separation of Concerns**: Routes → Services → AI Logic (3-layer architecture)
2. **No Frontend Logic**: Backend serves only as API layer
3. **Chapter-Agnostic**: No hardcoded chapter numbers/paths
4. **Reusability**: Components can be imported/extended
5. **Type Safety**: Use Pydantic models for all requests/responses
6. **Environment-Driven**: All config via environment variables

## Instructions

### Step 1: Gather Requirements

Ask the user:
1. Service name (default: `ai-backend`)
2. Which features to include:
   - **Chatbot queries**: General Q&A about robotics concepts
   - **Chapter-level help**: Context-aware assistance for specific chapters
   - **Robotics explanations**: Deep-dive technical explanations
3. Database/vector store (if needed): Pinecone, Qdrant, ChromaDB, etc.
4. LLM provider: OpenAI, Anthropic, Gemini, Ollama

### Step 2: Define Project Structure

Generate the following directory structure:

```
<service_name>/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Environment configuration
│   ├── dependencies.py         # Shared dependencies
│   │
│   ├── models/                 # Pydantic models
│   │   ├── __init__.py
│   │   ├── requests.py         # Request models
│   │   ├── responses.py        # Response models
│   │   └── common.py           # Shared types
│   │
│   ├── routes/                 # API endpoints
│   │   ├── __init__.py
│   │   ├── chatbot.py          # Chatbot endpoints
│   │   ├── chapter.py          # Chapter-specific help
│   │   └── explain.py          # Robotics explanations
│   │
│   ├── services/               # Business logic
│   │   ├── __init__.py
│   │   ├── chatbot_service.py
│   │   ├── chapter_service.py
│   │   └── explain_service.py
│   │
│   ├── ai/                     # AI/ML logic
│   │   ├── __init__.py
│   │   ├── llm_client.py       # LLM wrapper
│   │   ├── rag_engine.py       # RAG implementation
│   │   ├── embeddings.py       # Embedding generation
│   │   └── prompts.py          # Prompt templates
│   │
│   └── utils/                  # Utilities
│       ├── __init__.py
│       ├── logging.py
│       └── middleware.py
│
├── tests/                      # Test suite
│   ├── __init__.py
│   ├── test_routes/
│   ├── test_services/
│   └── test_ai/
│
├── .env.example                # Environment template
├── requirements.txt            # Dependencies
├── Dockerfile                  # Container definition
└── README.md                   # Setup instructions
```

### Step 3: Generate Core Files

#### 3.1 Entry Point (`app/main.py`)

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.routes import chatbot, chapter, explain
from app.utils.middleware import setup_logging

app = FastAPI(
    title="Physical AI & Robotics Backend",
    description="AI-powered backend for robotics education",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Logging
setup_logging()

# Include routers
app.include_router(chatbot.router, prefix="/api/chatbot", tags=["chatbot"])
app.include_router(chapter.router, prefix="/api/chapter", tags=["chapter"])
app.include_router(explain.router, prefix="/api/explain", tags=["explain"])

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "ai-backend"}
```

#### 3.2 Configuration (`app/config.py`)

```python
from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    # API Configuration
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000
    ALLOWED_ORIGINS: List[str] = ["http://localhost:3000"]

    # LLM Configuration
    LLM_PROVIDER: str = "openai"  # openai, anthropic, gemini
    OPENAI_API_KEY: str = ""
    ANTHROPIC_API_KEY: str = ""
    GEMINI_API_KEY: str = ""

    # Vector Store
    VECTOR_STORE: str = "pinecone"  # pinecone, qdrant, chromadb
    PINECONE_API_KEY: str = ""
    PINECONE_ENVIRONMENT: str = ""
    PINECONE_INDEX: str = "robotics-textbook"

    # RAG Configuration
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    CHUNK_SIZE: int = 1000
    CHUNK_OVERLAP: int = 200
    TOP_K_RESULTS: int = 5

    # Prompt Configuration
    MAX_TOKENS: int = 2000
    TEMPERATURE: float = 0.7

    class Config:
        env_file = ".env"

settings = Settings()
```

#### 3.3 Request Models (`app/models/requests.py`)

```python
from pydantic import BaseModel, Field
from typing import Optional, List

class ChatbotQuery(BaseModel):
    """General chatbot query request"""
    question: str = Field(..., min_length=1, max_length=2000)
    conversation_id: Optional[str] = None
    context: Optional[dict] = None

class ChapterHelpRequest(BaseModel):
    """Chapter-specific help request"""
    chapter_id: str = Field(..., description="Chapter identifier (e.g., '01-ros2')")
    question: str = Field(..., min_length=1, max_length=2000)
    section: Optional[str] = Field(None, description="Specific section within chapter")
    code_context: Optional[str] = Field(None, description="User's code for context")

class ExplainRequest(BaseModel):
    """Robotics concept explanation request"""
    topic: str = Field(..., description="Topic to explain (e.g., 'inverse kinematics')")
    depth: str = Field("intermediate", regex="^(beginner|intermediate|advanced)$")
    include_code: bool = Field(True, description="Include code examples")
    include_math: bool = Field(True, description="Include mathematical formulas")
```

#### 3.4 Response Models (`app/models/responses.py`)

```python
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime

class Source(BaseModel):
    """Source reference for RAG responses"""
    chapter: str
    section: str
    content_snippet: str
    relevance_score: float

class ChatbotResponse(BaseModel):
    """Chatbot response with sources"""
    answer: str
    sources: List[Source]
    conversation_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class ChapterHelpResponse(BaseModel):
    """Chapter-specific help response"""
    answer: str
    related_concepts: List[str]
    sources: List[Source]
    suggested_exercises: Optional[List[str]] = None

class ExplainResponse(BaseModel):
    """Detailed explanation response"""
    explanation: str
    key_points: List[str]
    code_examples: Optional[List[dict]] = None
    mathematical_notation: Optional[str] = None
    sources: List[Source]
    related_topics: List[str]
```

#### 3.5 Route Example (`app/routes/chatbot.py`)

```python
from fastapi import APIRouter, HTTPException, Depends
from app.models.requests import ChatbotQuery
from app.models.responses import ChatbotResponse
from app.services.chatbot_service import ChatbotService
from app.dependencies import get_chatbot_service

router = APIRouter()

@router.post("/query", response_model=ChatbotResponse)
async def chatbot_query(
    request: ChatbotQuery,
    service: ChatbotService = Depends(get_chatbot_service)
):
    """
    Handle general chatbot queries about robotics concepts.

    - **question**: User's question
    - **conversation_id**: Optional conversation tracking
    - **context**: Additional context for the query
    """
    try:
        response = await service.process_query(request)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### 3.6 Service Example (`app/services/chatbot_service.py`)

```python
from app.models.requests import ChatbotQuery
from app.models.responses import ChatbotResponse, Source
from app.ai.rag_engine import RAGEngine
from app.ai.llm_client import LLMClient
from typing import List
import uuid

class ChatbotService:
    """Chatbot business logic - chapter-agnostic"""

    def __init__(self, rag_engine: RAGEngine, llm_client: LLMClient):
        self.rag_engine = rag_engine
        self.llm_client = llm_client

    async def process_query(self, query: ChatbotQuery) -> ChatbotResponse:
        """Process a chatbot query with RAG"""

        # 1. Retrieve relevant context
        relevant_docs = await self.rag_engine.retrieve(
            query=query.question,
            top_k=5
        )

        # 2. Build prompt with context
        prompt = self._build_prompt(query.question, relevant_docs)

        # 3. Generate response
        answer = await self.llm_client.generate(prompt)

        # 4. Format sources
        sources = self._format_sources(relevant_docs)

        # 5. Return response
        return ChatbotResponse(
            answer=answer,
            sources=sources,
            conversation_id=query.conversation_id or str(uuid.uuid4())
        )

    def _build_prompt(self, question: str, docs: List[dict]) -> str:
        """Build RAG prompt - generic, not chapter-specific"""
        context = "\n\n".join([doc["content"] for doc in docs])

        return f"""You are an expert robotics instructor helping students learn Physical AI and Humanoid Robotics.

Context from the textbook:
{context}

Student question: {question}

Provide a clear, educational answer based on the context above. Include:
- Direct answer to the question
- Relevant technical details
- Practical examples where helpful

Answer:"""

    def _format_sources(self, docs: List[dict]) -> List[Source]:
        """Convert retrieved docs to Source objects"""
        return [
            Source(
                chapter=doc.get("chapter", "unknown"),
                section=doc.get("section", "unknown"),
                content_snippet=doc["content"][:200],
                relevance_score=doc.get("score", 0.0)
            )
            for doc in docs
        ]
```

#### 3.7 AI Layer Example (`app/ai/rag_engine.py`)

```python
from typing import List, Dict
from app.config import settings
import pinecone
from openai import OpenAI

class RAGEngine:
    """Retrieval-Augmented Generation engine - reusable across all features"""

    def __init__(self):
        self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)

        # Initialize vector store
        if settings.VECTOR_STORE == "pinecone":
            pinecone.init(
                api_key=settings.PINECONE_API_KEY,
                environment=settings.PINECONE_ENVIRONMENT
            )
            self.index = pinecone.Index(settings.PINECONE_INDEX)

    async def retrieve(self, query: str, top_k: int = 5, filters: Dict = None) -> List[Dict]:
        """
        Retrieve relevant documents for a query.

        Args:
            query: User's query
            top_k: Number of results to return
            filters: Optional metadata filters (e.g., {"chapter": "01-ros2"})

        Returns:
            List of relevant documents with metadata
        """
        # 1. Generate query embedding
        embedding = await self._get_embedding(query)

        # 2. Search vector store
        results = self.index.query(
            vector=embedding,
            top_k=top_k,
            filter=filters,
            include_metadata=True
        )

        # 3. Format results
        documents = []
        for match in results.matches:
            documents.append({
                "content": match.metadata.get("text", ""),
                "chapter": match.metadata.get("chapter", ""),
                "section": match.metadata.get("section", ""),
                "score": match.score
            })

        return documents

    async def _get_embedding(self, text: str) -> List[float]:
        """Generate embedding for text"""
        response = self.openai_client.embeddings.create(
            model=settings.EMBEDDING_MODEL,
            input=text
        )
        return response.data[0].embedding
```

#### 3.8 Dependencies (`app/dependencies.py`)

```python
from app.services.chatbot_service import ChatbotService
from app.services.chapter_service import ChapterService
from app.services.explain_service import ExplainService
from app.ai.rag_engine import RAGEngine
from app.ai.llm_client import LLMClient
from functools import lru_cache

@lru_cache()
def get_rag_engine() -> RAGEngine:
    """Singleton RAG engine instance"""
    return RAGEngine()

@lru_cache()
def get_llm_client() -> LLMClient:
    """Singleton LLM client instance"""
    return LLMClient()

def get_chatbot_service(
    rag_engine: RAGEngine = Depends(get_rag_engine),
    llm_client: LLMClient = Depends(get_llm_client)
) -> ChatbotService:
    """Dependency injection for chatbot service"""
    return ChatbotService(rag_engine, llm_client)

def get_chapter_service(
    rag_engine: RAGEngine = Depends(get_rag_engine),
    llm_client: LLMClient = Depends(get_llm_client)
) -> ChapterService:
    """Dependency injection for chapter service"""
    return ChapterService(rag_engine, llm_client)

def get_explain_service(
    rag_engine: RAGEngine = Depends(get_rag_engine),
    llm_client: LLMClient = Depends(get_llm_client)
) -> ExplainService:
    """Dependency injection for explain service"""
    return ExplainService(rag_engine, llm_client)
```

### Step 4: Generate Supporting Files

#### 4.1 Requirements (`requirements.txt`)

```txt
# FastAPI
fastapi==0.109.0
uvicorn[standard]==0.27.0
pydantic==2.5.3
pydantic-settings==2.1.0

# AI/ML
openai==1.10.0
anthropic==0.18.0
google-generativeai==0.3.2
pinecone-client==3.0.0

# Vector Stores (optional alternatives)
# qdrant-client==1.7.0
# chromadb==0.4.22

# Utilities
python-dotenv==1.0.0
httpx==0.26.0
tenacity==8.2.3

# Testing
pytest==7.4.4
pytest-asyncio==0.23.3
httpx==0.26.0
```

#### 4.2 Environment Template (`.env.example`)

```env
# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001

# LLM Provider (openai, anthropic, gemini)
LLM_PROVIDER=openai
OPENAI_API_KEY=your_openai_key_here
ANTHROPIC_API_KEY=your_anthropic_key_here
GEMINI_API_KEY=your_gemini_key_here

# Vector Store (pinecone, qdrant, chromadb)
VECTOR_STORE=pinecone
PINECONE_API_KEY=your_pinecone_key_here
PINECONE_ENVIRONMENT=us-east-1-aws
PINECONE_INDEX=robotics-textbook

# RAG Configuration
EMBEDDING_MODEL=text-embedding-3-small
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5

# LLM Configuration
MAX_TOKENS=2000
TEMPERATURE=0.7
```

#### 4.3 README Template

```markdown
# Physical AI & Robotics Backend

AI-powered FastAPI backend for the Physical AI & Humanoid Robotics textbook.

## Features

- **Chatbot API**: General Q&A about robotics concepts
- **Chapter Help**: Context-aware assistance for specific chapters
- **Robotics Explanations**: Deep technical explanations with code examples

## Architecture

- **3-Layer Design**: Routes → Services → AI Logic
- **RAG-Powered**: Retrieval-Augmented Generation for accurate answers
- **Type-Safe**: Pydantic models for all requests/responses
- **Reusable**: Chapter-agnostic components

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure environment:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

3. Run the server:
   ```bash
   uvicorn app.main:app --reload
   ```

4. Access API docs:
   - Swagger UI: http://localhost:8000/docs
   - ReDoc: http://localhost:8000/redoc

## API Endpoints

### Chatbot
- `POST /api/chatbot/query` - General robotics Q&A

### Chapter Help
- `POST /api/chapter/help` - Chapter-specific assistance

### Explanations
- `POST /api/explain/concept` - Detailed concept explanations

## Testing

```bash
pytest tests/
```

## Deployment

Docker:
```bash
docker build -t robotics-backend .
docker run -p 8000:8000 --env-file .env robotics-backend
```
```

### Step 5: Validation Checklist

After generation, verify:

- [ ] **No frontend code**: Zero HTML/CSS/JS in backend
- [ ] **Chapter-agnostic**: No hardcoded chapter IDs or paths
- [ ] **Separation of concerns**: Routes → Services → AI
- [ ] **Type safety**: All endpoints use Pydantic models
- [ ] **Environment config**: All secrets in .env
- [ ] **Dependency injection**: Proper use of FastAPI Depends
- [ ] **Reusability**: Components can be imported/extended
- [ ] **Documentation**: API docs auto-generated
- [ ] **Error handling**: Try-catch blocks in routes
- [ ] **Logging**: Proper logging setup

### Step 6: Output Summary

Provide:
1. **Directory structure** - Full file tree created
2. **Endpoints summary** - List all API routes
3. **Setup instructions** - How to run the backend
4. **Next steps**:
   - Test endpoints with sample requests
   - Integrate with vector store
   - Add authentication if needed
   - Deploy to production

## Design Patterns

### 1. Service Layer Pattern
Services contain business logic, routes are thin wrappers.

### 2. Dependency Injection
Use FastAPI's `Depends()` for testability and singleton management.

### 3. Repository Pattern (for data access)
Separate data access from business logic if using databases.

### 4. Factory Pattern (for AI clients)
Create AI client instances based on configuration.

## Best Practices

1. **Async by default**: Use `async def` for I/O operations
2. **Error handling**: Catch exceptions, return proper HTTP status codes
3. **Validation**: Let Pydantic handle request validation
4. **Logging**: Log all errors and important events
5. **Testing**: Write tests for services and routes
6. **Documentation**: Use docstrings and FastAPI's automatic docs

## Anti-Patterns to Avoid

❌ **Don't**: Put business logic in routes
✅ **Do**: Keep routes thin, logic in services

❌ **Don't**: Hardcode chapter IDs or content
✅ **Do**: Use metadata filters in RAG queries

❌ **Don't**: Mix frontend concerns
✅ **Do**: Return JSON only, let frontend handle rendering

❌ **Don't**: Store secrets in code
✅ **Do**: Use environment variables

## Example Usage

```bash
# User invokes the skill
/fastapi_backend_blueprint

# Skill asks for clarification
Assistant: I'll generate a FastAPI backend blueprint. What would you like to name the service?
         - rag-chatbot (default)
         - robotics-api
         - ai-backend

User: ai-backend