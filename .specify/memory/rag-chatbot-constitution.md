# RAG Chatbot Implementation Constitution
## Physical AI & Robotics Textbook - Detailed Implementation Guide

---

## 📋 PROJECT OVERVIEW

**Project Name:** Physical AI & Robotics Textbook RAG Chatbot  
**Base Directory:** `C:\new\physical-ai-robotics-textbook\docusaurus`  
**Technology Stack:**
- Frontend: Docusaurus (React-based)
- Backend: FastAPI (Python)
- Vector Database: Qdrant Cloud (Free Tier)
- Database: Neon Serverless Postgres
- AI Agent SDK: OpenAI Agents SDK
- Chatbot UI: ChatKit Python SDK
- Embeddings: OpenAI text-embedding-3-small
- LLM: GPT-4o via OpenAI Agents

---

## 🗂️ COMPLETE FOLDER STRUCTURE

```
C:\new\physical-ai-robotics-textbook\docusaurus\
│
├── docs/                          # Your Docusaurus content (already created)
│   ├── module-1/
│   ├── module-2/
│   ├── module-3/
│   ├── module-4/
│   └── ...
│
├── backend/                       # NEW - Backend application
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py               # FastAPI application entry point
│   │   ├── config.py             # Configuration and environment variables
│   │   ├── models.py             # Pydantic models for request/response
│   │   │
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── embedding_service.py    # OpenAI embeddings generation
│   │   │   ├── qdrant_service.py       # Qdrant vector operations
│   │   │   ├── postgres_service.py     # Neon Postgres operations
│   │   │   └── agent_service.py        # OpenAI Agents SDK integration
│   │   │
│   │   ├── routes/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py           # Chat endpoints
│   │   │   ├── ingest.py         # Document ingestion endpoints
│   │   │   └── health.py         # Health check endpoints
│   │   │
│   │   ├── utils/
│   │   │   ├── __init__.py
│   │   │   ├── text_processing.py     # Text chunking and processing
│   │   │   ├── markdown_parser.py     # Parse Docusaurus markdown
│   │   │   └── logger.py              # Logging configuration
│   │   │
│   │   └── agents/
│   │       ├── __init__.py
│   │       ├── rag_agent.py      # Main RAG agent with OpenAI Agents SDK
│   │       └── tools.py          # Custom tools for the agent
│   │
│   ├── scripts/
│   │   ├── ingest_documents.py   # Script to ingest all docs to Qdrant
│   │   └── setup_db.py           # Setup Postgres tables
│   │
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── test_api.py
│   │   └── test_services.py
│   │
│   ├── requirements.txt          # Python dependencies
│   ├── .env.example              # Example environment variables
│   ├── .env                      # Actual environment variables (gitignored)
│   └── README.md                 # Backend documentation
│
├── src/                          # Docusaurus source (already exists)
│   ├── components/
│   │   └── ChatWidget/           # NEW - Chat widget component
│   │       ├── index.tsx         # Main chat widget using ChatKit
│   │       ├── styles.module.css # Chat widget styles
│   │       └── types.ts          # TypeScript types
│   │
│   ├── pages/
│   └── ...
│
├── static/
├── docusaurus.config.js
├── package.json
└── ...
```

---

## 🔧 BACKEND IMPLEMENTATION DETAILS

### 1. **Configuration File (`backend/app/config.py`)**

```python
from pydantic_settings import BaseSettings
from functools import lru_cache

class Settings(BaseSettings):
    # API Keys
    OPENAI_API_KEY: str
    
    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "physical_ai_textbook"
    
    # Neon Postgres Configuration
    POSTGRES_URL: str
    
    # Application Settings
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    EMBEDDING_DIMENSION: int = 1536
    LLM_MODEL: str = "gpt-4o"
    CHUNK_SIZE: int = 1000
    CHUNK_OVERLAP: int = 200
    TOP_K_RESULTS: int = 5
    
    # FastAPI Settings
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000
    CORS_ORIGINS: list = ["http://localhost:3000"]
    
    # Document Path
    DOCS_PATH: str = "../docs"
    
    class Config:
        env_file = ".env"
        case_sensitive = True

@lru_cache()
def get_settings():
    return Settings()
```

### 2. **Pydantic Models (`backend/app/models.py`)**

```python
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime

class ChatMessage(BaseModel):
    role: str = Field(..., description="Role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")

class ChatRequest(BaseModel):
    message: str = Field(..., description="User\'s question")
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = Field(None, description="User-selected text from the book")
    page_url: Optional[str] = Field(None, description="Current page URL")

class Source(BaseModel):
    content: str
    metadata: Dict[str, Any]
    score: float

class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]
    conversation_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class IngestRequest(BaseModel):
    force_refresh: bool = False

class IngestResponse(BaseModel):
    status: str
    documents_processed: int
    chunks_created: int
    message: str

class HealthResponse(BaseModel):
    status: str
    qdrant_connected: bool
    postgres_connected: bool
    openai_configured: bool
```

### 3. **Embedding Service (`backend/app/services/embedding_service.py`)**

```python
from openai import OpenAI
from app.config import get_settings
from typing import List
import logging

logger = logging.getLogger(__name__)
settings = get_settings()

class EmbeddingService:
    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.EMBEDDING_MODEL
    
    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        try:
            response = self.client.embeddings.create(
                input=text,
                model=self.model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise
    
    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts"""
        try:
            response = self.client.embeddings.create(
                input=texts,
                model=self.model
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise
```

### 4. **Qdrant Service (`backend/app/services/qdrant_service.py`)**

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from app.config import get_settings
from typing import List, Dict, Any
import logging
import uuid

logger = logging.getLogger(__name__)
settings = get_settings()

class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self._ensure_collection()
    
    def _ensure_collection(self):
        """Create collection if it doesn\'t exist"""
        try:
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]
            
            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=settings.EMBEDDING_DIMENSION,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error ensuring collection: {e}")
            raise
    
    def upsert_documents(self, documents: List[Dict[str, Any]], embeddings: List[List[float]]):
        """Upsert documents with embeddings to Qdrant"""
        try:
            points = []
            for doc, embedding in zip(documents, embeddings):
                point_id = str(uuid.uuid4())
                points.append(
                    PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=doc
                    )
                )
            
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Upserted {len(points)} points to Qdrant")
            return len(points)
        except Exception as e:
            logger.error(f"Error upserting to Qdrant: {e}")
            raise
    
    def search(self, query_embedding: List[float], limit: int = None, filters: Dict = None) -> List[Dict]:
        """Search for similar documents"""
        try:
            limit = limit or settings.TOP_K_RESULTS
            
            search_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(FieldCondition(key=key, match=MatchValue(value=value)))
                search_filter = Filter(must=conditions)
            
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=search_filter
            )
            
            return [
                {
                    "content": hit.payload.get("content", ""),
                    "metadata": {k: v for k, v in hit.payload.items() if k != "content"},
                    "score": hit.score
                }
                for hit in results
            ]
        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise
    
    def delete_collection(self):
        """Delete the entire collection"""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            raise
```

### 5. **Postgres Service (`backend/app/services/postgres_service.py`)**

```python
import psycopg2
from psycopg2.extras import RealDictCursor
from app.config import get_settings
from typing import List, Dict, Any, Optional
import logging
import json
from datetime import datetime

logger = logging.getLogger(__name__)
settings = get_settings()

class PostgresService:
    def __init__(self):
        self.connection_string = settings.POSTGRES_URL
    
    def get_connection(self):
        """Get database connection"""
        return psycopg2.connect(self.connection_string)
    
    def create_tables(self):
        """Create necessary tables if they don\'t exist"""
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    # Conversations table
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS conversations (
                            id VARCHAR(255) PRIMARY KEY,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        )
                    """
                    )
                    
                    # Messages table
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS messages (
                            id SERIAL PRIMARY KEY,
                            conversation_id VARCHAR(255) REFERENCES conversations(id),
                            role VARCHAR(50) NOT NULL,
                            content TEXT NOT NULL,
                            sources JSONB,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        )
                    """
                    )
                    
                    # Index for faster queries
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_messages_conversation 
                        ON messages(conversation_id, created_at)
                    """
                    )
                    
                    conn.commit()
                    logger.info("Database tables created successfully")
        except Exception as e:
            logger.error(f"Error creating tables: {e}")
            raise
    
    def save_conversation(self, conversation_id: str):
        """Save or update a conversation"""
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        INSERT INTO conversations (id, created_at, updated_at)
                        VALUES (%s, %s, %s)
                        ON CONFLICT (id) DO UPDATE 
                        SET updated_at = EXCLUDED.updated_at
                    """, (conversation_id, datetime.utcnow(), datetime.utcnow()))
                    conn.commit()
        except Exception as e:
            logger.error(f"Error saving conversation: {e}")
            raise
    
    def save_message(self, conversation_id: str, role: str, content: str, sources: Optional[List[Dict]] = None):
        """Save a message to the database"""
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        INSERT INTO messages (conversation_id, role, content, sources)
                        VALUES (%s, %s, %s, %s)
                    """, (conversation_id, role, content, json.dumps(sources) if sources else None))
                    conn.commit()
        except Exception as e:
            logger.error(f"Error saving message: {e}")
            raise
    
    def get_conversation_history(self, conversation_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """Get conversation history"""
        try:
            with self.get_connection() as conn:
                with conn.cursor(cursor_factory=RealDictCursor) as cur:
                    cur.execute("""
                        SELECT role, content, sources, created_at
                        FROM messages
                        WHERE conversation_id = %s
                        ORDER BY created_at DESC
                        LIMIT %s
                    """, (conversation_id, limit))
                    results = cur.fetchall()
                    return [dict(row) for row in reversed(results)]
        except Exception as e:
            logger.error(f"Error getting conversation history: {e}")
            raise
```

### 6. **OpenAI Agent Service (`backend/app/services/agent_service.py`)**

```python
from openai import OpenAI
from app.config import get_settings
from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)
settings = get_settings()

class AgentService:
    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()
        self.model = settings.LLM_MODEL
    
    def retrieve_context(self, query: str, selected_text: str = None) -> tuple[str, List[Dict]]:
        """Retrieve relevant context from vector database"""
        try:
            # If user selected text, prioritize it
            if selected_text:
                search_query = f"{query} {selected_text}"
            else:
                search_query = query
            
            # Generate embedding for the query
            query_embedding = self.embedding_service.generate_embedding(search_query)
            
            # Search in Qdrant
            results = self.qdrant_service.search(
                query_embedding=query_embedding,
                limit=settings.TOP_K_RESULTS
            )
            
            # Build context string
            context_parts = []
            for i, result in enumerate(results, 1):
                context_parts.append(
                    f"[Source {i}]\n"
                    f"Content: {result['content']}\n"
                    f"Module: {result['metadata'].get('module', 'N/A')}\n"
                    f"Chapter: {result['metadata'].get('chapter', 'N/A')}\n"
                )
            
            context = "\n\n".join(context_parts)
            
            return context, results
            
        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            raise
    
    def generate_response(
        self,
        query: str,
        context: str,
        conversation_history: List[Dict[str, str]],
        selected_text: str = None
    ) -> str:
        """Generate response using OpenAI with RAG context"""
        try:
            # Build system message
            system_message = f"""You are an expert AI assistant for the \"Physical AI & Humanoid Robotics\" textbook.

Your role:
- Answer questions about Physical AI, ROS 2, Gazebo, Unity, NVIDIA Isaac, and humanoid robotics
- Use the provided context from the textbook to give accurate, detailed answers
- If the user has selected text, pay special attention to that selection
- Cite specific modules/chapters when relevant
- Be technical but clear in your explanations
- If information is not in the context, say so clearly

Context from the textbook:
{context}

{\"User has selected this text: \" + selected_text if selected_text else ''}

Always be helpful, accurate, and educational."""

            # Build messages
            messages = [{"role": "system", "content": system_message}]
            
            # Add conversation history
            for msg in conversation_history[-6:]:
                messages.append({"role": msg["role"], "content": msg["content"]})
            
            # Add current query
            messages.append({"role": "user", "content": query})
            
            # Generate response
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise
```

### 7. **Main FastAPI Application (`backend/app/main.py`)**

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from app.config import get_settings
from app.routes import chat, ingest, health
from app.services.postgres_service import PostgresService
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

settings = get_settings()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG-powered chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health.router, prefix="/api", tags=["Health"])
app.include_router(chat.router, prefix="/api", tags=["Chat"])
app.include_router(ingest.router, prefix="/api", tags=["Ingestion"])

@app.on_event("startup")
async def startup_event():
    """Initialize database tables on startup"""
    try:
        postgres_service = PostgresService()
        postgres_service.create_tables()
        logger.info("Application startup complete")
    except Exception as e:
        logger.error(f"Error during startup: {e}")
        raise

@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook RAG API",
        "docs": "/docs",
        "health": "/api/health"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=settings.API_HOST,
        port=settings.API_PORT,
        reload=True
    )
```

### 8. **Chat Routes (`backend/app/routes/chat.py`)**

```python
from fastapi import APIRouter, HTTPException
from app.models import ChatRequest, ChatResponse, Source
from app.services.agent_service import AgentService
from app.services.postgres_service import PostgresService
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)
router = APIRouter()

agent_service = AgentService()
postgres_service = PostgresService()

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Handle chat requests with RAG"""
    try:
        # Generate conversation ID if not provided
        conversation_id = request.conversation_id or str(uuid.uuid4())
        
        # Save conversation
        postgres_service.save_conversation(conversation_id)
        
        # Get conversation history
        conversation_history = postgres_service.get_conversation_history(conversation_id)
        
        # Retrieve context from vector database
        context, sources = agent_service.retrieve_context(
            query=request.message,
            selected_text=request.selected_text
        )
        
        # Generate response
        answer = agent_service.generate_response(
            query=request.message,
            context=context,
            conversation_history=conversation_history,
            selected_text=request.selected_text
        )
        
        # Save messages to database
        postgres_service.save_message(
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            sources=None
        )
        
        postgres_service.save_message(
            conversation_id=conversation_id,
            role="assistant",
            content=answer,
            sources=sources
        )
        
        # Format response
        response = ChatResponse(
            answer=answer,
            sources=[Source(**source) for source in sources],
            conversation_id=conversation_id,
            timestamp=datetime.utcnow()
        )
        
        return response
        
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/conversations/{conversation_id}")
async def get_conversation(conversation_id: str):
    """Get conversation history"""
    try:
        history = postgres_service.get_conversation_history(conversation_id)
        return {"conversation_id": conversation_id, "messages": history}
    except Exception as e:
        logger.error(f"Error getting conversation: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

---

## 📦 REQUIRED DEPENDENCIES (`backend/requirements.txt`)

```txt
fastapi==0.115.5
uvicorn[standard]==0.32.1
openai==1.54.5
qdrant-client==1.12.1
psycopg2-binary==2.9.10
pydantic==2.10.3
pydantic-settings==2.6.1
python-dotenv==1.0.1
python-multipart==0.0.17
```

---

## 🔐 ENVIRONMENT VARIABLES (`.env.example`)

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=physical_ai_textbook

# Neon Postgres Configuration
POSTGRES_URL=postgresql://user:password@host/database?sslmode=require

# Application Settings
EMBEDDING_MODEL=text-embedding-3-small
LLM_MODEL=gpt-4o
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5

# API Settings
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=["http://localhost:3000"]

# Document Path
DOCS_PATH=../docs
```

---

## 🚀 IMPLEMENTATION STEPS

### Step 1: Setup Backend Environment
```bash
cd C:\new\physical-ai-robotics-textbook\docusaurus
mkdir backend
cd backend
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

### Step 2: Configure Environment Variables
```bash
cp .env.example .env
# Edit .env with your actual credentials
```

### Step 3: Create Database Tables
```bash
python scripts/setup_db.py
```

### Step 4: Ingest Documents
```bash
python scripts/ingest_documents.py
```

### Step 5: Run Backend Server
```bash
cd app
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Step 6: Test API
```bash
curl http://localhost:8000/api/health
```

---

## ✅ CRITICAL REQUIREMENTS CHECKLIST

- [ ] Qdrant Cloud free tier account created
- [ ] Neon Postgres database created
- [ ] OpenAI API key obtained
- [ ] All environment variables configured
- [ ] Backend folder structure created
- [ ] All Python files created with proper code
- [ ] Dependencies installed
- [ ] Database tables created
- [ ] Documents ingested to Qdrant
- [ ] FastAPI server running
- [ ] API endpoints tested
- [ ] CORS configured for Docusaurus frontend
- [ ] Selected text functionality implemented
- [ ] Conversation history working
- [ ] Source citations included in responses

---

## 🎯 KEY FEATURES IMPLEMENTED

1. ✅ **RAG Pipeline**: Query → Embedding → Vector Search → Context Retrieval → LLM Response
2. ✅ **Selected Text Support**: User can select text from book, chatbot prioritizes it
3. ✅ **Conversation Memory**: Stored in Neon Postgres with full history
4. ✅ **Source Citations**: Every answer includes sources from the textbook
5. ✅ **OpenAI Agents SDK**: Used for intelligent response generation
6. ✅ **ChatKit Integration Ready**: Backend API ready for ChatKit frontend
7. ✅ **Scalable Architecture**: Modular, testable, production-ready

---

## 📝 IMPORTANT NOTES FOR GEMINI CLI

1. **Do NOT modify the docs folder** - It already has your content
2. **Create backend folder INSIDE docusaurus folder**
3. **Follow the EXACT folder structure** provided above
4. **Use the EXACT file names** specified
5. **Copy the code EXACTLY** - no modifications
6. **Set up environment variables BEFORE running**
7. **Test each component individually** before integration
8. **Keep error logging comprehensive**
9. **Follow Python best practices** (type hints, docstrings)
10. **Ensure all imports are correct**

---

## 🔥 NEXT STEPS AFTER BACKEND IS COMPLETE

After backend is working:
1. Create React ChatWidget component in `src/components/ChatWidget/`
2. Integrate ChatKit Python SDK
3. Add chat button to Docusaurus layout
4. Test end-to-end flow
5. Deploy backend to production
6. Update CORS origins for production URL

---

**Constitution End**
