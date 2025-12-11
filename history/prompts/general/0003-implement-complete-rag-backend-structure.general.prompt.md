ID: 0002
TITLE: Implement complete RAG backend structure
STAGE: general
DATE_ISO: 2025-12-05
SURFACE: agent
MODEL: gemini-1.5-flash
FEATURE: rag-backend-structure
BRANCH: main
USER: user
COMMAND: "CRITICAL ISSUE: Backend structure is incomplete. No RAG implementation exists.

CURRENT STRUCTURE (WRONG):
backend/
├── src/
├── main.py (only has /health endpoint)
├── .env
└── requirements.txt

REQUIRED STRUCTURE:
backend/
├── rag_agent/
│   ├── __init__.py
│   └── rag_service.py          # RAG logic with OpenAI + Qdrant
├── vector_search/
│   ├── __init__.py
│   └── index_documents.py      # Index markdown files
├── database_setup/
│   ├── __init__.py
│   └── database.py             # Already exists (I have the code)
├── models/
│   ├── __init__.py
│   └── schemas.py              # Pydantic models
├── main.py                     # FastAPI app with chat endpoints
├── .env
└── requirements.txt

IMPLEMENTATION REQUIREMENTS:

1. CREATE rag_agent/rag_service.py:
```python
from openai import OpenAI
from qdrant_client import QdrantClient
import os

class RAGService:
    def __init__(self):
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = "book_content"
    
    async def chat_with_rag(self, user_id: str, user_message: str, 
                           conversation_id: int = None, selected_text: str = None):
        """
        Main RAG pipeline:
        1. Generate embedding for user question
        2. Search Qdrant for relevant chunks
        3. Build context from retrieved chunks
        4. Call OpenAI with context
        5. Stream response back
        """
        
        # Step 1: Generate embedding
        embedding_response = self.openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=user_message
        )
        query_embedding = embedding_response.data[0].embedding
        
        # Step 2: Search Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=5
        )
        
        # Step 3: Build context
        context = "\n\n".join([
            f"[{hit.payload['chapter']}]\n{hit.payload['text']}"
            for hit in search_results
        ])
        
        # Step 4: Prepare messages for OpenAI
        messages = [
            {
                "role": "system",
                "content": "You are a helpful assistant for a Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context from the book."
            }
        ]
        
        if selected_text:
            messages.append({
                "role": "user",
                "content": f"Selected text: {selected_text}\n\nContext from book:\n{context}\n\nQuestion: {user_message}"
            })
        else:
            messages.append({
                "role": "user",
                "content": f"Context from book:\n{context}\n\nQuestion: {user_message}"
            })
        
        # Step 5: Stream OpenAI response
        stream = self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            stream=True
        )
        
        full_response = ""
        for chunk in stream:
            if chunk.choices[0].delta.content:
                content = chunk.choices[0].delta.content
                full_response += content
                yield f"data: {content}\n\n"
        
        # Extract sources
        sources = list(set([hit.payload['chapter'] for hit in search_results]))
        yield f"data: [SOURCES]{','.join(sources)}\n\n"
```

2. CREATE vector_search/index_documents.py:
```python
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import os
from pathlib import Path
import hashlib

def index_documents():
    """
    Read all markdown files from docs/, chunk them, generate embeddings,
    and store in Qdrant
    """
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    
    collection_name = "book_content"
    
    # Create collection if not exists
    try:
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )
    except Exception as e:
        print(f"Collection might already exist: {e}")
    
    # Find docs directory
    docs_dir = Path(__file__).parent.parent.parent / "docs"
    
    if not docs_dir.exists():
        print(f"Docs directory not found: {docs_dir}")
        return
    
    points = []
    point_id = 0
    
    # Process all markdown files
    for md_file in docs_dir.rglob("*.md*"):
        print(f"Processing: {md_file}")
        
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Simple chunking (split by paragraphs)
        chunks = [p.strip() for p in content.split('\n\n') if len(p.strip()) > 100]
        
        for chunk in chunks:
            # Generate embedding
            embedding_response = openai_client.embeddings.create(
                model="text-embedding-3-small",
                input=chunk
            )
            embedding = embedding_response.data[0].embedding
            
            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": chunk,
                    "file_path": str(md_file),
                    "chapter": md_file.stem,
                }
            )
            points.append(point)
            point_id += 1
    
    # Upload to Qdrant
    qdrant_client.upsert(collection_name=collection_name, points=points)
    print(f"Indexed {len(points)} chunks from {len(list(docs_dir.rglob('*.md*')))} files")
```

3. CREATE models/schemas.py:
```python
from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    user_id: str
    message: str
    conversation_id: Optional[int] = None
    selected_text: Optional[str] = None
```

4. UPDATE main.py (REPLACE COMPLETELY):
```python
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import os
from dotenv import load_dotenv
from sqlalchemy.orm import Session
import threading

from rag_agent.rag_service import RAGService
from models.schemas import ChatRequest
from database_setup.database import get_db
from vector_search.index_documents import index_documents

load_dotenv()

app = FastAPI()

# CORS
origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG Service
rag_service = RAGService()

@app.get("/health")
async def health_check():
    return {"status": "ok"}

@app.post("/api/chat/ask")
async def chat_endpoint(request: ChatRequest):
    """Main chat endpoint"""
    async def generate():
        async for chunk in rag_service.chat_with_rag(
            user_id=request.user_id,
            user_message=request.message,
            conversation_id=request.conversation_id
        ):
            yield chunk
    
    return StreamingResponse(generate(), media_type="text/event-stream")

@app.post("/api/chat/selected-text")
async def chat_selected_endpoint(request: ChatRequest):
    """Selected text chat"""
    if not request.selected_text:
        return {"error": "selected_text required"}, 400
    
    async def generate():
        async for chunk in rag_service.chat_with_rag(
            user_id=request.user_id,
            user_message=request.message,
            conversation_id=request.conversation_id,
            selected_text=request.selected_text
        ):
            yield chunk
    
    return StreamingResponse(generate(), media_type="text/event-stream")

# Indexing
indexing_status = {"is_indexing": False}

def run_indexing():
    global indexing_status
    try:
        indexing_status["is_indexing"] = True
        index_documents()
        indexing_status["is_indexing"] = False
    except Exception as e:
        indexing_status["is_indexing"] = False
        print(f"Error: {e}")

@app.post("/api/indexing/rebuild")
async def rebuild_index():
    if indexing_status["is_indexing"]:
        return {"status": "already_running"}
    
    thread = threading.Thread(target=run_indexing, daemon=True)
    thread.start()
    return {"status": "started"}

@app.get("/api/indexing/status")
async def get_indexing_status():
    return indexing_status

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

5. UPDATE requirements.txt:
fastapi==0.104.1
uvicorn==0.24.0
openai==1.3.0
qdrant-client==1.7.0
python-dotenv==1.0.0
sqlalchemy==2.0.23
psycopg2-binary==2.9.9
pydantic==2.5.0

6. ENSURE .env HAS:
OPENAI_API_KEY=sk-proj-...
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
DATABASE_URL=postgresql://...
CORS_ORIGINS=http://localhost:3000

CREATE THIS COMPLETE STRUCTURE NOW.
After creating, I need to:
1. Run: pip install -r requirements.txt
2. Index content: curl -X POST http://localhost:8000/api/indexing/rebuild
3. Test chat: curl -X POST http://localhost:8000/api/chat/ask -H "Content-Type: application/json" -d '{"user_id":"test","message":"What is ROS 2?"}'