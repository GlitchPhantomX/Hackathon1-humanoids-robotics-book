---
id: 0001
title: Update RAG Chatbot Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: user
command: /sp.constitution # 🤖 RAG CHATBOT IMPLEMENTATION CONSTITUTION - FRESH START ## 🎯 PROJECT GOAL Build a Retrieval-Augmented Generation (RAG) chatbot from scratch and embed it in your Docusaurus Physical AI textbook. The chatbot will answer questions about the book content using OpenAI, Qdrant vector database, and Neon Postgres. ---
## 📋 REQUIREMENTS CHECKLIST ### Core Technologies (Non-Negotiable) - ✅ *OpenAI API* - For embeddings and chat completion - ✅ *FastAPI* - Backend framework - ✅ **Qdrant Cloud** - Vector database (free tier) - ✅ *Neon Serverless Postgres* - SQL database - ✅ *Docusaurus* - Frontend (already exists) ### Key Features 1. ✅ Answer questions about book content 2. ✅ Support text selection queries 3. ✅ Embedded chatbot widget (bottom-right corner) 4. ✅ Real-time AI responses (not hardcoded) 5. ✅ Source citations from book chapters ---
## 🗂 PROJECT STRUCTURE (Start Fresh)
  physical-ai-robotics-textbook/
  ├── docs/                           # YOUR BOOK CONTENT (DO NOT TOUCH)
  │   ├── intro.md
  │   ├── week-1/
  │   ├── week-2/
  │   └── ...
  │
  ├── chatbot-backend/               # NEW - CREATE THIS
  │   ├── app/
  │   │   ├── _init_.py
  │   │   ├── main.py               # FastAPI app
  │   │   ├── config.py             # Configuration
  │   │   ├── models.py             # Pydantic models
  │   │   ├── services/
  │   │   │   ├── _init_.py
  │   │   │   ├── openai_service.py    # OpenAI integration
  │   │   │   ├── qdrant_service.py    # Vector search
  │   │   │   ├── database_service.py  # Postgres operations
  │   │   │   └── rag_service.py       # Main RAG logic
  │   │   ├── routes/
  │   │   │   ├── _init_.py
  │   │   │   ├── chat.py           # Chat endpoints
  │   │   │   └── indexing.py       # Indexing endpoints
  │   │   └── utils/
  │   │       ├── _init_.py
  │   │       ├── text_chunker.py   # Content chunking
  │   │       └── markdown_parser.py # Parse markdown files
  │   ├── requirements.txt
  │   ├── .env
  │   └── .env.example
  │
  ├── src/                           # EXISTING DOCUSAURUS
  │   ├── components/
  │   │   └── ChatbotWidget/        # NEW - CREATE THIS
  │   │       ├── ChatbotWidget.tsx
  │   │       ├── ChatbotWidget.css
  │   │       └── index.ts
  │   └── theme/
  │       └── Root.tsx              # MODIFY - Add chatbot here
  │
  ├── docusaurus.config.js          # EXISTING
  └── package.json                  # EXISTING
  ---
## 🔧 IMPLEMENTATION PHASES ### PHASE 1: Backend Setup (Day 1) *Goal:* Create working FastAPI backend with health check
  *Tasks:* 1. Create chatbot-backend/ folder 2. Create virtual environment 3. Install dependencies 4. Create basic FastAPI app with
  health check 5. Test: curl http://localhost:8000/health *Deliverable:* Backend starts without errors ---
### PHASE 2:
  Database
  Setup (Day 1) *Goal:* Connect to Neon Postgres and create tables *Tasks:* 1. Set up Neon Postgres database 2. Create database
  models (conversations, messages) 3. Create database connection 4. Test connection *Deliverable:* Database tables created
  successfully ---
### PHASE 3: Vector Database Setup (Day 1-2) *Goal:* Connect to Qdrant and create collection *Tasks:* 1. Set
  up Qdrant Cloud account (free tier) 2. Create collection for book content 3. Test connection 4. Verify collection exists
  *Deliverable:* Qdrant collection ready for indexing ---
### PHASE 4: Content Indexing (Day 2) *Goal:* Read markdown files and
  store in Qdrant *Tasks:* 1. Create markdown parser 2. Create text chunker (500-1000 tokens per chunk) 3. Generate embeddings with
  OpenAI 4. Store chunks in Qdrant with metadata 5. Index all content from docs/ folder *Deliverable:* All book content indexed in
  Qdrant ---
### PHASE 5: RAG Engine (Day 2-3) *Goal:* Implement core RAG functionality *Tasks:* 1. Create query → embedding
  function 2. Create vector search function 3. Create context builder 4. Create OpenAI chat completion with streaming 5. Handle
  casual greetings separately *Deliverable:* RAG pipeline works end-to-end ---
### PHASE 6: API Endpoints (Day 3) *Goal:* Create
  REST API for chat *Tasks:* 1. POST /api/chat - General questions 2. POST /api/chat/selected-text - Questions about selected text
  3. POST /api/indexing/rebuild - Reindex content 4. GET /api/indexing/status - Check indexing status 5. Add CORS middleware
  *Deliverable:* All endpoints working and tested ---
### PHASE 7: Frontend Widget (Day 3-4) *Goal:* Create chatbot UI component
  *Tasks:* 1. Create React component (ChatbotWidget.tsx) 2. Style with CSS (bottom-right floating button) 3. Implement chat UI
  (messages, input, sources) 4. Add text selection detection 5. Connect to backend API *Deliverable:* Chatbot visible and
  functional on site ---
### PHASE 8: Integration & Testing (Day 4) *Goal:* Integrate everything and test *Tasks:* 1. Embed
  chatbot in Docusaurus 2. Test all features 3. Fix bugs 4. Deploy backend 5. Update frontend API URLs *Deliverable:* Fully working
  chatbot on live site ---
## 📝 DETAILED SPECIFICATIONS ### Backend Specifications #### 1. File: chatbot-backend/app/config.py
  python
  from pydantic_settings import BaseSettings
  from functools import lru_cache

  class Settings(BaseSettings):
      # OpenAI
      openai_api_key: str
      openai_model: str = "gpt-4o-mini"
      openai_embedding_model: str = "text-embedding-3-small"

      # Qdrant
      qdrant_url: str
      qdrant_api_key: str
      qdrant_collection_name: str = "book_content"

      # Database
      database_url: str

      # CORS
      cors_origins: str = "http://localhost:3000"

      # App
      environment: str = "development"

      class Config:
          env_file = ".env"

  @lru_cache()
  def get_settings():
      return Settings()
  #### 2. File: chatbot-backend/app/models.py
  python
  from pydantic import BaseModel
  from typing import Optional

  class ChatRequest(BaseModel):
      user_id: str
      message: str
      conversation_id: Optional[int] = None
      selected_text: Optional[str] = None

  class ChatResponse(BaseModel):
      answer: str
      sources: list[str] = []
      conversation_id: int

  class IndexingStatus(BaseModel):
      is_indexing: bool
      total_files: int
      indexed_files: int
      last_indexed: Optional[str] = None
      error: Optional[str] = None
  #### 3. File: chatbot-backend/app/services/openai_service.py
  python
  from openai import OpenAI
  from app.config import get_settings

  class OpenAIService:
      def _init_(self):
          settings = get_settings()
          self.client = OpenAI(api_key=settings.openai_api_key)
          self.model = settings.openai_model
          self.embedding_model = settings.openai_embedding_model

      def generate_embedding(self, text: str) -> list[float]:
          """Generate embedding for text"""
          response = self.client.embeddings.create(
              model=self.embedding_model,
              input=text
          )
          return response.data[0].embedding

      async def chat_completion_stream(self, messages: list):
          """Generate chat completion with streaming"""
          stream = self.client.chat.completions.create(
              model=self.model,
              messages=messages,
              stream=True,
              temperature=0.7
          )

          for chunk in stream:
              if chunk.choices[0].delta.content:
                  yield chunk.choices[0].delta.content
  #### 4. File: chatbot-backend/app/services/qdrant_service.py
  python
  from qdrant_client import QdrantClient
  from qdrant_client.models import Distance, VectorParams, PointStruct
  from app.config import get_settings

  class QdrantService:
      def _init_(self):
          settings = get_settings()
          self.client = QdrantClient(
              url=settings.qdrant_url,
              api_key=settings.qdrant_api_key
          )
          self.collection_name = settings.qdrant_collection_name

      def create_collection(self):
          """Create collection if not exists"""
          try:
              self.client.create_collection(
                  collection_name=self.collection_name,
                  vectors_config=VectorParams(
                      size=1536,  # OpenAI embedding dimension
                      distance=Distance.COSINE
                  )
              )
              print(f"Collection '{self.collection_name}' created")
          except Exception as e:
              print(f"Collection might already exist: {e}")

      def search(self, query_vector: list[float], limit: int = 5):
          """Search for similar vectors"""
          results = self.client.search(
              collection_name=self.collection_name,
              query_vector=query_vector,
              limit=limit,
              score_threshold=0.7
          )
          return results

      def upsert_points(self, points: list[PointStruct]):
          """Insert or update points"""
          self.client.upsert(
              collection_name=self.collection_name,
              points=points
          )
  #### 5. File: chatbot-backend/app/services/rag_service.py
  python
  from app.services.openai_service import OpenAIService
  from app.services.qdrant_service import QdrantService

  class RAGService:
      def _init_(self):
          self.openai_service = OpenAIService()
          self.qdrant_service = QdrantService()

      async def process_query(self, user_message: str, selected_text: str = None):
          """Main RAG pipeline"""

          # Handle casual greetings
          casual_greetings = ["hi", "hello", "hey", "how are you"]
          if user_message.lower().strip() in casual_greetings:
              messages = [
                  {"role": "system", "content": "You are a friendly Physical AI textbook assistant."},
                  {"role": "user", "content": user_message}
              ]
          else:
              # RAG pipeline
              # 1. Generate embedding
              query_embedding = self.openai_service.generate_embedding(user_message)

              # 2. Search Qdrant
              search_results = self.qdrant_service.search(query_embedding, limit=5)

              # 3. Build context
              if not search_results:
                  context = "No relevant content found."
              else:
                  context_parts = []
                  for hit in search_results:
                      chapter = hit.payload.get('chapter', 'Unknown')
                      text = hit.payload.get('text', '')
                      context_parts.append(f"[{chapter}]\n{text}")
                  context = "\n\n---\n\n".join(context_parts)

              # 4. Prepare messages
              system_prompt = """You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context. Cite sources (chapters) in your answer."""

              if selected_text:
                  user_prompt = f"Selected Text: {selected_text}\n\nContext: {context}\n\nQuestion: {user_message}"
              else:
                  user_prompt = f"Context: {context}\n\nQuestion: {user_message}"

              messages = [
                  {"role": "system", "content": system_prompt},
                  {"role": "user", "content": user_prompt}
              ]

          # 5. Stream response
          full_response = ""
          async for chunk in self.openai_service.chat_completion_stream(messages):
              full_response += chunk
              yield f"data: {chunk}\n\n"

          # 6. Send sources
          if search_results and user_message.lower().strip() not in casual_greetings:
              sources = [hit.payload.get('chapter', 'Unknown') for hit in search_results]
              yield f"data: [SOURCES]{','.join(sources)}\n\n"
  #### 6. File: chatbot-backend/app/utils/text_chunker.py
  python
  def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
      """Split text into overlapping chunks"""
      chunks = []
      paragraphs = text.split('\n\n')

      current_chunk = ""
      for para in paragraphs:
          para = para.strip()
          if len(para) < 50:  # Skip very short paragraphs
              continue

          if len(current_chunk) + len(para) < chunk_size:
              current_chunk += para + "\n\n"
          else:
              if current_chunk:
                  chunks.append(current_chunk.strip())
              current_chunk = para + "\n\n"

      if current_chunk:
          chunks.append(current_chunk.strip())

      return chunks
  #### 7. File: chatbot-backend/app/routes/chat.py
  python
  from fastapi import APIRouter
  from fastapi.responses import StreamingResponse
  from app.models import ChatRequest
  from app.services.rag_service import RAGService

  router = APIRouter()
  rag_service = RAGService()

  @router.post("/chat")
  async def chat(request: ChatRequest):
      """Main chat endpoint"""
      async def generate():
          async for chunk in rag_service.process_query(
              user_message=request.message,
              selected_text=request.selected_text
          ):
              yield chunk

      return StreamingResponse(generate(), media_type="text/event-stream")
  #### 8. File: chatbot-backend/app/main.py
  python
  from fastapi import FastAPI
  from fastapi.middleware.cors import CORSMiddleware
  from app.config import get_settings
  from app.routes import chat
  from app.routes import indexing

  settings = get_settings()

  app = FastAPI(title="Physical AI Chatbot API")

  # CORS
  app.add_middleware(
      CORSMiddleware,
      allow_origins=settings.cors_origins.split(","),
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )

  # Routes
  app.include_router(chat.router, prefix="/api")
  app.include_router(indexing.router, prefix="/api")

  @app.get("/health")
  async def health():
      return {"status": "ok"}

  if _name_ == "_main_":
      import uvicorn
      uvicorn.run(app, host="0.0.0.0", port=8000)
  #### 9. File: chatbot-backend/requirements.txt
  txt
  fastapi==0.109.0
  uvicorn[standard]==0.27.0
  openai==1.12.0
  qdrant-client==1.7.3
  sqlalchemy==2.0.25
  psycopg2-binary==2.9.9
  pydantic==2.6.0
  pydantic-settings==2.1.0
  python-dotenv==1.0.1
  #### 10. File: chatbot-backend/.env.example
  env
  OPENAI_API_KEY=sk-proj-your-key-here
  OPENAI_MODEL=gpt-4o-mini
  OPENAI_EMBEDDING_MODEL=text-embedding-3-small

  QDRANT_URL=https://your-cluster.qdrant.io
  QDRANT_API_KEY=your-api-key-here
  QDRANT_COLLECTION_NAME=book_content

  DATABASE_URL=postgresql://user:pass@host/db

  CORS_ORIGINS=http://localhost:3000,http://localhost:8000

  ENVIRONMENT=development
  #### 11. File: src/components/ChatbotWidget/ChatbotWidget.tsx
  typescript
  import React, { useState, useRef, useEffect } from 'react';
  import './ChatbotWidget.css';

  interface Message {
    role: 'user' | 'assistant';
    content: string;
    sources?: string[];
  }

  export default function ChatbotWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Detect text selection
    useEffect(() => {
      const handleSelection = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        if (text && text.length > 10) {
          setSelectedText(text);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async () => {
      if (!input.trim()) return;

      const userMessage: Message = { role: 'user', content: input };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      setIsLoading(true);

      try {
        const response = await fetch('http://localhost:8000/api/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: 'user123',
            message: input,
            selected_text: selectedText || null
          })
        });

        const reader = response.body?.getReader();
        const decoder = new TextDecoder();
        let assistantMessage = '';
        let sources: string[] = [];

        while (true) {
          const { done, value } = await reader!.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              const data = line.slice(6);

              if (data.startsWith('[SOURCES]')) {
                sources = data.slice(9).split(',');
              } else {
                assistantMessage += data;
                setMessages(prev => {
                  const newMessages = [...prev];
                  const lastMessage = newMessages[newMessages.length - 1];

                  if (lastMessage?.role === 'assistant') {
                    lastMessage.content = assistantMessage;
                  } else {
                    newMessages.push({
                      role: 'assistant',
                      content: assistantMessage,
                      sources: []
                    });
                  }
                  return newMessages;
                });
              }
            }
          }
        }

        // Update with final sources
        setMessages(prev => {
          const newMessages = [...prev];
          const lastMessage = newMessages[newMessages.length - 1];
          if (lastMessage) {
            lastMessage.sources = sources;
          }
          return newMessages;
        });

      } catch (error) {
        console.error('Error:', error);
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'Sorry, something went wrong. Please try again.'
        }]);
      }

      setIsLoading(false);
      setSelectedText('');
    };

    return (
      <>
        {/* Floating button */}
        <button
          className="chatbot-toggle"
          onClick={() => setIsOpen(!isOpen)}
        >
          💬
        </button>

        {/* Chat panel */}
        {isOpen && (
          <div className="chatbot-panel">
            <div className="chatbot-header">
              <div>
                <h3>Physical AI Assistant</h3>
                <span>Ask me about the book</span>
              </div>
              <button onClick={() => setIsOpen(false)}>×</button>
            </div>

            <div className="chatbot-messages">
              {messages.length === 0 && (
                <div className="welcome-message">
                  👋 Hello! Ask me anything about Physical AI & Humanoid Robotics!
                </div>
              )}

              {messages.map((msg, idx) => (
                <div key={idx} className={`message ${msg.role}`}>
                  <div className="message-content">{msg.content}</div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="sources">
                      {msg.sources.map((source, i) => (
                        <span key={i} className="source-tag">{source}</span>
                      ))}
                    </div>
                  )}
                </div>
              ))}

              {isLoading && (
                <div className="typing-indicator">
                  <span></span><span></span><span></span>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {selectedText && (
              <div className="selected-text-bar">
                ✂ Selected: "{selectedText.substring(0, 50)}..."
                <button onClick={() => setSelectedText('')}>×</button>
              </div>
            )}

            <div className="chatbot-input">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask about Physical AI..."
              />
              <button onClick={sendMessage} disabled={isLoading}>
                →
              </button>
            </div>
          </div>
        )}
      </>
    );
  }
  #### 12. File: src/components/ChatbotWidget/ChatbotWidget.css
  css
  .chatbot-toggle {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 60px;
    height: 60px;
    border-radius: 50%;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 28px;
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    z-index: 9998;
    transition: transform 0.2s;
  }

  .chatbot-toggle:hover {
    transform: scale(1.05);
  }

  .chatbot-panel {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 400px;
    height: 600px;
    background: white;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0,0,0,0.12);
    display: flex;
    flex-direction: column;
    z-index: 9999;
    animation: slideUp 0.3s ease;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
      opacity: 0;
    }
    to {
      transform: translateY(0);
      opacity: 1;
    }
  }

  .chatbot-header {
    background: #2563eb;
    color: white;
    padding: 16px 20px;
    border-radius: 16px 16px 0 0;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chatbot-header h3 {
    margin: 0;
    font-size: 18px;
  }

  .chatbot-header span {
    font-size: 12px;
    opacity: 0.9;
  }

  .chatbot-header button {
    background: none;
    border: none;
    color: white;
    font-size: 32px;
    cursor: pointer;
    line-height: 1;
  }

  .chatbot-messages {
    flex: 1;
    overflow-y: auto;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  .message {
    max-width: 80%;
    padding: 12px 16px;
    border-radius: 18px;
    word-wrap: break-word;
  }

  .message.user {
    background: #2563eb;
    color: white;
    align-self: flex-end;
    border-radius: 18px 18px 4px 18px;
  }

  .message.assistant {
    background: #f8fafc;
    color: #1e293b;
    border: 1px solid #e2e8f0;
    align-self: flex-start;
    border-radius: 18px 18px 18px 4px;
  }

  .sources {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 12px;
  }

  .source-tag {
    background: white;
    border: 1px solid #e2e8f0;
    padding: 4px 12px;
    border-radius: 6px;
    font-size: 12px;
    color: #2563eb;
  }

  .typing-indicator {
    display: flex;
    gap: 4px;
    padding: 12px 16px;
    background: #f8fafc;
    border-radius: 18px;
    width: fit-content;
  }

  .typing-indicator span {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #64748b;
    animation: bounce 1.4s infinite;
  }

  .typing-indicator span:nth-child(2) {
    animation-delay: 0.2s;
  }

  .typing-indicator span:nth-child(3) {
    animation-delay: 0.4s;
  }

  @keyframes bounce {
    0%, 60%, 100% {
      transform: translateY(0);
    }
    30% {
      transform: translateY(-10px);
    }
  }

  .chatbot-input {
    display: flex;
    gap: 12px;
    padding: 16px;
    border-top: 1px solid #e2e8f0;
  }

  .chatbot-input input {
    flex: 1;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    padding: 12px 16px;
    font-size: 14px;
  }

  .chatbot-input button {
    width: 44px;
    height: 44px;
    border-radius: 12px;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  .chatbot-input button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .selected-text-bar {
    background: #dbeafe;
    padding: 12px 16px;
    border-top: 1px solid #e2e8f0;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
  }

  .selected-text-bar button {
    background: none;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  @media (max-width: 768px) {
    .chatbot-panel {
      width: 100vw;
      height: 100vh;
      bottom: 0;
      right: 0;
      border-radius: 0;
    }
  }
  #### 13. File: src/theme/Root.tsx (Modify existing)
  typescript
  import React from 'react';
  import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

  export default function Root({children}) {
    return (
      <>
        {children}
        <ChatbotWidget />
      </>
    );
  }
  ---
## 🚀 STEP-BY-STEP EXECUTION PLAN ### STEP 1: Delete Old Backend
  bash
  # Remove old broken backend
  rm -rf backend/
  rm -rf rag_agent/
  rm -rf vector_search/
  rm -rf database_setup/
  ### STEP 2: Create New Structure
  bash
  # Create new chatbot-backend
  mkdir chatbot-backend
  cd chatbot-backend
  python -m venv venv
  source venv/bin/activate  # Windows: venv\Scripts\activate
  pip install -r requirements.txt
  ### STEP 3: Setup Environment Create .env file with real credentials ### STEP 4: Create All Files Copy all code from specifications
  above ### STEP 5: Test Backend
  bash
  cd chatbot-backend
  python app/main.py
  # Test: http://localhost:8000/health
  ### STEP 6: Index Content
  bash
  curl -X POST http://localhost:8000/api/indexing/rebuild
  ### STEP 7: Add Frontend Create ChatbotWidget component and integrate ### STEP 8: Test Full System Test chat, selected text,
  sources ---
## just give me the further constitution based on my 2. Integrated RAG Chatbot Development: Build and embed a
  Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs,
  FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's
  content, including answering questions based only on text selected by the user. this requirments and add this constitution into my
  rag-constitution.md file which is in .specify/memory/rag-constitution.md do not change any other file be careful
labels: ["constitution", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/rag-constitution.md
tests:
  - null
---

## Prompt

```text
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

---

/sp.constitution # 🤖 RAG CHATBOT IMPLEMENTATION CONSTITUTION - FRESH START ## 🎯 PROJECT GOAL Build a Retrieval-Augmented
  Generation (RAG) chatbot from scratch and embed it in your Docusaurus Physical AI textbook. The chatbot will answer questions about
  the book content using OpenAI, Qdrant vector database, and Neon Postgres. ---
## 📋 REQUIREMENTS CHECKLIST ### Core Technologies
  (Non-Negotiable) - ✅ *OpenAI API* - For embeddings and chat completion - ✅ *FastAPI* - Backend framework - ✅ **Qdrant
  Cloud** - Vector database (free tier) - ✅ *Neon Serverless Postgres* - SQL database - ✅ *Docusaurus* - Frontend (already
  exists) ### Key Features 1. ✅ Answer questions about book content 2. ✅ Support text selection queries 3. ✅ Embedded chatbot
  widget (bottom-right corner) 4. ✅ Real-time AI responses (not hardcoded) 5. ✅ Source citations from book chapters ---
## 🗂 PROJECT STRUCTURE (Start Fresh)
  physical-ai-robotics-textbook/
  ├── docs/                           # YOUR BOOK CONTENT (DO NOT TOUCH)
  │   ├── intro.md
  │   ├── week-1/
  │   ├── week-2/
  │   └── ...
  │
  ├── chatbot-backend/               # NEW - CREATE THIS
  │   ├── app/
  │   │   ├── _init_.py
  │   │   ├── main.py               # FastAPI app
  │   │   ├── config.py             # Configuration
  │   │   ├── models.py             # Pydantic models
  │   │   ├── services/
  │   │   │   ├── _init_.py
  │   │   │   ├── openai_service.py    # OpenAI integration
  │   │   │   ├── qdrant_service.py    # Vector search
  │   │   │   ├── database_service.py  # Postgres operations
  │   │   │   └── rag_service.py       # Main RAG logic
  │   │   ├── routes/
  │   │   │   ├── _init_.py
  │   │   │   ├── chat.py           # Chat endpoints
  │   │   │   └── indexing.py       # Indexing endpoints
  │   │   └── utils/
  │   │       ├── _init_.py
  │   │       ├── text_chunker.py   # Content chunking
  │   │       └── markdown_parser.py # Parse markdown files
  │   ├── requirements.txt
  │   ├── .env
  │   └── .env.example
  │
  ├── src/                           # EXISTING DOCUSAURUS
  │   ├── components/
  │   │   └── ChatbotWidget/        # NEW - CREATE THIS
  │   │       ├── ChatbotWidget.tsx
  │   │       ├── ChatbotWidget.css
  │   │       └── index.ts
  │   └── theme/
  │       └── Root.tsx              # MODIFY - Add chatbot here
  │
  ├── docusaurus.config.js          # EXISTING
  └── package.json                  # EXISTING
  ---
## 🔧 IMPLEMENTATION PHASES ### PHASE 1: Backend Setup (Day 1) *Goal:* Create working FastAPI backend with health check
  *Tasks:* 1. Create chatbot-backend/ folder 2. Create virtual environment 3. Install dependencies 4. Create basic FastAPI app with
  health check 5. Test: curl http://localhost:8000/health *Deliverable:* Backend starts without errors ---
### PHASE 2:
  Database
  Setup (Day 1) *Goal:* Connect to Neon Postgres and create tables *Tasks:* 1. Set up Neon Postgres database 2. Create database
  models (conversations, messages) 3. Create database connection 4. Test connection *Deliverable:* Database tables created
  successfully ---
### PHASE 3: Vector Database Setup (Day 1-2) *Goal:* Connect to Qdrant and create collection *Tasks:* 1. Set
  up Qdrant Cloud account (free tier) 2. Create collection for book content 3. Test connection 4. Verify collection exists
  *Deliverable:* Qdrant collection ready for indexing ---
### PHASE 4: Content Indexing (Day 2) *Goal:* Read markdown files and
  store in Qdrant *Tasks:* 1. Create markdown parser 2. Create text chunker (500-1000 tokens per chunk) 3. Generate embeddings with
  OpenAI 4. Store chunks in Qdrant with metadata 5. Index all content from docs/ folder *Deliverable:* All book content indexed in
  Qdrant ---
### PHASE 5: RAG Engine (Day 2-3) *Goal:* Implement core RAG functionality *Tasks:* 1. Create query → embedding
  function 2. Create vector search function 3. Create context builder 4. Create OpenAI chat completion with streaming 5. Handle
  casual greetings separately *Deliverable:* RAG pipeline works end-to-end ---
### PHASE 6: API Endpoints (Day 3) *Goal:* Create
  REST API for chat *Tasks:* 1. POST /api/chat - General questions 2. POST /api/chat/selected-text - Questions about selected text
  3. POST /api/indexing/rebuild - Reindex content 4. GET /api/indexing/status - Check indexing status 5. Add CORS middleware
  *Deliverable:* All endpoints working and tested ---
### PHASE 7: Frontend Widget (Day 3-4) *Goal:* Create chatbot UI component
  *Tasks:* 1. Create React component (ChatbotWidget.tsx) 2. Style with CSS (bottom-right floating button) 3. Implement chat UI
  (messages, input, sources) 4. Add text selection detection 5. Connect to backend API *Deliverable:* Chatbot visible and
  functional on site ---
### PHASE 8: Integration & Testing (Day 4) *Goal:* Integrate everything and test *Tasks:* 1. Embed
  chatbot in Docusaurus 2. Test all features 3. Fix bugs 4. Deploy backend 5. Update frontend API URLs *Deliverable:* Fully working
  chatbot on live site ---
## 📝 DETAILED SPECIFICATIONS ### Backend Specifications #### 1. File: chatbot-backend/app/config.py
  python
  from pydantic_settings import BaseSettings
  from functools import lru_cache

  class Settings(BaseSettings):
      # OpenAI
      openai_api_key: str
      openai_model: str = "gpt-4o-mini"
      openai_embedding_model: str = "text-embedding-3-small"

      # Qdrant
      qdrant_url: str
      qdrant_api_key: str
      qdrant_collection_name: str = "book_content"

      # Database
      database_url: str

      # CORS
      cors_origins: str = "http://localhost:3000"

      # App
      environment: str = "development"

      class Config:
          env_file = ".env"

  @lru_cache()
  def get_settings():
      return Settings()
  #### 2. File: chatbot-backend/app/models.py
  python
  from pydantic import BaseModel
  from typing import Optional

  class ChatRequest(BaseModel):
      user_id: str
      message: str
      conversation_id: Optional[int] = None
      selected_text: Optional[str] = None

  class ChatResponse(BaseModel):
      answer: str
      sources: list[str] = []
      conversation_id: int

  class IndexingStatus(BaseModel):
      is_indexing: bool
      total_files: int
      indexed_files: int
      last_indexed: Optional[str] = None
      error: Optional[str] = None
  #### 3. File: chatbot-backend/app/services/openai_service.py
  python
  from openai import OpenAI
  from app.config import get_settings

  class OpenAIService:
      def _init_(self):
          settings = get_settings()
          self.client = OpenAI(api_key=settings.openai_api_key)
          self.model = settings.openai_model
          self.embedding_model = settings.openai_embedding_model

      def generate_embedding(self, text: str) -> list[float]:
          """Generate embedding for text"""
          response = self.client.embeddings.create(
              model=self.embedding_model,
              input=text
          )
          return response.data[0].embedding

      async def chat_completion_stream(self, messages: list):
          """Generate chat completion with streaming"""
          stream = self.client.chat.completions.create(
              model=self.model,
              messages=messages,
              stream=True,
              temperature=0.7
          )

          for chunk in stream:
              if chunk.choices[0].delta.content:
                  yield chunk.choices[0].delta.content
  #### 4. File: chatbot-backend/app/services/qdrant_service.py
  python
  from qdrant_client import QdrantClient
  from qdrant_client.models import Distance, VectorParams, PointStruct
  from app.config import get_settings

  class QdrantService:
      def _init_(self):
          settings = get_settings()
          self.client = QdrantClient(
              url=settings.qdrant_url,
              api_key=settings.qdrant_api_key
          )
          self.collection_name = settings.qdrant_collection_name

      def create_collection(self):
          """Create collection if not exists"""
          try:
              self.client.create_collection(
                  collection_name=self.collection_name,
                  vectors_config=VectorParams(
                      size=1536,  # OpenAI embedding dimension
                      distance=Distance.COSINE
                  )
              )
              print(f"Collection '{self.collection_name}' created")
          except Exception as e:
              print(f"Collection might already exist: {e}")

      def search(self, query_vector: list[float], limit: int = 5):
          """Search for similar vectors"""
          results = self.client.search(
              collection_name=self.collection_name,
              query_vector=query_vector,
              limit=limit,
              score_threshold=0.7
          )
          return results

      def upsert_points(self, points: list[PointStruct]):
          """Insert or update points"""
          self.client.upsert(
              collection_name=self.collection_name,
              points=points
          )
  #### 5. File: chatbot-backend/app/services/rag_service.py
  python
  from app.services.openai_service import OpenAIService
  from app.services.qdrant_service import QdrantService

  class RAGService:
      def _init_(self):
          self.openai_service = OpenAIService()
          self.qdrant_service = QdrantService()

      async def process_query(self, user_message: str, selected_text: str = None):
          """Main RAG pipeline"""

          # Handle casual greetings
          casual_greetings = ["hi", "hello", "hey", "how are you"]
          if user_message.lower().strip() in casual_greetings:
              messages = [
                  {"role": "system", "content": "You are a friendly Physical AI textbook assistant."},
                  {"role": "user", "content": user_message}
              ]
          else:
              # RAG pipeline
              # 1. Generate embedding
              query_embedding = self.openai_service.generate_embedding(user_message)

              # 2. Search Qdrant
              search_results = self.qdrant_service.search(query_embedding, limit=5)

              # 3. Build context
              if not search_results:
                  context = "No relevant content found."
              else:
                  context_parts = []
                  for hit in search_results:
                      chapter = hit.payload.get('chapter', 'Unknown')
                      text = hit.payload.get('text', '')
                      context_parts.append(f"[{chapter}]\n{text}")
                  context = "\n\n---\n\n".join(context_parts)

              # 4. Prepare messages
              system_prompt = """You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context. Cite sources (chapters) in your answer."""

              if selected_text:
                  user_prompt = f"Selected Text: {selected_text}\n\nContext: {context}\n\nQuestion: {user_message}"
              else:
                  user_prompt = f"Context: {context}\n\nQuestion: {user_message}"

              messages = [
                  {"role": "system", "content": system_prompt},
                  {"role": "user", "content": user_prompt}
              ]

          # 5. Stream response
          full_response = ""
          async for chunk in self.openai_service.chat_completion_stream(messages):
              full_response += chunk
              yield f"data: {chunk}\n\n"

          # 6. Send sources
          if search_results and user_message.lower().strip() not in casual_greetings:
              sources = [hit.payload.get('chapter', 'Unknown') for hit in search_results]
              yield f"data: [SOURCES]{','.join(sources)}\n\n"
  #### 6. File: chatbot-backend/app/utils/text_chunker.py
  python
  def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
      """Split text into overlapping chunks"""
      chunks = []
      paragraphs = text.split('\n\n')

      current_chunk = ""
      for para in paragraphs:
          para = para.strip()
          if len(para) < 50:  # Skip very short paragraphs
              continue

          if len(current_chunk) + len(para) < chunk_size:
              current_chunk += para + "\n\n"
          else:
              if current_chunk:
                  chunks.append(current_chunk.strip())
              current_chunk = para + "\n\n"

      if current_chunk:
          chunks.append(current_chunk.strip())

      return chunks
  #### 7. File: chatbot-backend/app/routes/chat.py
  python
  from fastapi import APIRouter
  from fastapi.responses import StreamingResponse
  from app.models import ChatRequest
  from app.services.rag_service import RAGService

  router = APIRouter()
  rag_service = RAGService()

  @router.post("/chat")
  async def chat(request: ChatRequest):
      """Main chat endpoint"""
      async def generate():
          async for chunk in rag_service.process_query(
              user_message=request.message,
              selected_text=request.selected_text
          ):
              yield chunk

      return StreamingResponse(generate(), media_type="text/event-stream")
  #### 8. File: chatbot-backend/app/main.py
  python
  from fastapi import FastAPI
  from fastapi.middleware.cors import CORSMiddleware
  from app.config import get_settings
  from app.routes import chat
  from app.routes import indexing

  settings = get_settings()

  app = FastAPI(title="Physical AI Chatbot API")

  # CORS
  app.add_middleware(
      CORSMiddleware,
      allow_origins=settings.cors_origins.split(","),
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )

  # Routes
  app.include_router(chat.router, prefix="/api")
  app.include_router(indexing.router, prefix="/api")

  @app.get("/health")
  async def health():
      return {"status": "ok"}

  if _name_ == "_main_":
      import uvicorn
      uvicorn.run(app, host="0.0.0.0", port=8000)
  #### 9. File: chatbot-backend/requirements.txt
  txt
  fastapi==0.109.0
  uvicorn[standard]==0.27.0
  openai==1.12.0
  qdrant-client==1.7.3
  sqlalchemy==2.0.25
  psycopg2-binary==2.9.9
  pydantic==2.6.0
  pydantic-settings==2.1.0
  python-dotenv==1.0.1
  #### 10. File: chatbot-backend/.env.example
  env
  OPENAI_API_KEY=sk-proj-your-key-here
  OPENAI_MODEL=gpt-4o-mini
  OPENAI_EMBEDDING_MODEL=text-embedding-3-small

  QDRANT_URL=https://your-cluster.qdrant.io
  QDRANT_API_KEY=your-api-key-here
  QDRANT_COLLECTION_NAME=book_content

  DATABASE_URL=postgresql://user:pass@host/db

  CORS_ORIGINS=http://localhost:3000,http://localhost:8000

  ENVIRONMENT=development
  #### 11. File: src/components/ChatbotWidget/ChatbotWidget.tsx
  typescript
  import React, { useState, useRef, useEffect } from 'react';
  import './ChatbotWidget.css';

  interface Message {
    role: 'user' | 'assistant';
    content: string;
    sources?: string[];
  }

  export default function ChatbotWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Detect text selection
    useEffect(() => {
      const handleSelection = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        if (text && text.length > 10) {
          setSelectedText(text);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async () => {
      if (!input.trim()) return;

      const userMessage: Message = { role: 'user', content: input };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      setIsLoading(true);

      try {
        const response = await fetch('http://localhost:8000/api/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: 'user123',
            message: input,
            selected_text: selectedText || null
          })
        });

        const reader = response.body?.getReader();
        const decoder = new TextDecoder();
        let assistantMessage = '';
        let sources: string[] = [];

        while (true) {
          const { done, value } = await reader!.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              const data = line.slice(6);

              if (data.startsWith('[SOURCES]')) {
                sources = data.slice(9).split(',');
              } else {
                assistantMessage += data;
                setMessages(prev => {
                  const newMessages = [...prev];
                  const lastMessage = newMessages[newMessages.length - 1];

                  if (lastMessage?.role === 'assistant') {
                    lastMessage.content = assistantMessage;
                  } else {
                    newMessages.push({
                      role: 'assistant',
                      content: assistantMessage,
                      sources: []
                    });
                  }
                  return newMessages;
                });
              }
            }
          }
        }

        // Update with final sources
        setMessages(prev => {
          const newMessages = [...prev];
          const lastMessage = newMessages[newMessages.length - 1];
          if (lastMessage) {
            lastMessage.sources = sources;
          }
          return newMessages;
        });

      } catch (error) {
        console.error('Error:', error);
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'Sorry, something went wrong. Please try again.'
        }]);
      }

      setIsLoading(false);
      setSelectedText('');
    };

    return (
      <>
        {/* Floating button */}
        <button
          className="chatbot-toggle"
          onClick={() => setIsOpen(!isOpen)}
        >
          💬
        </button>

        {/* Chat panel */}
        {isOpen && (
          <div className="chatbot-panel">
            <div className="chatbot-header">
              <div>
                <h3>Physical AI Assistant</h3>
                <span>Ask me about the book</span>
              </div>
              <button onClick={() => setIsOpen(false)}>×</button>
            </div>

            <div className="chatbot-messages">
              {messages.length === 0 && (
                <div className="welcome-message">
                  👋 Hello! Ask me anything about Physical AI & Humanoid Robotics!
                </div>
              )}

              {messages.map((msg, idx) => (
                <div key={idx} className={`message ${msg.role}`}>
                  <div className="message-content">{msg.content}</div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="sources">
                      {msg.sources.map((source, i) => (
                        <span key={i} className="source-tag">{source}</span>
                      ))}
                    </div>
                  )}
                </div>
              ))}

              {isLoading && (
                <div className="typing-indicator">
                  <span></span><span></span><span></span>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {selectedText && (
              <div className="selected-text-bar">
                ✂ Selected: "{selectedText.substring(0, 50)}..."
                <button onClick={() => setSelectedText('')}>×</button>
              </div>
            )}

            <div className="chatbot-input">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask about Physical AI..."
              />
              <button onClick={sendMessage} disabled={isLoading}>
                →
              </button>
            </div>
          </div>
        )}
      </>
    );
  }
  #### 12. File: src/components/ChatbotWidget/ChatbotWidget.css
  css
  .chatbot-toggle {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 60px;
    height: 60px;
    border-radius: 50%;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 28px;
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    z-index: 9998;
    transition: transform 0.2s;
  }

  .chatbot-toggle:hover {
    transform: scale(1.05);
  }

  .chatbot-panel {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 400px;
    height: 600px;
    background: white;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0,0,0,0.12);
    display: flex;
    flex-direction: column;
    z-index: 9999;
    animation: slideUp 0.3s ease;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
      opacity: 0;
    }
    to {
      transform: translateY(0);
      opacity: 1;
    }
  }

  .chatbot-header {
    background: #2563eb;
    color: white;
    padding: 16px 20px;
    border-radius: 16px 16px 0 0;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chatbot-header h3 {
    margin: 0;
    font-size: 18px;
  }

  .chatbot-header span {
    font-size: 12px;
    opacity: 0.9;
  }

  .chatbot-header button {
    background: none;
    border: none;
    color: white;
    font-size: 32px;
    cursor: pointer;
    line-height: 1;
  }

  .chatbot-messages {
    flex: 1;
    overflow-y: auto;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  .message {
    max-width: 80%;
    padding: 12px 16px;
    border-radius: 18px;
    word-wrap: break-word;
  }

  .message.user {
    background: #2563eb;
    color: white;
    align-self: flex-end;
    border-radius: 18px 18px 4px 18px;
  }

  .message.assistant {
    background: #f8fafc;
    color: #1e293b;
    border: 1px solid #e2e8f0;
    align-self: flex-start;
    border-radius: 18px 18px 18px 4px;
  }

  .sources {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 12px;
  }

  .source-tag {
    background: white;
    border: 1px solid #e2e8f0;
    padding: 4px 12px;
    border-radius: 6px;
    font-size: 12px;
    color: #2563eb;
  }

  .typing-indicator {
    display: flex;
    gap: 4px;
    padding: 12px 16px;
    background: #f8fafc;
    border-radius: 18px;
    width: fit-content;
  }

  .typing-indicator span {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #64748b;
    animation: bounce 1.4s infinite;
  }

  .typing-indicator span:nth-child(2) {
    animation-delay: 0.2s;
  }

  .typing-indicator span:nth-child(3) {
    animation-delay: 0.4s;
  }

  @keyframes bounce {
    0%, 60%, 100% {
      transform: translateY(0);
    }
    30% {
      transform: translateY(-10px);
    }
  }

  .chatbot-input {
    display: flex;
    gap: 12px;
    padding: 16px;
    border-top: 1px solid #e2e8f0;
  }

  .chatbot-input input {
    flex: 1;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    padding: 12px 16px;
    font-size: 14px;
  }

  .chatbot-input button {
    width: 44px;
    height: 44px;
    border-radius: 12px;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  .chatbot-input button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .selected-text-bar {
    background: #dbeafe;
    padding: 12px 16px;
    border-top: 1px solid #e2e8f0;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
  }

  .selected-text-bar button {
    background: none;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  @media (max-width: 768px) {
    .chatbot-panel {
      width: 100vw;
      height: 100vh;
      bottom: 0;
      right: 0;
      border-radius: 0;
    }
  }
  #### 13. File: src/theme/Root.tsx (Modify existing)
  typescript
  import React from 'react';
  import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

  export default function Root({children}) {
    return (
      <>
        {children}
        <ChatbotWidget />
      </>
    );
  }
  ---
## 🚀 STEP-BY-STEP EXECUTION PLAN ### STEP 1: Delete Old Backend
  bash
  # Remove old broken backend
  rm -rf backend/
  rm -rf rag_agent/
  rm -rf vector_search/
  rm -rf database_setup/
  ### STEP 2: Create New Structure
  bash
  # Create new chatbot-backend
  mkdir chatbot-backend
  cd chatbot-backend
  python -m venv venv
  source venv/bin/activate  # Windows: venv\Scripts\activate
  pip install -r requirements.txt
  ### STEP 3: Setup Environment Create .env file with real credentials ### STEP 4: Create All Files Copy all code from specifications
  above ### STEP 5: Test Backend
  bash
  cd chatbot-backend
  python app/main.py
  # Test: http://localhost:8000/health
  ### STEP 6: Index Content
  bash
  curl -X POST http://localhost:8000/api/indexing/rebuild
  ### STEP 7: Add Frontend Create ChatbotWidget component and integrate ### STEP 8: Test Full System Test chat, selected text,
  sources ---
## just give me the further constitution based on my 2. Integrated RAG Chatbot Development: Build and embed a
  Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs,
  FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's
  content, including answering questions based only on text selected by the user. this requirments and add this constitution into my
  rag-constitution.md file which is in .specify/memory/rag-constitution.md do not change any other file be careful
labels: ["constitution", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/rag-constitution.md
tests:
  - null
---

## Prompt

```text
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

---

/sp.constitution # 🤖 RAG CHATBOT IMPLEMENTATION CONSTITUTION - FRESH START ## 🎯 PROJECT GOAL Build a Retrieval-Augmented
  Generation (RAG) chatbot from scratch and embed it in your Docusaurus Physical AI textbook. The chatbot will answer questions about
  the book content using OpenAI, Qdrant vector database, and Neon Postgres. ---
## 📋 REQUIREMENTS CHECKLIST ### Core Technologies
  (Non-Negotiable) - ✅ *OpenAI API* - For embeddings and chat completion - ✅ *FastAPI* - Backend framework - ✅ **Qdrant
  Cloud** - Vector database (free tier) - ✅ *Neon Serverless Postgres* - SQL database - ✅ *Docusaurus* - Frontend (already
  exists) ### Key Features 1. ✅ Answer questions about book content 2. ✅ Support text selection queries 3. ✅ Embedded chatbot
  widget (bottom-right corner) 4. ✅ Real-time AI responses (not hardcoded) 5. ✅ Source citations from book chapters ---
## 🗂 PROJECT STRUCTURE (Start Fresh)
  physical-ai-robotics-textbook/
  ├── docs/                           # YOUR BOOK CONTENT (DO NOT TOUCH)
  │   ├── intro.md
  │   ├── week-1/
  │   ├── week-2/
  │   └── ...
  │
  ├── chatbot-backend/               # NEW - CREATE THIS
  │   ├── app/
  │   │   ├── _init_.py
  │   │   ├── main.py               # FastAPI app
  │   │   ├── config.py             # Configuration
  │   │   ├── models.py             # Pydantic models
  │   │   ├── services/
  │   │   │   ├── _init_.py
  │   │   │   ├── openai_service.py    # OpenAI integration
  │   │   │   ├── qdrant_service.py    # Vector search
  │   │   │   ├── database_service.py  # Postgres operations
  │   │   │   └── rag_service.py       # Main RAG logic
  │   │   ├── routes/
  │   │   │   ├── _init_.py
  │   │   │   ├── chat.py           # Chat endpoints
  │   │   │   └── indexing.py       # Indexing endpoints
  │   │   └── utils/
  │   │       ├── _init_.py
  │   │       ├── text_chunker.py   # Content chunking
  │   │       └── markdown_parser.py # Parse markdown files
  │   ├── requirements.txt
  │   ├── .env
  │   └── .env.example
  │
  ├── src/                           # EXISTING DOCUSAURUS
  │   ├── components/
  │   │   └── ChatbotWidget/        # NEW - CREATE THIS
  │   │       ├── ChatbotWidget.tsx
  │   │       ├── ChatbotWidget.css
  │   │       └── index.ts
  │   └── theme/
  │       └── Root.tsx              # MODIFY - Add chatbot here
  │
  ├── docusaurus.config.js          # EXISTING
  └── package.json                  # EXISTING
  ---
## 🔧 IMPLEMENTATION PHASES ### PHASE 1: Backend Setup (Day 1) *Goal:* Create working FastAPI backend with health check
  *Tasks:* 1. Create chatbot-backend/ folder 2. Create virtual environment 3. Install dependencies 4. Create basic FastAPI app with
  health check 5. Test: curl http://localhost:8000/health *Deliverable:* Backend starts without errors ---
### PHASE 2:
  Database
  Setup (Day 1) *Goal:* Connect to Neon Postgres and create tables *Tasks:* 1. Set up Neon Postgres database 2. Create database
  models (conversations, messages) 3. Create database connection 4. Test connection *Deliverable:* Database tables created
  successfully ---
### PHASE 3: Vector Database Setup (Day 1-2) *Goal:* Connect to Qdrant and create collection *Tasks:* 1. Set
  up Qdrant Cloud account (free tier) 2. Create collection for book content 3. Test connection 4. Verify collection exists
  *Deliverable:* Qdrant collection ready for indexing ---
### PHASE 4: Content Indexing (Day 2) *Goal:* Read markdown files and
  store in Qdrant *Tasks:* 1. Create markdown parser 2. Create text chunker (500-1000 tokens per chunk) 3. Generate embeddings with
  OpenAI 4. Store chunks in Qdrant with metadata 5. Index all content from docs/ folder *Deliverable:* All book content indexed in
  Qdrant ---
### PHASE 5: RAG Engine (Day 2-3) *Goal:* Implement core RAG functionality *Tasks:* 1. Create query → embedding
  function 2. Create vector search function 3. Create context builder 4. Create OpenAI chat completion with streaming 5. Handle
  casual greetings separately *Deliverable:* RAG pipeline works end-to-end ---
### PHASE 6: API Endpoints (Day 3) *Goal:* Create
  REST API for chat *Tasks:* 1. POST /api/chat - General questions 2. POST /api/chat/selected-text - Questions about selected text
  3. POST /api/indexing/rebuild - Reindex content 4. GET /api/indexing/status - Check indexing status 5. Add CORS middleware
  *Deliverable:* All endpoints working and tested ---
### PHASE 7: Frontend Widget (Day 3-4) *Goal:* Create chatbot UI component
  *Tasks:* 1. Create React component (ChatbotWidget.tsx) 2. Style with CSS (bottom-right floating button) 3. Implement chat UI
  (messages, input, sources) 4. Add text selection detection 5. Connect to backend API *Deliverable:* Chatbot visible and
  functional on site ---
### PHASE 8: Integration & Testing (Day 4) *Goal:* Integrate everything and test *Tasks:* 1. Embed
  chatbot in Docusaurus 2. Test all features 3. Fix bugs 4. Deploy backend 5. Update frontend API URLs *Deliverable:* Fully working
  chatbot on live site ---
## 📝 DETAILED SPECIFICATIONS ### Backend Specifications #### 1. File: chatbot-backend/app/config.py
  python
  from pydantic_settings import BaseSettings
  from functools import lru_cache

  class Settings(BaseSettings):
      # OpenAI
      openai_api_key: str
      openai_model: str = "gpt-4o-mini"
      openai_embedding_model: str = "text-embedding-3-small"

      # Qdrant
      qdrant_url: str
      qdrant_api_key: str
      qdrant_collection_name: str = "book_content"

      # Database
      database_url: str

      # CORS
      cors_origins: str = "http://localhost:3000"

      # App
      environment: str = "development"

      class Config:
          env_file = ".env"

  @lru_cache()
  def get_settings():
      return Settings()
  #### 2. File: chatbot-backend/app/models.py
  python
  from pydantic import BaseModel
  from typing import Optional

  class ChatRequest(BaseModel):
      user_id: str
      message: str
      conversation_id: Optional[int] = None
      selected_text: Optional[str] = None

  class ChatResponse(BaseModel):
      answer: str
      sources: list[str] = []
      conversation_id: int

  class IndexingStatus(BaseModel):
      is_indexing: bool
      total_files: int
      indexed_files: int
      last_indexed: Optional[str] = None
      error: Optional[str] = None
  #### 3. File: chatbot-backend/app/services/openai_service.py
  python
  from openai import OpenAI
  from app.config import get_settings

  class OpenAIService:
      def _init_(self):
          settings = get_settings()
          self.client = OpenAI(api_key=settings.openai_api_key)
          self.model = settings.openai_model
          self.embedding_model = settings.openai_embedding_model

      def generate_embedding(self, text: str) -> list[float]:
          """Generate embedding for text"""
          response = self.client.embeddings.create(
              model=self.embedding_model,
              input=text
          )
          return response.data[0].embedding

      async def chat_completion_stream(self, messages: list):
          """Generate chat completion with streaming"""
          stream = self.client.chat.completions.create(
              model=self.model,
              messages=messages,
              stream=True,
              temperature=0.7
          )

          for chunk in stream:
              if chunk.choices[0].delta.content:
                  yield chunk.choices[0].delta.content
  #### 4. File: chatbot-backend/app/services/qdrant_service.py
  python
  from qdrant_client import QdrantClient
  from qdrant_client.models import Distance, VectorParams, PointStruct
  from app.config import get_settings

  class QdrantService:
      def _init_(self):
          settings = get_settings()
          self.client = QdrantClient(
              url=settings.qdrant_url,
              api_key=settings.qdrant_api_key
          )
          self.collection_name = settings.qdrant_collection_name

      def create_collection(self):
          """Create collection if not exists"""
          try:
              self.client.create_collection(
                  collection_name=self.collection_name,
                  vectors_config=VectorParams(
                      size=1536,  # OpenAI embedding dimension
                      distance=Distance.COSINE
                  )
              )
              print(f"Collection '{self.collection_name}' created")
          except Exception as e:
              print(f"Collection might already exist: {e}")

      def search(self, query_vector: list[float], limit: int = 5):
          """Search for similar vectors"""
          results = self.client.search(
              collection_name=self.collection_name,
              query_vector=query_vector,
              limit=limit,
              score_threshold=0.7
          )
          return results

      def upsert_points(self, points: list[PointStruct]):
          """Insert or update points"""
          self.client.upsert(
              collection_name=self.collection_name,
              points=points
          )
  #### 5. File: chatbot-backend/app/services/rag_service.py
  python
  from app.services.openai_service import OpenAIService
  from app.services.qdrant_service import QdrantService

  class RAGService:
      def _init_(self):
          self.openai_service = OpenAIService()
          self.qdrant_service = QdrantService()

      async def process_query(self, user_message: str, selected_text: str = None):
          """Main RAG pipeline"""

          # Handle casual greetings
          casual_greetings = ["hi", "hello", "hey", "how are you"]
          if user_message.lower().strip() in casual_greetings:
              messages = [
                  {"role": "system", "content": "You are a friendly Physical AI textbook assistant."},
                  {"role": "user", "content": user_message}
              ]
          else:
              # RAG pipeline
              # 1. Generate embedding
              query_embedding = self.openai_service.generate_embedding(user_message)

              # 2. Search Qdrant
              search_results = self.qdrant_service.search(query_embedding, limit=5)

              # 3. Build context
              if not search_results:
                  context = "No relevant content found."
              else:
                  context_parts = []
                  for hit in search_results:
                      chapter = hit.payload.get('chapter', 'Unknown')
                      text = hit.payload.get('text', '')
                      context_parts.append(f"[{chapter}]\n{text}")
                  context = "\n\n---\n\n".join(context_parts)

              # 4. Prepare messages
              system_prompt = """You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context. Cite sources (chapters) in your answer."""

              if selected_text:
                  user_prompt = f"Selected Text: {selected_text}\n\nContext: {context}\n\nQuestion: {user_message}"
              else:
                  user_prompt = f"Context: {context}\n\nQuestion: {user_message}"

              messages = [
                  {"role": "system", "content": system_prompt},
                  {"role": "user", "content": user_prompt}
              ]

          # 5. Stream response
          full_response = ""
          async for chunk in self.openai_service.chat_completion_stream(messages):
              full_response += chunk
              yield f"data: {chunk}\n\n"

          # 6. Send sources
          if search_results and user_message.lower().strip() not in casual_greetings:
              sources = [hit.payload.get('chapter', 'Unknown') for hit in search_results]
              yield f"data: [SOURCES]{','.join(sources)}\n\n"
  #### 6. File: chatbot-backend/app/utils/text_chunker.py
  python
  def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
      """Split text into overlapping chunks"""
      chunks = []
      paragraphs = text.split('\n\n')

      current_chunk = ""
      for para in paragraphs:
          para = para.strip()
          if len(para) < 50:  # Skip very short paragraphs
              continue

          if len(current_chunk) + len(para) < chunk_size:
              current_chunk += para + "\n\n"
          else:
              if current_chunk:
                  chunks.append(current_chunk.strip())
              current_chunk = para + "\n\n"

      if current_chunk:
          chunks.append(current_chunk.strip())

      return chunks
  #### 7. File: chatbot-backend/app/routes/chat.py
  python
  from fastapi import APIRouter
  from fastapi.responses import StreamingResponse
  from app.models import ChatRequest
  from app.services.rag_service import RAGService

  router = APIRouter()
  rag_service = RAGService()

  @router.post("/chat")
  async def chat(request: ChatRequest):
      """Main chat endpoint"""
      async def generate():
          async for chunk in rag_service.process_query(
              user_message=request.message,
              selected_text=request.selected_text
          ):
              yield chunk

      return StreamingResponse(generate(), media_type="text/event-stream")
  #### 8. File: chatbot-backend/app/main.py
  python
  from fastapi import FastAPI
  from fastapi.middleware.cors import CORSMiddleware
  from app.config import get_settings
  from app.routes import chat
  from app.routes import indexing

  settings = get_settings()

  app = FastAPI(title="Physical AI Chatbot API")

  # CORS
  app.add_middleware(
      CORSMiddleware,
      allow_origins=settings.cors_origins.split(","),
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )

  # Routes
  app.include_router(chat.router, prefix="/api")
  app.include_router(indexing.router, prefix="/api")

  @app.get("/health")
  async def health():
      return {"status": "ok"}

  if _name_ == "_main_":
      import uvicorn
      uvicorn.run(app, host="0.0.0.0", port=8000)
  #### 9. File: chatbot-backend/requirements.txt
  txt
  fastapi==0.109.0
  uvicorn[standard]==0.27.0
  openai==1.12.0
  qdrant-client==1.7.3
  sqlalchemy==2.0.25
  psycopg2-binary==2.9.9
  pydantic==2.6.0
  pydantic-settings==2.1.0
  python-dotenv==1.0.1
  #### 10. File: chatbot-backend/.env.example
  env
  OPENAI_API_KEY=sk-proj-your-key-here
  OPENAI_MODEL=gpt-4o-mini
  OPENAI_EMBEDDING_MODEL=text-embedding-3-small

  QDRANT_URL=https://your-cluster.qdrant.io
  QDRANT_API_KEY=your-api-key-here
  QDRANT_COLLECTION_NAME=book_content

  DATABASE_URL=postgresql://user:pass@host/db

  CORS_ORIGINS=http://localhost:3000,http://localhost:8000

  ENVIRONMENT=development
  #### 11. File: src/components/ChatbotWidget/ChatbotWidget.tsx
  typescript
  import React, { useState, useRef, useEffect } from 'react';
  import './ChatbotWidget.css';

  interface Message {
    role: 'user' | 'assistant';
    content: string;
    sources?: string[];
  }

  export default function ChatbotWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Detect text selection
    useEffect(() => {
      const handleSelection = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        if (text && text.length > 10) {
          setSelectedText(text);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async () => {
      if (!input.trim()) return;

      const userMessage: Message = { role: 'user', content: input };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      setIsLoading(true);

      try {
        const response = await fetch('http://localhost:8000/api/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: 'user123',
            message: input,
            selected_text: selectedText || null
          })
        });

        const reader = response.body?.getReader();
        const decoder = new TextDecoder();
        let assistantMessage = '';
        let sources: string[] = [];

        while (true) {
          const { done, value } = await reader!.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              const data = line.slice(6);

              if (data.startsWith('[SOURCES]')) {
                sources = data.slice(9).split(',');
              } else {
                assistantMessage += data;
                setMessages(prev => {
                  const newMessages = [...prev];
                  const lastMessage = newMessages[newMessages.length - 1];

                  if (lastMessage?.role === 'assistant') {
                    lastMessage.content = assistantMessage;
                  } else {
                    newMessages.push({
                      role: 'assistant',
                      content: assistantMessage,
                      sources: []
                    });
                  }
                  return newMessages;
                });
              }
            }
          }
        }

        // Update with final sources
        setMessages(prev => {
          const newMessages = [...prev];
          const lastMessage = newMessages[newMessages.length - 1];
          if (lastMessage) {
            lastMessage.sources = sources;
          }
          return newMessages;
        });

      } catch (error) {
        console.error('Error:', error);
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'Sorry, something went wrong. Please try again.'
        }]);
      }

      setIsLoading(false);
      setSelectedText('');
    };

    return (
      <>
        {/* Floating button */}
        <button
          className="chatbot-toggle"
          onClick={() => setIsOpen(!isOpen)}
        >
          💬
        </button>

        {/* Chat panel */}
        {isOpen && (
          <div className="chatbot-panel">
            <div className="chatbot-header">
              <div>
                <h3>Physical AI Assistant</h3>
                <span>Ask me about the book</span>
              </div>
              <button onClick={() => setIsOpen(false)}>×</button>
            </div>

            <div className="chatbot-messages">
              {messages.length === 0 && (
                <div className="welcome-message">
                  👋 Hello! Ask me anything about Physical AI & Humanoid Robotics!
                </div>
              )}

              {messages.map((msg, idx) => (
                <div key={idx} className={`message ${msg.role}`}>
                  <div className="message-content">{msg.content}</div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="sources">
                      {msg.sources.map((source, i) => (
                        <span key={i} className="source-tag">{source}</span>
                      ))}
                    </div>
                  )}
                </div>
              ))}

              {isLoading && (
                <div className="typing-indicator">
                  <span></span><span></span><span></span>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {selectedText && (
              <div className="selected-text-bar">
                ✂ Selected: "{selectedText.substring(0, 50)}..."
                <button onClick={() => setSelectedText('')}>×</button>
              </div>
            )}

            <div className="chatbot-input">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask about Physical AI..."
              />
              <button onClick={sendMessage} disabled={isLoading}>
                →
              </button>
            </div>
          </div>
        )}
      </>
    );
  }
  #### 12. File: src/components/ChatbotWidget/ChatbotWidget.css
  css
  .chatbot-toggle {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 60px;
    height: 60px;
    border-radius: 50%;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 28px;
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    z-index: 9998;
    transition: transform 0.2s;
  }

  .chatbot-toggle:hover {
    transform: scale(1.05);
  }

  .chatbot-panel {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 400px;
    height: 600px;
    background: white;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0,0,0,0.12);
    display: flex;
    flex-direction: column;
    z-index: 9999;
    animation: slideUp 0.3s ease;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
      opacity: 0;
    }
    to {
      transform: translateY(0);
      opacity: 1;
    }
  }

  .chatbot-header {
    background: #2563eb;
    color: white;
    padding: 16px 20px;
    border-radius: 16px 16px 0 0;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chatbot-header h3 {
    margin: 0;
    font-size: 18px;
  }

  .chatbot-header span {
    font-size: 12px;
    opacity: 0.9;
  }

  .chatbot-header button {
    background: none;
    border: none;
    color: white;
    font-size: 32px;
    cursor: pointer;
    line-height: 1;
  }

  .chatbot-messages {
    flex: 1;
    overflow-y: auto;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  .message {
    max-width: 80%;
    padding: 12px 16px;
    border-radius: 18px;
    word-wrap: break-word;
  }

  .message.user {
    background: #2563eb;
    color: white;
    align-self: flex-end;
    border-radius: 18px 18px 4px 18px;
  }

  .message.assistant {
    background: #f8fafc;
    color: #1e293b;
    border: 1px solid #e2e8f0;
    align-self: flex-start;
    border-radius: 18px 18px 18px 4px;
  }

  .sources {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 12px;
  }

  .source-tag {
    background: white;
    border: 1px solid #e2e8f0;
    padding: 4px 12px;
    border-radius: 6px;
    font-size: 12px;
    color: #2563eb;
  }

  .typing-indicator {
    display: flex;
    gap: 4px;
    padding: 12px 16px;
    background: #f8fafc;
    border-radius: 18px;
    width: fit-content;
  }

  .typing-indicator span {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #64748b;
    animation: bounce 1.4s infinite;
  }

  .typing-indicator span:nth-child(2) {
    animation-delay: 0.2s;
  }

  .typing-indicator span:nth-child(3) {
    animation-delay: 0.4s;
  }

  @keyframes bounce {
    0%, 60%, 100% {
      transform: translateY(0);
    }
    30% {
      transform: translateY(-10px);
    }
  }

  .chatbot-input {
    display: flex;
    gap: 12px;
    padding: 16px;
    border-top: 1px solid #e2e8f0;
  }

  .chatbot-input input {
    flex: 1;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    padding: 12px 16px;
    font-size: 14px;
  }

  .chatbot-input button {
    width: 44px;
    height: 44px;
    border-radius: 12px;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  .chatbot-input button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .selected-text-bar {
    background: #dbeafe;
    padding: 12px 16px;
    border-top: 1px solid #e2e8f0;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
  }

  .selected-text-bar button {
    background: none;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  @media (max-width: 768px) {
    .chatbot-panel {
      width: 100vw;
      height: 100vh;
      bottom: 0;
      right: 0;
      border-radius: 0;
    }
  }
  #### 13. File: src/theme/Root.tsx (Modify existing)
  typescript
  import React from 'react';
  import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

  export default function Root({children}) {
    return (
      <>
        {children}
        <ChatbotWidget />
      </>
    );
  }
  ---
## 🚀 STEP-BY-STEP EXECUTION PLAN ### STEP 1: Delete Old Backend
  bash
  # Remove old broken backend
  rm -rf backend/
  rm -rf rag_agent/
  rm -rf vector_search/
  rm -rf database_setup/
  ### STEP 2: Create New Structure
  bash
  # Create new chatbot-backend
  mkdir chatbot-backend
  cd chatbot-backend
  python -m venv venv
  source venv/bin/activate  # Windows: venv\Scripts\activate
  pip install -r requirements.txt
  ### STEP 3: Setup Environment Create .env file with real credentials ### STEP 4: Create All Files Copy all code from specifications
  above ### STEP 5: Test Backend
  bash
  cd chatbot-backend
  python app/main.py
  # Test: http://localhost:8000/health
  ### STEP 6: Index Content
  bash
  curl -X POST http://localhost:8000/api/indexing/rebuild
  ### STEP 7: Add Frontend Create ChatbotWidget component and integrate ### STEP 8: Test Full System Test chat, selected text,
  sources ---
## just give me the further constitution based on my 2. Integrated RAG Chatbot Development: Build and embed a
  Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs,
  FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's
  content, including answering questions based only on text selected by the user. this requirments and add this constitution into my
  rag-constitution.md file which is in .specify/memory/rag-constitution.md do not change any other file be careful
labels: ["constitution", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/rag-constitution.md
tests:
  - null
---

## Prompt

```text
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

---

/sp.constitution # 🤖 RAG CHATBOT IMPLEMENTATION CONSTITUTION - FRESH START ## 🎯 PROJECT GOAL Build a Retrieval-Augmented
  Generation (RAG) chatbot from scratch and embed it in your Docusaurus Physical AI textbook. The chatbot will answer questions about
  the book content using OpenAI, Qdrant vector database, and Neon Postgres. ---
## 📋 REQUIREMENTS CHECKLIST ### Core Technologies
  (Non-Negotiable) - ✅ *OpenAI API* - For embeddings and chat completion - ✅ *FastAPI* - Backend framework - ✅ **Qdrant
  Cloud** - Vector database (free tier) - ✅ *Neon Serverless Postgres* - SQL database - ✅ *Docusaurus* - Frontend (already
  exists) ### Key Features 1. ✅ Answer questions about book content 2. ✅ Support text selection queries 3. ✅ Embedded chatbot
  widget (bottom-right corner) 4. ✅ Real-time AI responses (not hardcoded) 5. ✅ Source citations from book chapters ---
## 🗂 PROJECT STRUCTURE (Start Fresh)
  physical-ai-robotics-textbook/
  ├── docs/                           # YOUR BOOK CONTENT (DO NOT TOUCH)
  │   ├── intro.md
  │   ├── week-1/
  │   ├── week-2/
  │   └── ...
  │
  ├── chatbot-backend/               # NEW - CREATE THIS
  │   ├── app/
  │   │   ├── _init_.py
  │   │   ├── main.py               # FastAPI app
  │   │   ├── config.py             # Configuration
  │   │   ├── models.py             # Pydantic models
  │   │   ├── services/
  │   │   │   ├── _init_.py
  │   │   │   ├── openai_service.py    # OpenAI integration
  │   │   │   ├── qdrant_service.py    # Vector search
  │   │   │   ├── database_service.py  # Postgres operations
  │   │   │   └── rag_service.py       # Main RAG logic
  │   │   ├── routes/
  │   │   │   ├── _init_.py
  │   │   │   ├── chat.py           # Chat endpoints
  │   │   │   └── indexing.py       # Indexing endpoints
  │   │   └── utils/
  │   │       ├── _init_.py
  │   │       ├── text_chunker.py   # Content chunking
  │   │       └── markdown_parser.py # Parse markdown files
  │   ├── requirements.txt
  │   ├── .env
  │   └── .env.example
  │
  ├── src/                           # EXISTING DOCUSAURUS
  │   ├── components/
  │   │   └── ChatbotWidget/        # NEW - CREATE THIS
  │   │       ├── ChatbotWidget.tsx
  │   │       ├── ChatbotWidget.css
  │   │       └── index.ts
  │   └── theme/
  │       └── Root.tsx              # MODIFY - Add chatbot here
  │
  ├── docusaurus.config.js          # EXISTING
  └── package.json                  # EXISTING
  ---
## 🔧 IMPLEMENTATION PHASES ### PHASE 1: Backend Setup (Day 1) *Goal:* Create working FastAPI backend with health check
  *Tasks:* 1. Create chatbot-backend/ folder 2. Create virtual environment 3. Install dependencies 4. Create basic FastAPI app with
  health check 5. Test: curl http://localhost:8000/health *Deliverable:* Backend starts without errors ---
### PHASE 2:
  Database
  Setup (Day 1) *Goal:* Connect to Neon Postgres and create tables *Tasks:* 1. Set up Neon Postgres database 2. Create database
  models (conversations, messages) 3. Create database connection 4. Test connection *Deliverable:* Database tables created
  successfully ---
### PHASE 3: Vector Database Setup (Day 1-2) *Goal:* Connect to Qdrant and create collection *Tasks:* 1. Set
  up Qdrant Cloud account (free tier) 2. Create collection for book content 3. Test connection 4. Verify collection exists
  *Deliverable:* Qdrant collection ready for indexing ---
### PHASE 4: Content Indexing (Day 2) *Goal:* Read markdown files and
  store in Qdrant *Tasks:* 1. Create markdown parser 2. Create text chunker (500-1000 tokens per chunk) 3. Generate embeddings with
  OpenAI 4. Store chunks in Qdrant with metadata 5. Index all content from docs/ folder *Deliverable:* All book content indexed in
  Qdrant ---
### PHASE 5: RAG Engine (Day 2-3) *Goal:* Implement core RAG functionality *Tasks:* 1. Create query → embedding
  function 2. Create vector search function 3. Create context builder 4. Create OpenAI chat completion with streaming 5. Handle
  casual greetings separately *Deliverable:* RAG pipeline works end-to-end ---
### PHASE 6: API Endpoints (Day 3) *Goal:* Create
  REST API for chat *Tasks:* 1. POST /api/chat - General questions 2. POST /api/chat/selected-text - Questions about selected text
  3. POST /api/indexing/rebuild - Reindex content 4. GET /api/indexing/status - Check indexing status 5. Add CORS middleware
  *Deliverable:* All endpoints working and tested ---
### PHASE 7: Frontend Widget (Day 3-4) *Goal:* Create chatbot UI component
  *Tasks:* 1. Create React component (ChatbotWidget.tsx) 2. Style with CSS (bottom-right floating button) 3. Implement chat UI
  (messages, input, sources) 4. Add text selection detection 5. Connect to backend API *Deliverable:* Chatbot visible and
  functional on site ---
### PHASE 8: Integration & Testing (Day 4) *Goal:* Integrate everything and test *Tasks:* 1. Embed
  chatbot in Docusaurus 2. Test all features 3. Fix bugs 4. Deploy backend 5. Update frontend API URLs *Deliverable:* Fully working
  chatbot on live site ---
## 📝 DETAILED SPECIFICATIONS ### Backend Specifications #### 1. File: chatbot-backend/app/config.py
  python
  from pydantic_settings import BaseSettings
  from functools import lru_cache

  class Settings(BaseSettings):
      # OpenAI
      openai_api_key: str
      openai_model: str = "gpt-4o-mini"
      openai_embedding_model: str = "text-embedding-3-small"

      # Qdrant
      qdrant_url: str
      qdrant_api_key: str
      qdrant_collection_name: str = "book_content"

      # Database
      database_url: str

      # CORS
      cors_origins: str = "http://localhost:3000"

      # App
      environment: str = "development"

      class Config:
          env_file = ".env"

  @lru_cache()
  def get_settings():
      return Settings()
  #### 2. File: chatbot-backend/app/models.py
  python
  from pydantic import BaseModel
  from typing import Optional

  class ChatRequest(BaseModel):
      user_id: str
      message: str
      conversation_id: Optional[int] = None
      selected_text: Optional[str] = None

  class ChatResponse(BaseModel):
      answer: str
      sources: list[str] = []
      conversation_id: int

  class IndexingStatus(BaseModel):
      is_indexing: bool
      total_files: int
      indexed_files: int
      last_indexed: Optional[str] = None
      error: Optional[str] = None
  #### 3. File: chatbot-backend/app/services/openai_service.py
  python
  from openai import OpenAI
  from app.config import get_settings

  class OpenAIService:
      def _init_(self):
          settings = get_settings()
          self.client = OpenAI(api_key=settings.openai_api_key)
          self.model = settings.openai_model
          self.embedding_model = settings.openai_embedding_model

      def generate_embedding(self, text: str) -> list[float]:
          """Generate embedding for text"""
          response = self.client.embeddings.create(
              model=self.embedding_model,
              input=text
          )
          return response.data[0].embedding

      async def chat_completion_stream(self, messages: list):
          """Generate chat completion with streaming"""
          stream = self.client.chat.completions.create(
              model=self.model,
              messages=messages,
              stream=True,
              temperature=0.7
          )

          for chunk in stream:
              if chunk.choices[0].delta.content:
                  yield chunk.choices[0].delta.content
  #### 4. File: chatbot-backend/app/services/qdrant_service.py
  python
  from qdrant_client import QdrantClient
  from qdrant_client.models import Distance, VectorParams, PointStruct
  from app.config import get_settings

  class QdrantService:
      def _init_(self):
          settings = get_settings()
          self.client = QdrantClient(
              url=settings.qdrant_url,
              api_key=settings.qdrant_api_key
          )
          self.collection_name = settings.qdrant_collection_name

      def create_collection(self):
          """Create collection if not exists"""
          try:
              self.client.create_collection(
                  collection_name=self.collection_name,
                  vectors_config=VectorParams(
                      size=1536,  # OpenAI embedding dimension
                      distance=Distance.COSINE
                  )
              )
              print(f"Collection '{self.collection_name}' created")
          except Exception as e:
              print(f"Collection might already exist: {e}")

      def search(self, query_vector: list[float], limit: int = 5):
          """Search for similar vectors"""
          results = self.client.search(
              collection_name=self.collection_name,
              query_vector=query_vector,
              limit=limit,
              score_threshold=0.7
          )
          return results

      def upsert_points(self, points: list[PointStruct]):
          """Insert or update points"""
          self.client.upsert(
              collection_name=self.collection_name,
              points=points
          )
  #### 5. File: chatbot-backend/app/services/rag_service.py
  python
  from app.services.openai_service import OpenAIService
  from app.services.qdrant_service import QdrantService

  class RAGService:
      def _init_(self):
          self.openai_service = OpenAIService()
          self.qdrant_service = QdrantService()

      async def process_query(self, user_message: str, selected_text: str = None):
          """Main RAG pipeline"""

          # Handle casual greetings
          casual_greetings = ["hi", "hello", "hey", "how are you"]
          if user_message.lower().strip() in casual_greetings:
              messages = [
                  {"role": "system", "content": "You are a friendly Physical AI textbook assistant."},
                  {"role": "user", "content": user_message}
              ]
          else:
              # RAG pipeline
              # 1. Generate embedding
              query_embedding = self.openai_service.generate_embedding(user_message)

              # 2. Search Qdrant
              search_results = self.qdrant_service.search(query_embedding, limit=5)

              # 3. Build context
              if not search_results:
                  context = "No relevant content found."
              else:
                  context_parts = []
                  for hit in search_results:
                      chapter = hit.payload.get('chapter', 'Unknown')
                      text = hit.payload.get('text', '')
                      context_parts.append(f"[{chapter}]\n{text}")
                  context = "\n\n---\n\n".join(context_parts)

              # 4. Prepare messages
              system_prompt = """You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context. Cite sources (chapters) in your answer."""

              if selected_text:
                  user_prompt = f"Selected Text: {selected_text}\n\nContext: {context}\n\nQuestion: {user_message}"
              else:
                  user_prompt = f"Context: {context}\n\nQuestion: {user_message}"

              messages = [
                  {"role": "system", "content": system_prompt},
                  {"role": "user", "content": user_prompt}
              ]

          # 5. Stream response
          full_response = ""
          async for chunk in self.openai_service.chat_completion_stream(messages):
              full_response += chunk
              yield f"data: {chunk}\n\n"

          # 6. Send sources
          if search_results and user_message.lower().strip() not in casual_greetings:
              sources = [hit.payload.get('chapter', 'Unknown') for hit in search_results]
              yield f"data: [SOURCES]{','.join(sources)}\n\n"
  #### 6. File: chatbot-backend/app/utils/text_chunker.py
  python
  def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
      """Split text into overlapping chunks"""
      chunks = []
      paragraphs = text.split('\n\n')

      current_chunk = ""
      for para in paragraphs:
          para = para.strip()
          if len(para) < 50:  # Skip very short paragraphs
              continue

          if len(current_chunk) + len(para) < chunk_size:
              current_chunk += para + "\n\n"
          else:
              if current_chunk:
                  chunks.append(current_chunk.strip())
              current_chunk = para + "\n\n"

      if current_chunk:
          chunks.append(current_chunk.strip())

      return chunks
  #### 7. File: chatbot-backend/app/routes/chat.py
  python
  from fastapi import APIRouter
  from fastapi.responses import StreamingResponse
  from app.models import ChatRequest
  from app.services.rag_service import RAGService

  router = APIRouter()
  rag_service = RAGService()

  @router.post("/chat")
  async def chat(request: ChatRequest):
      """Main chat endpoint"""
      async def generate():
          async for chunk in rag_service.process_query(
              user_message=request.message,
              selected_text=request.selected_text
          ):
              yield chunk

      return StreamingResponse(generate(), media_type="text/event-stream")
  #### 8. File: chatbot-backend/app/main.py
  python
  from fastapi import FastAPI
  from fastapi.middleware.cors import CORSMiddleware
  from app.config import get_settings
  from app.routes import chat
  from app.routes import indexing

  settings = get_settings()

  app = FastAPI(title="Physical AI Chatbot API")

  # CORS
  app.add_middleware(
      CORSMiddleware,
      allow_origins=settings.cors_origins.split(","),
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )

  # Routes
  app.include_router(chat.router, prefix="/api")
  app.include_router(indexing.router, prefix="/api")

  @app.get("/health")
  async def health():
      return {"status": "ok"}

  if _name_ == "_main_":
      import uvicorn
      uvicorn.run(app, host="0.0.0.0", port=8000)
  #### 9. File: chatbot-backend/requirements.txt
  txt
  fastapi==0.109.0
  uvicorn[standard]==0.27.0
  openai==1.12.0
  qdrant-client==1.7.3
  sqlalchemy==2.0.25
  psycopg2-binary==2.9.9
  pydantic==2.6.0
  pydantic-settings==2.1.0
  python-dotenv==1.0.1
  #### 10. File: chatbot-backend/.env.example
  env
  OPENAI_API_KEY=sk-proj-your-key-here
  OPENAI_MODEL=gpt-4o-mini
  OPENAI_EMBEDDING_MODEL=text-embedding-3-small

  QDRANT_URL=https://your-cluster.qdrant.io
  QDRANT_API_KEY=your-api-key-here
  QDRANT_COLLECTION_NAME=book_content

  DATABASE_URL=postgresql://user:pass@host/db

  CORS_ORIGINS=http://localhost:3000,http://localhost:8000

  ENVIRONMENT=development
  #### 11. File: src/components/ChatbotWidget/ChatbotWidget.tsx
  typescript
  import React, { useState, useRef, useEffect } from 'react';
  import './ChatbotWidget.css';

  interface Message {
    role: 'user' | 'assistant';
    content: string;
    sources?: string[];
  }

  export default function ChatbotWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    // Detect text selection
    useEffect(() => {
      const handleSelection = () => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        if (text && text.length > 10) {
          setSelectedText(text);
        }
      };

      document.addEventListener('mouseup', handleSelection);
      return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async () => {
      if (!input.trim()) return;

      const userMessage: Message = { role: 'user', content: input };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      setIsLoading(true);

      try {
        const response = await fetch('http://localhost:8000/api/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            user_id: 'user123',
            message: input,
            selected_text: selectedText || null
          })
        });

        const reader = response.body?.getReader();
        const decoder = new TextDecoder();
        let assistantMessage = '';
        let sources: string[] = [];

        while (true) {
          const { done, value } = await reader!.read();
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              const data = line.slice(6);

              if (data.startsWith('[SOURCES]')) {
                sources = data.slice(9).split(',');
              } else {
                assistantMessage += data;
                setMessages(prev => {
                  const newMessages = [...prev];
                  const lastMessage = newMessages[newMessages.length - 1];

                  if (lastMessage?.role === 'assistant') {
                    lastMessage.content = assistantMessage;
                  } else {
                    newMessages.push({
                      role: 'assistant',
                      content: assistantMessage,
                      sources: []
                    });
                  }
                  return newMessages;
                });
              }
            }
          }
        }

        // Update with final sources
        setMessages(prev => {
          const newMessages = [...prev];
          const lastMessage = newMessages[newMessages.length - 1];
          if (lastMessage) {
            lastMessage.sources = sources;
          }
          return newMessages;
        });

      } catch (error) {
        console.error('Error:', error);
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'Sorry, something went wrong. Please try again.'
        }]);
      }

      setIsLoading(false);
      setSelectedText('');
    };

    return (
      <>
        {/* Floating button */}
        <button
          className="chatbot-toggle"
          onClick={() => setIsOpen(!isOpen)}
        >
          💬
        </button>

        {/* Chat panel */}
        {isOpen && (
          <div className="chatbot-panel">
            <div className="chatbot-header">
              <div>
                <h3>Physical AI Assistant</h3>
                <span>Ask me about the book</span>
              </div>
              <button onClick={() => setIsOpen(false)}>×</button>
            </div>

            <div className="chatbot-messages">
              {messages.length === 0 && (
                <div className="welcome-message">
                  👋 Hello! Ask me anything about Physical AI & Humanoid Robotics!
                </div>
              )}

              {messages.map((msg, idx) => (
                <div key={idx} className={`message ${msg.role}`}>
                  <div className="message-content">{msg.content}</div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="sources">
                      {msg.sources.map((source, i) => (
                        <span key={i} className="source-tag">{source}</span>
                      ))}
                    </div>
                  )}
                </div>
              ))}

              {isLoading && (
                <div className="typing-indicator">
                  <span></span><span></span><span></span>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {selectedText && (
              <div className="selected-text-bar">
                ✂ Selected: "{selectedText.substring(0, 50)}..."
                <button onClick={() => setSelectedText('')}>×</button>
              </div>
            )}

            <div className="chatbot-input">
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask about Physical AI..."
              />
              <button onClick={sendMessage} disabled={isLoading}>
                →
              </button>
            </div>
          </div>
        )}
      </>
    );
  }
  #### 12. File: src/components/ChatbotWidget/ChatbotWidget.css
  css
  .chatbot-toggle {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 60px;
    height: 60px;
    border-radius: 50%;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 28px;
    cursor: pointer;
    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    z-index: 9998;
    transition: transform 0.2s;
  }

  .chatbot-toggle:hover {
    transform: scale(1.05);
  }

  .chatbot-panel {
    position: fixed;
    bottom: 20px;
    right: 20px;
    width: 400px;
    height: 600px;
    background: white;
    border-radius: 16px;
    box-shadow: 0 8px 32px rgba(0,0,0,0.12);
    display: flex;
    flex-direction: column;
    z-index: 9999;
    animation: slideUp 0.3s ease;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
      opacity: 0;
    }
    to {
      transform: translateY(0);
      opacity: 1;
    }
  }

  .chatbot-header {
    background: #2563eb;
    color: white;
    padding: 16px 20px;
    border-radius: 16px 16px 0 0;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .chatbot-header h3 {
    margin: 0;
    font-size: 18px;
  }

  .chatbot-header span {
    font-size: 12px;
    opacity: 0.9;
  }

  .chatbot-header button {
    background: none;
    border: none;
    color: white;
    font-size: 32px;
    cursor: pointer;
    line-height: 1;
  }

  .chatbot-messages {
    flex: 1;
    overflow-y: auto;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  .message {
    max-width: 80%;
    padding: 12px 16px;
    border-radius: 18px;
    word-wrap: break-word;
  }

  .message.user {
    background: #2563eb;
    color: white;
    align-self: flex-end;
    border-radius: 18px 18px 4px 18px;
  }

  .message.assistant {
    background: #f8fafc;
    color: #1e293b;
    border: 1px solid #e2e8f0;
    align-self: flex-start;
    border-radius: 18px 18px 18px 4px;
  }

  .sources {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 12px;
  }

  .source-tag {
    background: white;
    border: 1px solid #e2e8f0;
    padding: 4px 12px;
    border-radius: 6px;
    font-size: 12px;
    color: #2563eb;
  }

  .typing-indicator {
    display: flex;
    gap: 4px;
    padding: 12px 16px;
    background: #f8fafc;
    border-radius: 18px;
    width: fit-content;
  }

  .typing-indicator span {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #64748b;
    animation: bounce 1.4s infinite;
  }

  .typing-indicator span:nth-child(2) {
    animation-delay: 0.2s;
  }

  .typing-indicator span:nth-child(3) {
    animation-delay: 0.4s;
  }

  @keyframes bounce {
    0%, 60%, 100% {
      transform: translateY(0);
    }
    30% {
      transform: translateY(-10px);
    }
  }

  .chatbot-input {
    display: flex;
    gap: 12px;
    padding: 16px;
    border-top: 1px solid #e2e8f0;
  }

  .chatbot-input input {
    flex: 1;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    padding: 12px 16px;
    font-size: 14px;
  }

  .chatbot-input button {
    width: 44px;
    height: 44px;
    border-radius: 12px;
    background: #2563eb;
    color: white;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  .chatbot-input button:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .selected-text-bar {
    background: #dbeafe;
    padding: 12px 16px;
    border-top: 1px solid #e2e8f0;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
  }

  .selected-text-bar button {
    background: none;
    border: none;
    font-size: 20px;
    cursor: pointer;
  }

  @media (max-width: 768px) {
    .chatbot-panel {
      width: 100vw;
      height: 100vh;
      bottom: 0;
      right: 0;
      border-radius: 0;
    }
  }
  #### 13. File: src/theme/Root.tsx (Modify existing)
  typescript
  import React from 'react';
  import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

  export default function Root({children}) {
    return (
      <>
        {children}
        <ChatbotWidget />
      </>
    );
  }
  ---
## 🚀 STEP-BY-STEP EXECUTION PLAN ### STEP 1: Delete Old Backend
  bash
  # Remove old broken backend
  rm -rf backend/
  rm -rf rag_agent/
  rm -rf vector_search/
  rm -rf database_setup/
  ### STEP 2: Create New Structure
  bash
  # Create new chatbot-backend
  mkdir chatbot-backend
  cd chatbot-backend
  python -m venv venv
  source venv/bin/activate  # Windows: venv\Scripts\activate
  pip install -r requirements.txt
  ### STEP 3: Setup Environment Create .env file with real credentials ### STEP 4: Create All Files Copy all code from specifications
  above ### STEP 5: Test Backend
  bash
  cd chatbot-backend
  python app/main.py
  # Test: http://localhost:8000/health
  ### STEP 6: Index Content
  bash
  curl -X POST http://localhost:8000/api/indexing/rebuild
  ### STEP 7: Add Frontend Create ChatbotWidget component and integrate ### STEP 8: Test Full System Test chat, selected text,
  sources ---
## just give me the further constitution based on my 2. Integrated RAG Chatbot Development: Build and embed a
  Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs,
  FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's
  content, including answering questions based only on text selected by the user. this requirments and add this constitution into my
  rag-constitution.md file which is in .specify/memory/rag-constitution.md do not change any other file be careful
labels: ["constitution", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/rag-constitution.md
tests:
  - null
---

## Prompt

```text
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

---
