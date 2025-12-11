# RAG Chatbot Specification Document
## Physical AI & Robotics Textbook - Complete Technical Specification
---
## 📋 EXECUTIVE SUMMARY
*Project Name:* Physical AI & Robotics Textbook RAG Chatbot
*Project Type:* Retrieval-Augmented Generation (RAG) Chatbot System
*Project Root:* C:\new\physical-ai-robotics-textbook\docusaurus
*Objective:* Build an intelligent chatbot embedded in Docusaurus that answers questions from the Physical AI textbook using RAG architecture
*Technology Stack:*
- *Frontend:* Docusaurus (React + TypeScript)
- *Backend API:* FastAPI (Python 3.11+)
- *Vector Database:* Qdrant Cloud (Free Tier)
- *Relational Database:* Neon Serverless Postgres
- *AI Framework:* OpenAI Agents SDK
- *Chat UI:* ChatKit Python SDK Integration
- *Embeddings:* OpenAI text-embedding-3-small (1536 dimensions)
- *LLM:* GPT-4o via OpenAI API
- *Deployment:* Backend on local/cloud, Frontend on GitHub Pages

## Clarifications
### Session 2025-12-07
- Q: When a user interacts with a source citation, what is the precise expected behavior? → A: Smoothly scroll to the relevant section anchor on the current page. If the source is on a different page, navigate to that page and then then scroll.
- Q: If the backend cannot connect to the OpenAI API for generating a response, what should the user see in the chat widget? → A: Display a clear, static message in the chat interface like "I am currently unavailable. Please try again later." with a retry button.
- Q: How should the chatbot handle a very long (e.g., exceeding 2000 characters) or clearly nonsensical user query? → A: Inform the user that the query is too long or unclear, and ask them to rephrase or shorten it.

---
## 🎯 FUNCTIONAL REQUIREMENTS
### FR-1: Document Ingestion & Vectorization
*Priority:* CRITICAL
*Description:* System must ingest all Markdown files from /docs folder and convert them into vector embeddings
*Requirements:*
- FR-1.1: Parse all .md and .mdx files from docs/ directory recursively
- FR-1.2: Extract metadata (module name, chapter name, file path) from each document
- FR-1.3: Split documents into chunks of 1000 characters with 200 character overlap
- FR-1.4: Generate embeddings using OpenAI text-embedding-3-small model
- FR-1.5: Store embeddings in Qdrant Cloud collection named physical_ai_textbook
- FR-1.6: Store document metadata alongside vectors (module, chapter, source_file, chunk_index)
- FR-1.7: Support force refresh to re-ingest all documents
- FR-1.8: Handle errors gracefully (file read errors, API failures)
- FR-1.9: Log ingestion statistics (total files, chunks created, time taken)
*Acceptance Criteria:*
- All documents from /docs folder successfully ingested
- Each chunk has corresponding vector in Qdrant
- Metadata accurately reflects document structure
- Ingestion script can be re-run without duplicates

### FR-2: Vector Search & Retrieval
*Priority:* CRITICAL
*Description:* System must retrieve most relevant document chunks based on user queries
*Requirements:*
- FR-2.1: Convert user query to embedding using same model as documents
- FR-2.2: Perform cosine similarity search in Qdrant
- FR-2.3: Return top 5 most relevant chunks by default (configurable)
- FR-2.4: Include relevance score (0-1) for each retrieved chunk
- FR-2.5: If user selects text, combine selected text with query for better retrieval
- FR-2.6: Support filtering by module or chapter (optional filters)
- FR-2.7: Handle edge cases (empty results, query too short)
- FR-2.8: Return results in under 500ms (target performance)
*Acceptance Criteria:*
- Query "What is ROS 2?" returns relevant chunks about ROS 2
- Selected text influences search results appropriately
- Relevance scores are meaningful and accurate
- No irrelevant results in top 5

### FR-3: Conversation Management
*Priority:* HIGH
*Description:* System must maintain conversation history for contextual responses
*Requirements:*
- FR-3.1: Generate unique conversation ID for each new chat session
- FR-3.2: Store conversation metadata in Neon Postgres conversations table
- FR-3.3: Store each message (user + assistant) in messages table
- FR-3.4: Link messages to conversation via foreign key
- FR-3.5: Include timestamp for each message
- FR-3.6: Store source citations with assistant messages as JSONB
- FR-3.7: Retrieve last 10 messages for conversation context
- FR-3.8: Support multiple concurrent conversations per user
- FR-3.9: Update conversation timestamp on each new message
*Database Schema:*
```sql
-- conversations table
CREATE TABLE conversations (
    id VARCHAR(255) PRIMARY KEY,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- messages table
CREATE TABLE messages (
    id SERIAL PRIMARY KEY,
    conversation_id VARCHAR(255) REFERENCES conversations(id),
    role VARCHAR(50) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    sources JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for performance
CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
```
*Acceptance Criteria:*
- Each conversation has unique ID
- All messages persisted correctly
- Conversation history retrieved in chronological order
- No data loss on server restart

### FR-4: AI Agent Response Generation
*Priority:* CRITICAL
*Description:* System must generate accurate, contextual responses using OpenAI Agents SDK
*Requirements:*
- FR-4.1: Use OpenAI Agents SDK (not direct API calls)
- FR-4.2: Implement RAG pattern: Retrieve → Augment → Generate
- FR-4.3: System prompt defines agent as textbook expert assistant
- FR-4.4: Include retrieved context in system prompt
- FR-4.5: Include last 6 messages from conversation history for context
- FR-4.6: If user selected text, highlight it in the prompt
- FR-4.7: Generate response using GPT-4o model
- FR-4.8: Set temperature to 0.7 for balanced creativity/accuracy
- FR-4.9: Limit response to 1000 tokens maximum
- FR-4.10: Agent must cite sources (module/chapter) in responses
- FR-4.11: Agent must say "I don't know" if answer not in context
- FR-4.12: Handle API errors with retry logic (3 attempts)
- FR-4.13: If a query is very long (>2000 chars) or nonsensical, inform the user it's too long/unclear and ask them to rephrase/shorten.
*System Prompt Template:*
```
You are an expert AI assistant for the "Physical AI & Humanoid Robotics" textbook. Your role:
- Answer questions about Physical AI, ROS 2, Gazebo, Unity, NVIDIA Isaac, and humanoid robotics
- Use the provided context from the textbook to give accurate, detailed answers
- If the user has selected text, pay special attention to that selection
- Cite specific modules/chapters when relevant
- Be technical but clear in your explanations
- If information is not in the context, say so clearly

Context from the textbook: {context}
{selected_text_if_any}

Always be helpful, accurate, and educational.
```
*Acceptance Criteria:*
- Responses are accurate based on textbook content
- Sources are properly cited
- Agent admits when information is not available
- Responses are contextually relevant to conversation

### FR-5: Selected Text Query Support
*Priority:* HIGH
*Description:* Users can highlight text in the book and ask questions about it
*Requirements:*
- FR-5.1: Frontend must capture user text selection
- FR-5.2: Send selected text along with query in API request
- FR-5.3: Backend combines selected text with query for embedding
- FR-5.4: Agent gives higher priority to selected text in response
- FR-5.5: Selected text included in system prompt separately
- FR-5.6: If no text selected, system works normally
- FR-5.7: Selected text limited to 2000 characters maximum
*API Request Format:*
```json
{
    "message": "What does this mean?",
    "conversation_id": "uuid-here",
    "selected_text": "ROS 2 is a middleware...",
    "page_url": "/docs/module-1/ros2-intro"
}
```
*Acceptance Criteria:*
- User can select text and get relevant answers
- Agent references the selected text in response
- Works seamlessly without selected text too

### FR-6: REST API Endpoints
*Priority:* CRITICAL
*Description:* FastAPI backend must expose well-defined REST endpoints
*Required Endpoints:*
#### 6.1: POST /api/chat
*Purpose:* Handle chat messages and return AI responses
*Request Body:*
```json
{
    "message": "string (required)",
    "conversation_id": "string (optional)",
    "selected_text": "string (optional)",
    "page_url": "string (optional)"
}
```
*Response Body:*
```json
{
    "answer": "string",
    "sources": [
        {
            "content": "string",
            "metadata": {
                "module": "string",
                "chapter": "string",
                "source_file": "string"
            },
            "score": "float"
        }
    ],
    "conversation_id": "string",
    "timestamp": "ISO8601 datetime"
}
```
*Status Codes:*
- 200: Success
- 400: Bad request (missing message)
- 500: Internal server error

#### 6.2: GET /api/conversations/{conversation_id}
*Purpose:* Retrieve conversation history
*Response Body:*
```json
{
    "conversation_id": "string",
    "messages": [
        {
            "role": "string",
            "content": "string",
            "sources": "array or null",
            "created_at": "ISO8601 datetime"
        }
    ]
}
```
*Status Codes:*
- 200: Success
- 404: Conversation not found
- 500: Internal server error

#### 6.3: POST /api/ingest
*Purpose:* Trigger document ingestion/re-ingestion
*Request Body:*
```json
{
    "force_refresh": "boolean (default: false)"
}
```
*Response Body:*
```json
{
    "status": "success",
    "documents_processed": "integer",
    "chunks_created": "integer",
    "message": "string"
}
```
*Status Codes:*
- 200: Success
- 500: Ingestion failed

#### 6.4: GET /api/health
*Purpose:* Check system health and service connectivity
*Response Body:*
```json
{
    "status": "healthy",
    "qdrant_connected": "boolean",
    "postgres_connected": "boolean",
    "openai_configured": "boolean"
}
```
*Status Codes:*
- 200: All services healthy
- 503: One or more services unavailable

### FR-7: Frontend Integration (Docusaurus)
*Priority:* HIGH
*Description:* Chat widget must be embedded in Docusaurus site
*Requirements:*
- FR-7.1: Create React component ChatWidget in src/components/
- FR-7.2: Use ChatKit Python SDK for UI (via React wrapper)
- FR-7.3: Display floating chat button on all pages
- FR-7.4: Open chat modal on button click
- FR-7.5: Support text selection → Ask Question workflow
- FR-7.6: Show typing indicator while waiting for response
- FR-7.7: Display sources below each assistant message
- FR-7.8: Clicking a source citation scrolls the main view to the relevant section of the textbook. If the source is on a different page, navigate to that page and then scroll.
- FR-7.9: Maintain conversation state across page navigation
- FR-7.10: Store conversation_id in sessionStorage
- FR-7.11: Style consistent with Docusaurus theme
- FR-7.12: Responsive design (mobile + desktop)
*ChatWidget Features:*
- Collapsible chat window
- Message history scrollable
- User messages right-aligned (blue)
- AI messages left-aligned (gray)
- Source citations clickable to smoothly scroll to the relevant section anchor on the current page or navigate to a new page and then scroll.
- Clear conversation button
- Error state display
*Acceptance Criteria:*
- Chat widget visible on all pages
- User can chat without leaving the page
- Sources are clickable and accurate
- Mobile experience is smooth

### FR-8: Error Handling & Logging
*Priority:* MEDIUM
*Description:* Comprehensive error handling across all components
*Requirements:*
- FR-8.1: All API endpoints have try-catch blocks
- FR-8.2: Return meaningful error messages to frontend
- FR-8.3: Log all errors with timestamp, endpoint, and stack trace
- FR-8.4: Log all API requests (query, conversation_id, timestamp)
- FR-8.5: Log performance metrics (embedding time, search time, LLM time)
- FR-8.6: Use Python logging module with INFO level minimum
- FR-8.7: Rotate logs daily (max 7 days retention)
- FR-8.8: Never log sensitive data (API keys, user PII)
- FR-8.9: Frontend shows user-friendly error messages and for critical service outages, displays "I am currently unavailable. Please try again later." with a retry button.
- FR-8.10: Implement retry logic for transient failures
*Log Format:*
```
2024-12-07 14:30:45 - app.routes.chat - INFO - Chat request received: conv_id=abc123, query_length=45
2024-12-07 14:30:46 - app.services.qdrant - INFO - Search completed: 5 results in 234ms
2024-12-07 14:30:47 - app.services.agent - ERROR - OpenAI API error: Rate limit exceeded
```
*Acceptance Criteria:*
- All errors logged with context
- User never sees raw error messages
- Performance bottlenecks identifiable from logs

## 🏗 NON-FUNCTIONAL REQUIREMENTS
### NFR-1: Performance
- NFR-1.1: API response time < 3 seconds for 95% of requests
- NFR-1.2: Vector search completes in < 500ms
- NFR-1.3: Document ingestion processes 100 chunks/minute minimum
- NFR-1.4: Frontend chat widget loads in < 1 second
- NFR-1.5: Support 10 concurrent users without degradation

### NFR-2: Scalability
- NFR-2.1: Qdrant collection can handle 10,000+ vectors
- NFR-2.2: Postgres can store 100,000+ messages
- NFR-2.3: Architecture supports horizontal scaling of API servers
- NFR-2.4: Stateless API design (no in-memory session storage)

### NFR-3: Security
- NFR-3.1: API keys stored in environment variables only
- NFR-3.2: CORS properly configured (whitelist origins)
- NFR-3.3: No sensitive data in logs or error messages
- NFR-3.4: Database connections use SSL/TLS
- NFR-3.5: Input validation on all API endpoints
- NFR-3.6: SQL injection prevention (parameterized queries)

### NFR-4: Reliability
- NFR-4.1: 99% uptime for API services
- NFR-4.2: Graceful degradation if vector search fails
- NFR-4.3: Database transactions are ACID compliant
- NFR-4.4: Automatic reconnection to databases on connection loss
- NFR-4.5: No data loss during server restarts

### NFR-5: Maintainability
- NFR-5.1: Code follows PEP 8 style guide (Python)
- NFR-5.2: All functions have type hints
- NFR-5.3: All modules have docstrings
- NFR-5.4: Modular architecture (services, routes, models separated)
- NFR-5.5: Configuration externalized to .env file
- NFR-5.6: No hardcoded values in code

### NFR-6: Usability
- NFR-6.1: Chat interface intuitive (no learning curve)
- NFR-6.2: Responses are human-readable and well-formatted
- NFR-6.3: Sources clearly displayed and distinguishable
- NFR-6.4: Error messages are helpful, not technical
- NFR-6.5: Mobile users can chat comfortably

## 🗂 SYSTEM ARCHITECTURE
### High-Level Architecture
```mermaid
 graph TD
    USER[USER (Browser)] --> DOCUSAURUS[Docusaurus Frontend (React + ChatWidget Component)]
    DOCUSAURUS -- HTTP/REST (JSON) --> BACKEND[FastAPI Backend]
    BACKEND --> ROUTES[Routes (Endpoints)]
    BACKEND --> SERVICES[Services (Business)]
    BACKEND --> MODELS[Models (Schemas)]
    ROUTES --> SERVICES
    SERVICES --> MODELS
    SERVICES --> OPENAI[OpenAI API (Embeddings + GPT-4o)]
    SERVICES --> QDRANT[Qdrant Cloud (Vectors)]
    SERVICES --> POSTGRES[Neon Postgres (Conversations)]
```
### Data Flow: User Query → Response
1. User types question in ChatWidget
2. Frontend sends POST /api/chat {message, conversation_id, selected_text}
3. Backend: EmbeddingService generates query embedding
4. Backend: QdrantService searches for similar chunks
5. Backend: Retrieve top 5 relevant chunks with metadata
6. Backend: PostgresService fetches conversation history
7. Backend: AgentService builds context + history + query
8. Backend: OpenAI Agents SDK generates response
9. Backend: PostgresService saves user message + AI response
10. Backend: Returns response + sources to frontend
11. Frontend: Displays answer + clickable sources

## 📁 COMPLETE FILE STRUCTURE
```
C:\new\physical-ai-robotics-textbook\docusaurus\
│ 
├── docs/ # Your existing textbook content
│ ├── intro.md
│ ├── module-1/
│ │ ├── ros2-intro.md
│ │ └── ...
│ ├── module-2/
│ ├── module-3/
│ └── module-4/
│ 
├── backend/ # NEW - Backend application
│ ├── app/
│ │ ├── __init__.py
│ │ ├── main.py # FastAPI app entry point
│ │ ├── config.py # Settings and env vars
│ │ ├── models.py # Pydantic request/response models
│ │ 
│ │ ├── services/ # Business logic layer
│ │ │ ├── __init__.py
│ │ │ ├── embedding_service.py
│ │ │ ├── qdrant_service.py
│ │ │ ├── postgres_service.py
│ │ │ └── agent_service.py
│ │ │ 
│ │ ├── routes/ # API endpoints
│ │ │ ├── __init__.py
│ │ │ ├── chat.py
│ │ │ ├── ingest.py
│ │ │ └── health.py
│ │ │ 
│ │ ├── utils/ # Helper functions
│ │ │ ├── __init__.py
│ │ │ ├── text_processing.py
│ │ │ ├── markdown_parser.py
│ │ │ └── logger.py
│ │ │ 
│ │ └── agents/ # OpenAI Agents SDK
│ │ │ ├── __init__.py
│ │ │ ├── rag_agent.py
│ │ │ └── tools.py
│ │ 
│ ├── scripts/
│ │ ├── ingest_documents.py # Ingest docs to Qdrant
│ │ └── setup_db.py # Create Postgres tables
│ │ 
│ ├── tests/
│ │ ├── __init__.py
│ │ ├── test_api.py
│ │ └── test_services.py
│ │ 
│ ├── requirements.txt
│ ├── .env.example
│ ├── .env # gitignored
│ ├── .gitignore
│ └── README.md
│ 
├── src/ # Docusaurus source
│ ├── components/
│ │ ├── ChatWidget/ # NEW - Chat UI component
│ │ │ ├── index.tsx
│ │ │ ├── ChatWindow.tsx
│ │ │ ├── MessageList.tsx
│ │ │ ├── InputBox.tsx
│ │ │ ├── SourceCard.tsx
│ │ │ ├── styles.module.css
│ │ │ └── types.ts
│ │ │ 
│ │ └── HomepageFeatures/
│ │ 
│ ├── pages/
│ ├── css/
│ └── ...
│ 
├── static/
├── docusaurus.config.js
├── sidebars.js
├── package.json
└── ...
```

### Backend Technology Details
*Python Version:* 3.11 or higher
*Package Manager:* pip
*Virtual Environment:* venv
*Core Dependencies:*
```python
fastapi==0.115.5 # Web framework
uvicorn[standard]==0.32.1 # ASGI server
openai==1.54.5 # OpenAI SDK (includes Agents)
qdrant-client==1.12.1 # Qdrant vector database
psycopg2-binary==2.9.10 # PostgreSQL adapter
pydantic==2.10.3 # Data validation
pydantic-settings==2.6.1 # Settings management
python-dotenv==1.0.1 # Environment variables
python-multipart==0.0.17 # File upload support
```
### Frontend Technology Details
*Framework:* Docusaurus 3.x
*Language:* TypeScript
*Chat UI:* Custom React components (ChatKit-style)
*Additional Dependencies:*
```json
{
    "axios": "^1.6.0",
    "react-markdown": "^9.0.0",
    "@heroicons/react": "^2.0.0"
}
```
### Database Schemas
*Qdrant Collection Schema:*
```python
Collection Name: "physical_ai_textbook"
Vector Size: 1536 (OpenAI text-embedding-3-small)
Distance Metric: Cosine
Payload Structure:
{
    "content": str, # Chunk text content
    "module": str, # e.g., "Module 1"
    "chapter": str, # e.g., "ROS 2 Fundamentals"
    "source_file": str, # e.g., "docs/module-1/ros2.md"
    "chunk_index": int, # Position in original document
    "total_chunks": int, # Total chunks in document
    "word_count": int, # Words in this chunk
    "created_at": str # ISO timestamp
}
```
*Postgres Tables:*
```sql
-- Table 1: conversations
CREATE TABLE conversations (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255), -- Future: for auth
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB -- Future: user preferences
);

-- Table 2: messages
CREATE TABLE messages (
    id SERIAL PRIMARY KEY,
    conversation_id VARCHAR(255) NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(50) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    sources JSONB, -- Array of source objects
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    token_count INT, -- For future analytics
    latency_ms INT -- Response time tracking
);

-- Indexes
CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
CREATE INDEX idx_conversations_updated ON conversations(updated_at DESC);
```
### API Configuration
*Base URL:* http://localhost:8000 (development)
*Production URL:* TBD (e.g., Railway, Render, AWS)
*CORS Settings:*
```python
allow_origins = [
    "http://localhost:3000", # Docusaurus dev
    "https://yourusername.github.io"
    # GitHub Pages
]
allow_methods = ["GET", "POST", "PUT", "DELETE", "OPTIONS"]
allow_headers = ["*"]
allow_credentials = True
```
*Rate Limiting:*
- 100 requests per minute per IP (future enhancement)
- 10 requests per second per conversation

## 🔑 ENVIRONMENT VARIABLES
### Required Variables
```env
# ============================================ # OpenAI Configuration # ============================================ 
OPENAI_API_KEY=sk-proj-...
# Required: OpenAI API key
OPENAI_ORG_ID=org-...
# Optional: Organization ID

# ============================================ # Qdrant Configuration # ============================================ 
QDRANT_URL=https://xyz.qdrant.io:6333
# Required: Qdrant Cloud URL
QDRANT_API_KEY=...
# Required: Qdrant API key
QDRANT_COLLECTION_NAME=physical_ai_textbook
# Collection name

# ============================================ # Neon Postgres Configuration # ============================================ 
POSTGRES_URL=postgresql://user:pass@host/db?sslmode=require
# Format: postgresql://[user]:[password]@[host]/[database]?sslmode=require

# ============================================ # Model Configuration # ============================================ 
EMBEDDING_MODEL=text-embedding-3-small
# OpenAI embedding model
EMBEDDING_DIMENSION=1536
# Embedding vector size
LLM_MODEL=gpt-4o
# OpenAI LLM model
MAX_TOKENS=1000
# Max response tokens
TEMPERATURE=0.7
# LLM creativity (0-1)

# ============================================ # RAG Configuration # ============================================ 
CHUNK_SIZE=1000
# Characters per chunk
CHUNK_OVERLAP=200
# Overlap between chunks
TOP_K_RESULTS=5
# Retrieved chunks per query
MIN_RELEVANCE_SCORE=0.7
# Minimum similarity score

# ============================================ # API Configuration # ============================================ 
API_HOST=0.0.0.0
# Bind address
API_PORT=8000
# Port number
API_WORKERS=4
# Uvicorn workers
CORS_ORIGINS=["http://localhost:3000"]
# Allowed origins (JSON array)

# ============================================ # File Paths # ============================================ 
DOCS_PATH=../docs
# Path to Docusaurus docs
LOG_PATH=./logs
# Path for log files

# ============================================ # Feature Flags (Optional) # ============================================ 
ENABLE_CACHING=true
# Cache embeddings
ENABLE_ANALYTICS=true
# Track usage metrics
DEBUG_MODE=false
# Verbose logging
```
### How to Obtain Each Credential
*OpenAI API Key:*
1. Go to https://platform.openai.com/api-keys
2. Sign in or create account
3. Click "Create new secret key"
4. Copy and save the key (starts with sk-)
*Qdrant Cloud:*
1. Go to https://cloud.qdrant.io/
2. Sign up for free tier
3. Create a cluster
4. Copy cluster URL (e.g., https://abc-def.qdrant.io)
5. Generate API key in dashboard
*Neon Postgres:*
1. Go to https://neon.tech/
2. Sign up for free tier
3. Create a project
4. Create a database
5. Copy connection string from dashboard
6. Format: postgresql://user:password@host/dbname?sslmode=require

## 📊 DATA FLOW DIAGRAMS
### Document Ingestion Flow
```mermaid
 graph TD
    A[Markdown Files (docs/*.md)] --> B(Parse & Extract - Content - Metadata)
    B --> C(Text Chunking (1000 chars) (200 overlap))
    C --> D(Generate Embeddings (OpenAI API))
    D --> E(Store in Qdrant - Vector - Payload)
```
### Chat Request Flow
```mermaid
 graph TD
    A[User sends message + conversation_id + selected_text] --> B(Generate query embedding)
    B --> C(Search Qdrant (Top 5 chunks))
    C --> D(Fetch conversation history (Postgres))
    D --> E(Build context: - System prompt - Retrieved chunks - Chat history - Selected text)
    E --> F(OpenAI Agents SDK generates response)
    F --> G(Save to Postgres: - User message - AI response - Sources)
    G --> H(Return response + sources to frontend)
    H --> I(Frontend: Displays answer + clickable sources)
```

## ✅ ACCEPTANCE CRITERIA
### System-Level Acceptance
- [ ] All documents from /docs successfully ingested to Qdrant
- [ ] User can ask any question about Physical AI topics
- [ ] Responses are accurate and cite correct sources
- [ ] Conversation history persists across sessions
- [ ] Chat widget loads on all Docusaurus pages
- [ ] Selected text query feature works correctly
- [ ] API responds within 3 seconds for 95% of requests
- [ ] No crashes or unhandled exceptions in normal operation
- [ ] All environment variables properly configured
- [ ] Backend API passes all health checks
- [ ] Frontend communicates with backend successfully
- [ ] Sources are clickable and navigate to correct pages
- [ ] Mobile experience is fully functional

### Feature-Level Acceptance
*Document Ingestion:*
- [ ] Script processes all .md and .mdx files
- [ ] Each file split into appropriate chunks
- [ ] All chunks have embeddings in Qdrant
- [ ] Metadata (module, chapter) correctly extracted
- [ ] Re-running ingestion updates existing data
*Vector Search:*
- [ ] Query "What is ROS 2?" returns ROS 2 documentation
- [ ] Query "Explain NVIDIA Isaac" returns Isaac-related content
- [ ] Top 5 results are relevant and scored correctly
- [ ] Selected text improves search relevance
- [ ] Empty queries handled gracefully
*Conversation Management:*
- [ ] New conversation generates unique ID
- [ ] All messages saved to Postgres
- [ ] History retrieved in correct order
- [ ] Timestamps accurate
- [ ] Multiple conversations don't interfere
*AI Responses:*
- [ ] Answers are factually correct based on textbook
- [ ] Sources cited in responses (module/chapter mentioned)
- [ ] Agent admits when answer not in context
- [ ] Responses are clear and well-formatted
- [ ] Code examples (if any) are properly formatted
*Chat Widget:*
- [ ] Widget visible on all pages
- [ ] Opens/closes smoothly
- [ ] Messages display correctly
- [ ] Sources expandable and readable
- [ ] Responsive on mobile devices
- [ ] Conversation persists during navigation

## 🧪 TESTING REQUIREMENTS
### Unit Tests
*Backend Services (Minimum 80% Coverage):*
- test_embedding_service.py
  - Test embedding generation for single text
  - Test batch embedding generation
  - Test error handling for API failures
- test_qdrant_service.py
  - Test collection creation
  - Test document upsert
  - Test vector search
  - Test filtering by metadata
- test_postgres_service.py
  - Test table creation
  - Test conversation CRUD operations
  - Test message CRUD operations
  - Test conversation history retrieval
- test_agent_service.py
  - Test context retrieval
  - Test response generation
  - Test handling of selected text
  - Test conversation history integration

### Integration Tests
- test_api.py
  - Test POST /api/chat endpoint
  - Test GET /api/conversations/{id}
  - Test POST /api/ingest
  - Test GET /api/health
  - Test CORS headers
  - Test error responses

### End-to-End Tests
- User asks question → receives answer with sources
- User selects text → asks question → gets relevant answer
- User continues conversation → context maintained
- User refreshes page → conversation persists
- Multiple users chat simultaneously → no interference

### Performance Tests
- 10 concurrent users sending queries
- Response time under load (target: <5s at 10 concurrent)
- Vector search performance (target: <500ms)
- Database query performance (target: <100ms)

### Manual Testing Checklist
- [ ] Chat on desktop Chrome
- [ ] Chat on mobile Safari
- [ ] Select text and ask question
- [ ] Ask follow-up questions
- [ ] Test with very long queries (>500 chars)
- [ ] Test with non-English text (Urdu queries - future)
- [ ] Test with special characters
- [ ] Test error states (backend down, API key invalid)
- [ ] Test loading states
- [ ] Test empty states (no conversation history)

## 🚀 DEPLOYMENT REQUIREMENTS
### Backend Deployment Options
*Option 1: Railway.app (Recommended)*
- Free tier: $5 credit/month
- Built-in Postgres (can use instead of Neon)
- Automatic deployments from GitHub
- Environment variables management
- HTTPS included
*Option 2: Render.com*
- Free tier available
- Automatic HTTPS
- Environment variables support
- Health check endpoints
*Option 3: AWS (Advanced)*
- EC2 instance (t2.micro free tier)
- RDS for Postgres
- Requires more configuration
*Deployment Steps (Railway):*
1. Create Procfile: `web: uvicorn app.main:app --host 0.0.0.0 --port $PORT`
2. Create railway.json with build configuration
3. Connect GitHub repository
4. Set environment variables in Railway dashboard
5. Deploy

### Frontend Deployment (GitHub Pages)
*Build Configuration:*
```bash
npm run build # Output: build/ directory
```
*Deploy:*
```bash
npm run deploy # Uses gh-pages branch
```
*Custom Domain (Optional):*
- Configure CNAME in repository settings
- Update docusaurus.config.js with custom URL

### Environment-Specific Configurations
*Development:*
- Backend: http://localhost:8000
- Frontend: http://localhost:3000
- Debug logging enabled
- CORS allows localhost
*Production:*
- Backend: https://your-backend.railway.app
- Frontend: https://username.github.io/repo-name
- Info-level logging
- CORS restricted to production domain

## 📚 DOCUMENTATION REQUIREMENTS
### Code Documentation
*Python Modules:*
- All functions must have docstrings (Google style)
- Type hints for all parameters and return values
- Module-level docstrings explaining purpose
*Example:*
```python
def generate_embedding(self, text: str) -> List[float]:
    """Generate vector embedding for given text.

    Args:
        text: Input text to embed (max 8000 tokens)

    Returns:
        List of 1536 floats representing the embedding

    Raises:
        ValueError: If text is empty
        APIError: If OpenAI API fails
    """
    # ... implementation ...
```
*React Components:*
- JSDoc comments for all components
- Props interface documented
- Usage examples in comments

### API Documentation
- OpenAPI/Swagger docs auto-generated by FastAPI
- Available at /docs endpoint
- Each endpoint has description, parameters, responses
- Include example requests/responses

### User Documentation
*README.md files:*
- backend/README.md: Setup, configuration, running locally
- docs/chatbot-guide.md: How to use the chat feature
- docs/developer-guide.md: For future contributors
*Setup Guide (for developers):*
1. Prerequisites
2. Installation steps
3. Configuration (environment variables)
4. Running locally
5. Running tests
6. Deployment

## 🔄 MAINTENANCE & MONITORING
### Logging Strategy
*Log Levels:*
- DEBUG: Detailed diagnostic info (development only)
- INFO: General informational messages
- WARNING: Warning messages (deprecated features, etc.)
- ERROR: Error messages (handled exceptions)
- CRITICAL: Critical issues (service failures)
*What to Log:*
- All API requests (timestamp, endpoint, params)
- Search queries and results count
- LLM API calls and response times
- Database operations
- Errors with full stack traces
- Performance metrics
*Log Rotation:*
- Daily rotation
- Keep last 7 days
- Compress old logs

### Monitoring Metrics
*Key Metrics to Track:*
- Total API requests per hour
- Average response time
- Error rate (%)
- Qdrant search latency
- Postgres query latency
- OpenAI API latency
- Token usage (for cost tracking)
- Active conversations count
- User queries per conversation
*Alerting Rules:*
- Response time > 5s for 5 consecutive requests
- Error rate > 5% in last hour
- Database connection failures
- OpenAI API rate limit hit

### Backup Strategy
*Postgres Backups:*
- Neon provides automatic backups
- Export conversations weekly to JSON
- Store in separate location
*Qdrant Backups:*
- Snapshot collection monthly
- Keep ingestion script for re-creation
- Original markdown files are source of truth

## 🎓 SUCCESS METRICS
### Technical Metrics
- *Ingestion Success Rate:* 100% of documents processed
- *Search Accuracy:* >80% of queries return relevant results
- *Response Time P95:* <3 seconds
- *API Uptime:* >99%
- *Error Rate:* <1%

### User Experience Metrics
- *User Satisfaction:* Qualitative feedback from users
- *Query Relevance:* Sources cited are actually relevant
- *Conversation Length:* Average 5+ messages (indicates engagement)
- *Return Rate:* Users come back to use chatbot again

### Business Metrics (Hackathon Judging)
- *Functionality:* All required features working (100 points)
- *Code Quality:* Clean, documented, maintainable code
- *User Experience:* Intuitive, fast, helpful chatbot
- *Innovation:* Any extra features beyond requirements

## 🛠 TROUBLESHOOTING GUIDE
### Common Issues & Solutions
*Issue: "Qdrant connection failed"*
- Check QDRANT_URL and QDRANT_API_KEY in .env
- Verify Qdrant cluster is running
- Check firewall/network settings
*Issue: "OpenAI API rate limit exceeded"*
- Implement request queuing
- Add exponential backoff retry logic
- Consider caching frequent queries
*Issue: "Postgres connection timeout"*
- Check POSTGRES_URL format
- Verify Neon database is active (may sleep on free tier)
- Check SSL mode is set to 'require'
*Issue: "No relevant results found"*
- Verify documents are ingested (check Qdrant dashboard)
- Lower MIN_RELEVANCE_SCORE threshold
- Check embedding model consistency
*Issue: "CORS error in frontend"*
- Add frontend URL to CORS_ORIGINS
- Check backend is running and accessible
- Verify API endpoint URLs are correct
*Issue: "Chat widget not loading"*
- Check browser console for errors
- Verify API_URL in frontend config
- Check network tab for failed requests

## 📋 IMPLEMENTATION CHECKLIST
### Pre-Development
- [ ] Read entire specification document
- [ ] Understand RAG architecture
- [ ] Set up development environment (Python 3.11+)
- [ ] Create project folder structure
- [ ] Initialize Git repository

### Backend Setup (Week 1)
- [ ] Create virtual environment
- [ ] Install dependencies from requirements.txt
- [ ] Create all Python module files
- [ ] Set up .env file with credentials
- [ ] Create Qdrant collection
- [ ] Create Postgres tables
- [ ] Test database connections

### Core Services (Week 1-2)
- [ ] Implement EmbeddingService
- [ ] Implement QdrantService
- [ ] Implement PostgresService
- [ ] Implement AgentService
- [ ] Write unit tests for each service
- [ ] Test services individually

### API Development (Week 2)
- [ ] Create FastAPI app structure
- [ ] Implement health endpoint
- [ ] Implement chat endpoint
- [ ] Implement ingest endpoint
- [ ] Configure CORS
- [ ] Test all endpoints with Postman/curl

### Document Ingestion (Week 2)
- [ ] Create markdown parser utility
- [ ] Create text chunking utility
- [ ] Create ingestion script
- [ ] Test ingestion with sample documents
- [ ] Ingest all textbook documents
- [ ] Verify in Qdrant dashboard

### Frontend Integration (Week 3)
- [ ] Create ChatWidget React component
- [ ] Implement message display
- [ ] Implement input box
- [ ] Implement source display
- [ ] Add to Docusaurus layout
- [ ] Test on all pages
- [ ] Make responsive for mobile

### Testing (Week 3)
- [ ] Write unit tests (80% coverage)
- [ ] Write integration tests
- [ ] Perform manual testing
- [ ] Test on different devices/browsers
- [ ] Fix bugs found during testing

### Deployment (Week 3-4)
- [ ] Deploy backend to Railway/Render
- [ ] Configure production environment variables
- [ ] Deploy frontend to GitHub Pages
- [ ] Update CORS for production
- [ ] Test production deployment
- [ ] Monitor logs for errors

### Documentation (Week 4)
- [ ] Write README files
- [ ] Document API endpoints
- [ ] Create user guide
- [ ] Add inline code comments
- [ ] Create demo video (<90 seconds)

### Final Review (Week 4)
- [ ] All features working as specified
- [ ] No console errors
- [ ] Performance acceptable
- [ ] Code is clean and documented
- [ ] Ready for submission

## 📞 SUPPORT & RESOURCES
### Official Documentation
- FastAPI: https://fastapi.tiangolo.com/
- OpenAI API: https://platform.openai.com/docs
- Qdrant: https://qdrant.tech/documentation/
- Neon Postgres: https://neon.tech/docs
- Docusaurus: https://docusaurus.io/docs

### Learning Resources
- RAG Tutorial: https://www.pinecone.io/learn/retrieval-augmented-generation/
- OpenAI Embeddings Guide: https://platform.openai.com/docs/guides/embeddings
- Vector Database Concepts: https://www.qdrant.tech/articles/what-is-a-vector-database/

### Community Support
- FastAPI Discord
- OpenAI Community Forum
- Qdrant Discord
- Stack Overflow

## 🎯 SPECIFICATION SUMMARY
This specification defines a complete RAG-powered chatbot system for the Physical AI & Robotics textbook. The system:
1. *Ingests* markdown documents into vector database
2. *Retrieves* relevant content based on user queries
3. *Generates* accurate responses using OpenAI Agents SDK
4. *Maintains* conversation history in Postgres
5. *Provides* intuitive chat interface in Docusaurus
6. *Supports* text selection for contextual queries
7. *Cites* sources for all answers
8. *Performs* at < 3 seconds response time
9. *Scales* to handle multiple concurrent users
10. *Deploys* to production with monitoring

*Total Estimated Development Time:* 3-4 weeks
*Complexity Level:* Intermediate to Advanced
*Team Size:* 1-2 developers

---
*Specification Version:* 1.0
*Last Updated:* December 7, 2024
*Document Owner:* Physical AI Textbook Team

---
## END OF SPECIFICATION
