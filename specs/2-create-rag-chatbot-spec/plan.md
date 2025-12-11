# Implementation Plan: RAG Chatbot

**Branch**: `2-create-rag-chatbot-spec` | **Date**: 2025-12-07 | **Spec**: [C:\new\specs\2-create-rag-chatbot-spec\spec.md](C:\new\specs\2-create-rag-chatbot-spec\spec.md)
**Input**: Feature specification from `C:\new\specs\2-create-rag-chatbot-spec\spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

---

## Summary

This project will build an intelligent chatbot embedded in a Docusaurus website for the "Physical AI & Robotics Textbook". The chatbot will use a Retrieval-Augmented Generation (RAG) architecture to answer questions based on the textbook's content. The system will feature a FastAPI backend, a Qdrant vector database, a Neon serverless Postgres database for conversational history, and a React-based frontend integrated with Docusaurus. Users can ask questions about the book content, and also select text from any page to ask context-specific questions via an interactive popup.

---

## Technical Context

**Language/Version**: Python 3.11+ (Backend), TypeScript/JavaScript (Frontend)
**Primary Dependencies**: 
- Backend: FastAPI, OpenAI SDK (Agents + Embeddings), Qdrant Client, psycopg2, Pydantic
- Frontend: Docusaurus 3.x, React 18, TypeScript, Axios
**Storage**: 
- Qdrant Cloud Free Tier (Vector Database)
- Neon Serverless Postgres Free Tier (Relational Database)
**AI Services**:
- OpenAI API (text-embedding-3-small for embeddings)
- OpenAI Agents SDK (GPT-4o for response generation)
**Testing**: pytest (Backend), Jest + React Testing Library (Frontend)
**Target Platform**: Web (Desktop + Mobile)
**Project Type**: Full-stack web application with RAG architecture
**Performance Goals**: 
- API response time < 3 seconds for 95% of requests
- Vector search < 500ms
- Document ingestion: 100 chunks/minute minimum
- Chat widget load time < 1 second
**Constraints**: 
- Use free tiers for Qdrant Cloud and Neon Postgres
- Must use OpenAI (not Gemini) as per hackathon requirements
- No browser localStorage/sessionStorage in artifacts
- Backend deployed separately from frontend
**Scale/Scope**: 
- Support 10 concurrent users
- Qdrant collection: 10,000+ vectors
- Postgres: 100,000+ messages
- Textbook size: ~50-100 markdown files

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Reproducible and versioned**: ✅ Yes, all code will be in Git repository with proper version control. Backend and frontend have separate package management (requirements.txt, package.json).

- **Privacy-aware**: ✅ Yes, the system implements privacy controls:
  - User queries are not shared between different conversation IDs
  - Text selections are sent to backend only when user explicitly asks a question
  - No persistent tracking of user behavior
  - Conversation data stored with unique IDs, not linked to personal identity

- **Minimal external cost**: ✅ Yes, using free tiers:
  - Qdrant Cloud: Free tier (1GB storage, sufficient for textbook vectors)
  - Neon Postgres: Free tier (3GB storage, 100 hours compute/month)
  - OpenAI API: Pay-as-you-go (estimated $5-10/month for development/testing)

- **Testable**: ✅ Yes, comprehensive testing strategy:
  - Unit tests for all services (target: 80% coverage)
  - Integration tests for API endpoints
  - End-to-end tests for critical user flows
  - Manual testing checklist for UI/UX

- **Audit & guardrails**: ✅ Yes, includes:
  - Logging all API requests and responses (app.utils.logger)
  - Error handling with meaningful messages
  - Input validation (query length limits, sanitization)
  - Rate limiting consideration for production
  - Performance metrics tracking

- **Model provider**: ⚠️ **CLARIFICATION REQUIRED**
  - **Hackathon Specification**: Explicitly requires OpenAI Agents SDK, OpenAI embeddings (text-embedding-3-small), and GPT-4o
  - **Constitution Preference**: Mentions Gemini API
  - **Resolution**: This project will use **OpenAI** as mandated by the hackathon requirements. The hackathon specification takes precedence for this specific project. If constitution's Gemini requirement is mandatory, the entire architecture needs redesign (Gemini doesn't have an Agents SDK or equivalent embedding model).

---

## Model Provider Justification

| Requirement Source | Model Provider | Reasoning |
|-------------------|---------------|-----------|
| Hackathon Spec (Primary) | OpenAI | Explicitly requires "OpenAI Agents SDK", "text-embedding-3-small", and "GPT-4o". These are core technical requirements. |
| Constitution (General Guideline) | Gemini | General preference, but hackathon-specific requirements override for competition submission. |
| **Final Decision** | **OpenAI** | Using OpenAI to meet hackathon deliverables. Gemini migration can be a future enhancement after competition. |

**Note**: If Gemini is absolutely mandatory, we need to:
1. Replace OpenAI Agents SDK with custom agent logic using Gemini SDK
2. Use Gemini embeddings (different dimensions, requires re-architecture)
3. Adjust RAG pipeline for Gemini's API structure
4. Update all specifications and code accordingly

**Recommendation**: Proceed with OpenAI for hackathon submission, document Gemini as a post-hackathon enhancement.

---

## Project Structure

### Documentation (this feature)

```text
specs/2-create-rag-chatbot-spec/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
C:\new\physical-ai-robotics-textbook\docusaurus\
│
├── docs/                           # EXISTING - Your textbook content (DO NOT MODIFY)
│   ├── intro.md
│   ├── module-1/
│   │   ├── ros2-intro.md
│   │   └── ...
│   ├── module-2/
│   ├── module-3/
│   └── module-4/
│
├── backend/                        # NEW - Backend application
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py                # FastAPI app entry point
│   │   ├── config.py              # Settings and env vars
│   │   ├── models.py              # Pydantic request/response models
│   │   │
│   │   ├── services/              # Business logic layer
│   │   │   ├── __init__.py
│   │   │   ├── embedding_service.py    # OpenAI embeddings
│   │   │   ├── qdrant_service.py       # Vector DB operations
│   │   │   ├── postgres_service.py     # Conversation storage
│   │   │   └── agent_service.py        # OpenAI Agents SDK integration
│   │   │
│   │   ├── routes/                # API endpoints
│   │   │   ├── __init__.py
│   │   │   ├── chat.py            # POST /api/chat, GET /api/conversations/{id}
│   │   │   ├── ingest.py          # POST /api/ingest
│   │   │   └── health.py          # GET /api/health
│   │   │
│   │   └── utils/                 # Helper functions
│   │       ├── __init__.py
│   │       ├── text_processing.py      # Text chunking (1000 chars, 200 overlap)
│   │       ├── markdown_parser.py      # Parse Docusaurus markdown
│   │       └── logger.py               # Logging configuration
│   │
│   ├── scripts/
│   │   ├── ingest_documents.py    # Ingest docs to Qdrant
│   │   └── setup_db.py            # Create Postgres tables
│   │
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── test_api.py            # API endpoint tests
│   │   ├── test_services.py       # Service layer tests
│   │   └── test_utils.py          # Utility function tests
│   │
│   ├── requirements.txt           # Python dependencies
│   ├── .env.example               # Example environment variables
│   ├── .env                       # Actual env vars (gitignored)
│   ├── .gitignore
│   ├── pytest.ini                 # pytest configuration
│   └── README.md                  # Backend documentation
│
├── src/                           # Docusaurus source (EXISTING, MODIFY)
│   ├── components/
│   │   ├── ChatWidget/            # NEW - Chat UI component
│   │   │   ├── index.tsx          # Main chat widget component
│   │   │   ├── ChatWindow.tsx     # Chat modal window
│   │   │   ├── MessageList.tsx    # Message display component
│   │   │   ├── InputBox.tsx       # User input component
│   │   │   ├── SourceCard.tsx     # Source citation component
│   │   │   ├── TextSelectionPopup.tsx  # NEW - Popup for selected text
│   │   │   ├── ErrorBoundary.tsx  # Error handling component
│   │   │   ├── styles.module.css  # Component styles
│   │   │   ├── types.ts           # TypeScript types
│   │   │   └── api.ts             # API client functions
│   │   │
│   │   └── HomepageFeatures/      # EXISTING
│   │
│   ├── pages/                     # EXISTING
│   ├── css/                       # EXISTING
│   └── theme/                     # NEW - Custom theme overrides
│       └── Root.tsx               # Wrap app with ChatWidget
│
├── static/                        # EXISTING - Static assets
├── docusaurus.config.js           # MODIFY - Add custom scripts
├── sidebars.js                    # EXISTING
├── package.json                   # MODIFY - Add new dependencies
├── tsconfig.json                  # EXISTING/MODIFY
└── README.md                      # MODIFY - Add chatbot documentation
```

**Structure Decision**: The project follows a clean separation between backend (FastAPI) and frontend (Docusaurus + React). This allows:
- Independent development and deployment
- Clear API contracts between layers
- Easy testing of backend services
- Reusability of frontend components
- Scalability (can add more frontends or backends)

---

## Key Features to Implement

### 1. Document Ingestion & Vectorization (FR-1)
- **Purpose**: Convert all markdown textbook files into searchable vector embeddings
- **Components**:
  - `backend/utils/markdown_parser.py`: Parse .md/.mdx files, extract frontmatter
  - `backend/utils/text_processing.py`: Chunk text (1000 chars, 200 overlap)
  - `backend/services/embedding_service.py`: Generate embeddings via OpenAI
  - `backend/services/qdrant_service.py`: Store vectors with metadata
  - `backend/scripts/ingest_documents.py`: Orchestrate full ingestion
- **Metadata Captured**: module name, chapter name, file path, chunk index, word count
- **Success Criteria**: All docs from `docs/` folder successfully vectorized and stored

### 2. Vector Search & Retrieval (FR-2)
- **Purpose**: Find most relevant textbook chunks for user queries
- **Flow**:
  1. User query → Generate embedding
  2. Cosine similarity search in Qdrant
  3. Return top 5 chunks with scores
  4. If user selected text, boost relevance of similar chunks
- **Performance Target**: < 500ms search time
- **Success Criteria**: Query "What is ROS 2?" returns ROS 2 documentation chunks

### 3. Conversation Management (FR-3)
- **Purpose**: Persist chat history for contextual responses
- **Database Schema**:
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
      role VARCHAR(50) NOT NULL,
      content TEXT NOT NULL,
      sources JSONB,
      created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
  );
  
  CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
  ```
- **Success Criteria**: Conversation history persists across page navigation

### 4. AI Agent Response Generation (FR-4)
- **Purpose**: Generate accurate, contextual answers using OpenAI Agents SDK
- **RAG Pipeline**:
  1. Retrieve relevant chunks (context)
  2. Fetch conversation history (last 6 messages)
  3. Build system prompt with context
  4. Call OpenAI Agents SDK (GPT-4o)
  5. Return response with source citations
- **System Prompt Template**:
  ```
  You are an expert AI assistant for the "Physical AI & Humanoid Robotics" textbook.
  
  Your role:
  - Answer questions about Physical AI, ROS 2, Gazebo, Unity, NVIDIA Isaac, humanoid robotics
  - Use the provided context from the textbook for accurate answers
  - Cite specific modules/chapters when relevant
  - Be technical but clear
  - If information is not in context, say so clearly
  
  Context from textbook:
  {retrieved_chunks}
  
  {selected_text_if_provided}
  ```
- **Success Criteria**: Responses cite correct sources and are factually accurate

### 5. Text Selection Popup (FR-5) ⭐ NEW CRITICAL FEATURE
- **Purpose**: Allow users to highlight text on any book page and instantly ask questions about it
- **User Flow**:
  1. User highlights text on a documentation page (10-2000 characters)
  2. Floating popup appears near selection within 200ms
  3. Popup shows:
     - Text preview (first 50 chars + "...")
     - "Ask about this 💬" button
     - "Explain in detail" quick action
     - Close (×) button
  4. User clicks "Ask about this"
  5. Chat widget opens with selected text in context box
  6. User types question or uses pre-filled "Explain this in detail"
  7. Backend prioritizes selected text in vector search
  8. Agent references selected text in response

- **Frontend Components**:
  - `TextSelectionPopup.tsx`: Floating popup UI
  - Event listeners: `mouseup`, `touchend` on article content
  - Positioning logic: Calculate position relative to selection
  - Z-index: 9999 (above all other elements)

- **Visual Feedback**:
  - Selected text highlighted in yellow (rgba(255, 235, 59, 0.3))
  - Smooth fade-in/out animations (100ms)
  - Popup styled with shadow, rounded corners
  - Responsive on mobile (44px minimum touch target)

- **Chat Widget Context Box**:
  ```
  ┌─────────────────────────────────────────┐
  │ 📄 Selected from Module 1: ROS 2        │
  │ "ROS 2 is a middleware for robotics..." │
  │                                      [×] │
  └─────────────────────────────────────────┘
  ```
  - User can remove context by clicking (×)
  - Input placeholder: "Ask about the selected text..."

- **Backend Processing**:
  - Combine query with selected text for embedding: `{query} [CONTEXT: {selected_text}]`
  - Include in system prompt: `<selected_text>{text}</selected_text>`
  - Agent instruction: "Pay special attention to the selected text and reference it in your response"

- **Success Criteria**:
  - User selects "ROS 2 is a middleware" → Popup appears
  - Clicks "Ask about this" → Chat opens with context loaded
  - Asks "What does this mean?" → Agent explains ROS 2 specifically
  - Selected text is clearly highlighted
  - Mobile experience is smooth (no accidental selections)

### 6. REST API Endpoints (FR-6)

#### POST /api/chat
```json
// Request
{
  "message": "What is ROS 2?",
  "conversation_id": "uuid-optional",
  "selected_text": "ROS 2 is a middleware...",  // optional
  "page_url": "/docs/module-1/ros2"             // optional
}

// Response
{
  "answer": "ROS 2 (Robot Operating System 2) is...",
  "sources": [
    {
      "content": "ROS 2 is a middleware for robotics...",
      "metadata": {
        "module": "Module 1",
        "chapter": "ROS 2 Fundamentals",
        "source_file": "docs/module-1/ros2-intro.md"
      },
      "score": 0.89
    }
  ],
  "conversation_id": "uuid-here",
  "timestamp": "2024-12-07T14:30:45Z"
}
```

#### GET /api/conversations/{conversation_id}
```json
// Response
{
  "conversation_id": "uuid",
  "messages": [
    {
      "role": "user",
      "content": "What is ROS 2?",
      "created_at": "2024-12-07T14:30:45Z"
    },
    {
      "role": "assistant",
      "content": "ROS 2 is...",
      "sources": [...],
      "created_at": "2024-12-07T14:30:47Z"
    }
  ]
}
```

#### POST /api/ingest
```json
// Request
{
  "force_refresh": false
}

// Response
{
  "status": "success",
  "documents_processed": 47,
  "chunks_created": 523,
  "message": "Documents successfully ingested"
}
```

#### GET /api/health
```json
// Response
{
  "status": "healthy",
  "qdrant_connected": true,
  "postgres_connected": true,
  "openai_configured": true
}
```

### 7. Frontend Chat Widget (FR-7)
- **Location**: Floating button on all Docusaurus pages
- **Features**:
  - Collapsible chat modal
  - Message history (scrollable)
  - User messages: right-aligned, blue background
  - AI messages: left-aligned, gray background
  - Source citations below each AI message
  - Loading indicator during API calls
  - Error states with retry button
  - Mobile-responsive design

- **Source Citation Behavior** (Answer to Question 1):
  - **Option B Implementation**: When user clicks a source:
    1. Extract module and chapter from source metadata
    2. Navigate to page: `/docs/module-1/ros2-intro`
    3. Scroll to relevant section using anchor: `#section-name`
    4. Smooth scroll behavior
    5. Temporary highlight of target section (yellow glow for 2 seconds)
  - **Benefits**:
    - User stays in documentation flow
    - No tab clutter
    - Works on mobile
    - Maintains chat context (can press back to return)

- **Component Tree**:
  ```
  <ChatWidget>
    ├── <ChatButton /> (floating circular button)
    └── <ChatWindow>
        ├── <MessageList>
        │   ├── <Message role="user" />
        │   ├── <Message role="assistant">
        │   │   └── <SourceCard onClick={navigateToSource} />
        │   └── ...
        ├── <InputBox 
        │     onSubmit={handleSend}
        │     selectedText={contextText}
        │   />
        └── <ErrorBoundary />
  ```

### 8. Error Handling & Validation (FR-8)

#### API Unavailability (Answer to Question 2)
- **Scenario**: OpenAI API is down or unreachable
- **User-Facing Message**:
  ```
  ⚠️ I'm currently unavailable due to a technical issue.
  Please try again in a few moments.
  
  [Retry Now]
  ```
- **Backend Behavior**:
  - Log error with full context
  - Return 503 Service Unavailable
  - Implement exponential backoff retry (3 attempts)
  - Fallback: Return cached response if query is identical to recent one

#### Long/Nonsensical Queries (Answer to Question 3)
- **Validation Rules**:
  - Minimum query length: 3 characters
  - Maximum query length: 500 characters
  - Detect nonsensical input (>50% special characters/numbers)

- **User-Facing Messages**:
  ```
  // Too long
  ⚠️ Your question is too long (max 500 characters).
  Please shorten it or break it into multiple questions.
  
  // Too short
  ⚠️ Please provide a more detailed question.
  Example: "What is ROS 2?" or "Explain NVIDIA Isaac Sim"
  
  // Nonsensical
  ⚠️ I couldn't understand your question. Please rephrase it clearly.
  Try asking about Physical AI, ROS 2, robotics, or topics from the textbook.
  ```

- **Implementation**: Frontend validation + backend validation (defense in depth)

---

## Implementation Phases

### Phase 0: Research & Environment Setup (Days 1-2)
**Objective**: Set up all development tools, accounts, and verify connectivity

**Tasks**:
- [ ] Install Python 3.11+ and Node.js 18+
- [ ] Clone repository and create `backend/` directory
- [ ] Set up Python virtual environment
- [ ] Create Qdrant Cloud account (free tier)
  - Create cluster
  - Note URL and API key
- [ ] Create Neon Postgres account (free tier)
  - Create database
  - Note connection string
- [ ] Obtain OpenAI API key
  - Create account at platform.openai.com
  - Generate API key
  - Add $5-10 credits for testing
- [ ] Create `.env` file with all credentials
- [ ] Test database connections (Qdrant, Postgres, OpenAI)
- [ ] Research text chunking strategies for technical documentation
- [ ] Research OpenAI Agents SDK examples

**Deliverables**:
- `research.md`: Document findings on RAG best practices, chunking strategies
- Working `.env` file (gitignored)
- Verified connectivity to all external services

---

### Phase 1: Backend Core Services (Days 3-7)
**Objective**: Build foundational backend services and database layer

**Tasks**:
- [ ] **Day 3: Project Structure & Configuration**
  - Create all backend directories and `__init__.py` files
  - Implement `app/config.py` with pydantic-settings
  - Implement `app/models.py` with all Pydantic models
  - Create `requirements.txt` with dependencies
  - Set up logging in `app/utils/logger.py`

- [ ] **Day 4: Database Services**
  - Implement `app/services/postgres_service.py`
    - `create_tables()` method
    - `save_conversation()`, `save_message()` methods
    - `get_conversation_history()` method
  - Implement `scripts/setup_db.py`
  - Test Postgres operations
  - Write unit tests for Postgres service

- [ ] **Day 5: Embedding & Vector Services**
  - Implement `app/services/embedding_service.py`
    - `generate_embedding()` for single text
    - `generate_embeddings_batch()` for multiple texts
  - Implement `app/services/qdrant_service.py`
    - `_ensure_collection()` method
    - `upsert_documents()` method
    - `search()` method with filters
  - Test embedding generation and vector storage
  - Write unit tests for both services

- [ ] **Day 6: Document Processing**
  - Implement `app/utils/markdown_parser.py`
    - Parse frontmatter (title, module, chapter)
    - Extract main content
    - Handle code blocks specially
  - Implement `app/utils/text_processing.py`
    - Chunking logic (1000 chars, 200 overlap)
    - Preserve paragraph boundaries
    - Metadata extraction
  - Test on sample markdown files
  - Write unit tests for utilities

- [ ] **Day 7: Document Ingestion**
  - Implement `scripts/ingest_documents.py`
    - Read all files from `docs/` recursively
    - Parse and chunk each document
    - Generate embeddings in batches
    - Upsert to Qdrant with metadata
    - Progress logging
  - Run full ingestion on textbook
  - Verify in Qdrant dashboard (count vectors, check metadata)
  - Write integration test for ingestion

**Deliverables**:
- `data-model.md`: Document database schemas, vector payload structure
- All service modules implemented and tested
- Full textbook ingested into Qdrant
- Unit test coverage: 80%+

---

### Phase 2: RAG Agent & API (Days 8-12)
**Objective**: Build OpenAI Agents integration and expose REST API

**Tasks**:
- [ ] **Day 8: Agent Service**
  - Implement `app/services/agent_service.py`
    - `retrieve_context()`: Query Qdrant, build context string
    - `generate_response()`: Call OpenAI Agents SDK
    - Handle selected text prioritization
    - Implement system prompt template
  - Test with sample queries
  - Write unit tests for agent service

- [ ] **Day 9: FastAPI Application**
  - Implement `app/main.py`
    - Initialize FastAPI app
    - Configure CORS
    - Include routers
    - Startup event for DB initialization
  - Implement `app/routes/health.py`
    - Health check endpoint
    - Service connectivity checks
  - Test health endpoint

- [ ] **Day 10: Chat Endpoints**
  - Implement `app/routes/chat.py`
    - POST `/api/chat` endpoint
    - GET `/api/conversations/{id}` endpoint
    - Request validation
    - Error handling
  - Test chat flow end-to-end
  - Write integration tests for chat endpoints

- [ ] **Day 11: Ingestion Endpoint**
  - Implement `app/routes/ingest.py`
    - POST `/api/ingest` endpoint
    - Call ingestion script
    - Return statistics
  - Test re-ingestion

- [ ] **Day 12: Backend Testing & Optimization**
  - Complete all unit tests
  - Complete all integration tests
  - Performance profiling (identify slow queries)
  - Optimize vector search parameters
  - Add request/response logging
  - Document API with OpenAPI/Swagger

**Deliverables**:
- `contracts/api.md`: API endpoint documentation with examples
- Fully functional backend with all endpoints
- Postman/curl tests for all endpoints
- API documentation at `/docs` (Swagger UI)

---

### Phase 3: Frontend Chat Widget (Days 13-17)
**Objective**: Build React chat UI and integrate with Docusaurus

**Tasks**:
- [ ] **Day 13: Chat Widget Setup**
  - Create `src/components/ChatWidget/` directory
  - Implement `types.ts` (TypeScript interfaces)
  - Implement `api.ts` (Axios client for backend)
  - Create basic `index.tsx` (ChatWidget main component)
  - Create `styles.module.css` (basic styles)
  - Add ChatWidget to Docusaurus theme (`src/theme/Root.tsx`)

- [ ] **Day 14: Chat Components**
  - Implement `ChatWindow.tsx`
    - Modal container
    - Open/close animations
    - Responsive layout
  - Implement `MessageList.tsx`
    - Message display
    - Auto-scroll to bottom
    - Loading indicator
  - Implement `InputBox.tsx`
    - Text input with submit
    - Character counter
    - Enter to send (Shift+Enter for newline)
    - Context box for selected text

- [ ] **Day 15: Source Citations & Error Handling**
  - Implement `SourceCard.tsx`
    - Display source content (truncated)
    - Expand/collapse functionality
    - Click to navigate (Option B implementation)
    - Metadata display (module, chapter)
  - Implement `ErrorBoundary.tsx`
    - Catch React errors
    - Display fallback UI
  - Implement error states in ChatWindow
    - API unavailable message with retry button
    - Validation error messages
    - Network error handling

- [ ] **Day 16: Text Selection Popup** ⭐
  - Implement `TextSelectionPopup.tsx`
    - Detect text selection (mouseup/touchend)
    - Calculate popup position (above/below selection)
    - Display popup with buttons
    - Handle "Ask about this" click
    - Handle "Explain in detail" click
  - Add selection highlighting (yellow background)
  - Integrate with ChatWidget (open with context)
  - Test on various screen sizes

- [ ] **Day 17: Frontend Polish & Testing**
  - Mobile responsiveness testing
  - Cross-browser testing (Chrome, Firefox, Safari)
  - Accessibility improvements (keyboard navigation, ARIA labels)
  - Loading states and animations
  - Dark mode support (if Docusaurus has dark theme)
  - Write frontend tests with React Testing Library

**Deliverables**:
- `quickstart.md`: Guide for running frontend locally
- Fully functional chat widget on all Docusaurus pages
- Text selection popup working
- Source navigation working (Option B)
- Responsive on mobile and desktop

---

### Phase 4: Integration Testing & Bug Fixes (Days 18-20)
**Objective**: Test entire system end-to-end and fix bugs

**Tasks**:
- [ ] **Day 18: End-to-End Testing**
  - Test complete user flows:
    1. Open chat → Ask question → Receive answer with sources
    2. Click source → Navigate to book section → Verify scroll
    3. Select text → Click popup → Ask question → Verify context
    4. Continue conversation → Verify history maintained
    5. Refresh page → Verify conversation persists (sessionStorage in frontend)
  - Test error scenarios:
    1. Backend down → Verify error message and retry
    2. Query too long → Verify validation error
    3. Empty query → Verify validation error
    4. Nonsensical query → Verify helpful message
  - Performance testing:
    - Measure API response times
    - Test with 5-10 concurrent users (simple load test)

- [ ] **Day 19: Bug Fixes & Edge Cases**
  - Fix any bugs found during testing
  - Handle edge cases:
    - Very long textbook sections (chunking boundary issues)
    - Special characters in queries
    - Code blocks in selected text
    - Empty search results
    - Network timeouts
  - Improve error messages based on testing

- [ ] **Day 20: Documentation & Code Cleanup**
  - Add docstrings to all Python functions
  - Add JSDoc comments to all React components
  - Update `backend/README.md` with setup instructions
  - Update root `README.md` with chatbot documentation
  - Code formatting (black for Python, prettier for TypeScript)
  - Remove debug code and console.logs
  - Create `.env.example` for both backend and frontend

**Deliverables**:
- Fully tested system with no critical bugs
- All documentation complete
- Clean, production-ready code

---

### Phase 5: Deployment (Days 21-23)
**Objective**: Deploy backend and frontend to production

**Tasks**:
- [ ] **Day 21: Backend Deployment**
  - Choose deployment platform (Railway.app recommended)
  - Create `Procfile`: `web: uvicorn app.main:app --host 0.0.0.0 --port $PORT`
  - Configure environment variables in platform dashboard
  - Deploy backend
  - Verify all endpoints working in production
  - Test from production URL

- [ ] **Day 22: Frontend Deployment**
  - Update API endpoint URLs in frontend (production backend URL)
  - Update CORS settings in backend (add production frontend URL)
  - Build Docusaurus: `npm run build`
  - Deploy to GitHub Pages: `npm run deploy`
  - Verify chat widget working on production site
  - Test all features on production

- [ ] **Day 23: Final Testing & Demo Preparation**
  - Full end-to-end test on production
  - Monitor logs for any errors
  - Performance check (response times)
  - Create demo video (<90 seconds)
    - Show chatbot answering questions
    - Show text selection feature
    - Show source navigation
    - Show conversation history
  - Prepare submission form data:
    - GitHub repo link (public)
    - Published book link (GitHub Pages or Vercel)
    - Demo video link (YouTube/Loom)
    - WhatsApp number

**Deliverables**:
- Backend deployed and accessible
- Frontend deployed to GitHub Pages
- Demo video ready
- Submission form filled
- Project ready for hackathon judging

---

## Testing Strategy

### Unit Tests (Target: 80% Coverage)
**Backend**:
- `tests/test_services.py`:
  - Test `EmbeddingService.generate_embedding()`
  - Test `QdrantService.upsert_documents()`, `search()`
  - Test `PostgresService` CRUD operations
  - Test `AgentService.retrieve_context()`, `generate_response()`
- `tests/test_utils.py`:
  - Test text chunking logic
  - Test markdown parsing
  - Test metadata extraction

**Frontend**:
- `ChatWidget.test.tsx`: Test component rendering
- `TextSelectionPopup.test.tsx`: Test popup display logic
- `api.test.ts`: Test API client functions (mocked)

### Integration Tests
- `tests/test_api.py`:
  - Test POST `/api/chat` with valid/invalid inputs
  - Test GET `/api/conversations/{id}`
  - Test POST `/api/ingest`
  - Test GET `/api/health`
  - Test CORS headers
  - Test error responses

### End-to-End Tests
- User asks question → receives answer (full RAG pipeline)
- User selects text → asks question → gets context-aware answer
- User clicks source → navigates to correct book section
- User continues conversation → history maintained
- Multiple users chat simultaneously → no interference

### Performance Tests
- Load test: 10 concurrent chat requests
- Measure vector search latency
- Measure embedding generation time
- Measure total API response time
- Identify bottlenecks

### Manual Testing Checklist
- [ ] Chat on desktop Chrome
- [ ] Chat on desktop Firefox
- [ ] Chat on mobile Safari
- [ ] Chat on mobile Chrome
- [ ] Select text and ask question
- [ ] Ask follow-up questions (test context)
- [ ] Test very long query (>500 chars)
- [ ] Test empty query
- [ ] Test nonsensical query
- [ ] Test with backend down (error handling)
- [ ] Test source navigation
- [ ] Test on slow network (loading states)
- [ ] Test on different Docusaurus pages
- [ ] Test conversation persistence across navigation

---

## Error Handling Strategy

### Input Validation (Frontend + Backend)
```typescript
// Frontend validation
const validateQuery = (query: string): string | null => {
  if (query.length < 3) {
    return "Please provide a more detailed question.";
  }
  if (query.length > 500) {
    return "Your question is too long. Please keep it under 500 characters.";
  }
  if (/^[^a-zA-Z0-9\s]{10,}/.test(query)) {
    return "I couldn't understand your question. Please rephrase it clearly.";
  }
  return null; // Valid
};
```

```python
# Backend validation
from fastapi import HTTPException

def validate_chat_request(request: ChatRequest):
    if len(request.message) < 3:
        raise HTTPException(
            status_code=400, 
            detail="Query too short. Minimum 3 characters."
        )
    if len(request.message) > 500:
        raise HTTPException(
            status_code=400, 
            detail="Query too long. Maximum 500 characters."
        )
    if request.selected_text and len(request.selected_text) > 2000:
        raise HTTPException(
            status_code=400, 
            detail="Selected text too long. Maximum 2000 characters."
        )
```

### API Error Handling
```python
# In agent_service.py
async def generate_response(self, query: str, context: str) -> str:
    try:
        response = await self.client.agents.create_and_run(...)
        return response
    except openai.APIError as e:
        logger.error(f"OpenAI API error: {e}")
        raise HTTPException(status_code=503, detail="AI service temporarily unavailable")
    except openai.RateLimitError as e:
        logger.error(f"OpenAI rate limit: {e}")
        raise HTTPException(status_code=429, detail="Too many requests. Please try again later.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
```

### Frontend Error Display
```typescript
// In ChatWindow.tsx
{error && (
  <div className="error-container">
    <div className="error-icon">⚠️</div>
    <div className="error-message">{error.message}</div>
    {error.code === 'SERVICE_UNAVAILABLE' && (
      <button onClick={handleRetry} className="retry-button">
        Retry Now
      </button>
    )}
  </div>
)}
```

---

## Performance Optimization

### Backend Optimizations
1. **Batch Embedding Generation**: Generate embeddings in batches of 10-20 instead of one-by-one
2. **Database Connection Pooling**: Use connection pool for Postgres
3. **Caching**: Cache frequent queries (future enhancement)
4. **Async Operations**: Use asyncio for I/O-bound operations
5. **Vector Search Tuning**: Optimize Qdrant search parameters (ef, m)

### Frontend Optimizations
1. **Lazy Loading**: Load ChatWidget only when chat button clicked
2. **Debouncing**: Debounce typing indicator (don't send on every keystroke)
3. **Message Pagination**: Load only last 20 messages initially
4. **Image Optimization**: Optimize any icons/images used
5. **Code Splitting**: Split React components for faster initial load

### Monitoring
- Log API response times
- Track token usage (OpenAI API)
- Monitor error rates
- Track user engagement (messages per session)

---

## Security Considerations

### API Key Protection
- Never commit `.env` file to Git (add to `.gitignore`)
- Use environment variables in production
- Rotate keys periodically
- Use separate keys for dev/prod

### Input Sanitization
- Validate all user inputs
- Prevent SQL injection (use parameterized queries)
- Prevent XSS attacks (sanitize HTML in markdown)
- Rate limiting (future enhancement)

### CORS Configuration
```python
# Production CORS
allow_origins = [
    "https://yourusername.github.io",  # GitHub Pages
    "http://localhost:3000"            # Dev only
]
```

### Content Safety
- Monitor for inappropriate queries (future: content filter)
- Log all queries for abuse detection
- Implement rate limiting per IP (future enhancement)

---

## Deployment Configuration

### Backend Deployment (Railway.app)

**Procfile**:
```
web: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**railway.json**:
```json
{
  "build": {
    "builder": "NIXPACKS"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "healthcheckPath": "/api/health"
  }
}
```

**Environment Variables** (set in Railway dashboard):
```
OPENAI_API_KEY=sk-proj-...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
POSTGRES_URL=postgresql://...
CORS_ORIGINS=["https://yourusername.github.io"]
```

### Frontend Deployment (GitHub Pages)

**docusaurus.config.js**:
```javascript
module.exports = {
  url: 'https://yourusername.github.io',
  baseUrl: '/physical-ai-robotics-textbook/',
  organizationName: 'yourusername',
  projectName: 'physical-ai-robotics-textbook',
  
  // Add backend URL
  customFields: {
    backendUrl: 'https://your-backend.railway.app'
  }
};
```

**Deploy command**:
```bash
npm run build
npm run deploy  # Deploys to gh-pages branch
```

---

## Success Metrics

### Technical Metrics
- ✅ **Ingestion Success**: 100% of documents processed
- ✅ **Search Accuracy**: >80% of queries return relevant results
- ✅ **Response Time P95**: <3 seconds
- ✅ **API Uptime**: >99%
- ✅ **Error Rate**: <1%
- ✅ **Test Coverage**: >80%

### User Experience Metrics
- ✅ **Query Relevance**: Sources are actually relevant to questions
- ✅ **Conversation Flow**: Context maintained across messages
- ✅ **Mobile Usability**: Smooth experience on mobile devices
- ✅ **Error Recovery**: Users can recover from errors easily

### Hackathon Scoring (200 points possible)
- **Base Functionality** (100 points):
  - RAG chatbot working
  - Embedded in Docusaurus
  - Selected text feature
  - Source citations
- **Bonus Points** (100 points possible):
  - Code quality and documentation (+20)
  - Text selection popup innovation (+20)
  - Error handling robustness (+20)
  - Mobile responsiveness (+20)
  - Performance optimization (+20)

---

## Risk Management

### High-Risk Items
| Risk | Impact | Mitigation |
|------|--------|------------|
| OpenAI API quota exceeded | High | Monitor usage, implement caching, use rate limiting |
| Qdrant free tier limits hit | High | Optimize vector storage, consider paid tier if needed |
| Document ingestion fails | High | Robust error handling, save progress, allow resume |
| Frontend-backend CORS issues | Medium | Test CORS early, proper configuration in both ends |
| Poor search relevance | Medium | Experiment with chunking strategies, embedding models |

### Contingency Plans
- **OpenAI outage**: Display graceful error, implement retry logic
- **Qdrant outage**: Cache recent queries (future enhancement)
- **Postgres outage**: Degrade gracefully (no history, but still answer questions)
- **Deployment issues**: Have local demo ready, screen recording backup

---

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Using OpenAI instead of Gemini (constitution preference) | Hackathon specification explicitly requires OpenAI Agents SDK, OpenAI embeddings, and GPT-4o. These are core technical requirements. | Gemini doesn't have an Agents SDK equivalent, has different embedding dimensions (768 vs 1536), and would require complete architecture redesign. Hackathon requirements take precedence. |
| Complex text selection feature | Essential for meeting hackathon bonus points and providing superior UX. Allows users to get context-specific answers. | Simple chat-only implementation would score lower and miss key functionality that makes the chatbot truly useful for learning from the textbook. |
| Separate backend deployment | FastAPI backend cannot run on GitHub Pages (static hosting). RAG pipeline needs server-side processing. | Serverless functions (Vercel, Netlify) would add complexity and cost. Railway provides free tier with proper backend support. |

---

## Timeline Summary

| Phase | Duration | Key Deliverables |
|-------|----------|-----------------|
| Phase 0: Setup | 2 days | Environment configured, accounts created, credentials obtained |
| Phase 1: Backend Core | 5 days | All services implemented, documents ingested, database ready |
| Phase 2: RAG & API | 5 days | OpenAI agent working, API endpoints functional, Swagger docs |
| Phase 3: Frontend | 5 days | Chat widget complete, text selection popup, source navigation |
| Phase 4: Testing | 3 days | All tests passing, bugs fixed, documentation complete |
| Phase 5: Deployment | 3 days | Production deployment, demo video, submission ready |
| **TOTAL** | **23 days** | **Fully functional RAG chatbot deployed and tested** |

**Buffer**: 7 days for unexpected issues, additional testing, or feature polish

**Hackathon Deadline**: November 30, 2025 at 6:00 PM
**Recommended Start Date**: November 1, 2025 (to allow 30-day timeline with buffer)

---

## Final Checklist

### Pre-Submission Checklist
- [ ] All documents in `docs/` folder successfully ingested to Qdrant
- [ ] Chat widget visible and functional on all Docusaurus pages
- [ ] User can ask questions and receive accurate answers with sources
- [ ] Text selection popup working (10-2000 character range)
- [ ] Selected text context properly passed to backend
- [ ] Source citations clickable and navigate correctly (Option B)
- [ ] Error handling working (API down, long queries, nonsensical input)
- [ ] Conversation history persists across page navigation
- [ ] Mobile experience is smooth (tested on iOS and Android)
- [ ] Backend deployed and accessible via HTTPS
- [ ] Frontend deployed to GitHub Pages
- [ ] All API endpoints tested and working
- [ ] Test coverage >80% for backend
- [ ] No console errors in browser
- [ ] Demo video created (<90 seconds)
- [ ] README.md updated with chatbot documentation
- [ ] GitHub repository is public
- [ ] Submission form filled with all required information

### Submission Package
1. **Public GitHub Repository Link**: https://github.com/yourusername/physical-ai-robotics-textbook
2. **Published Book Link**: https://yourusername.github.io/physical-ai-robotics-textbook/
3. **Demo Video Link**: https://youtu.be/... or https://loom.com/...
4. **WhatsApp Number**: +92-XXX-XXXXXXX

---

## Additional Resources

### Documentation Links
- FastAPI: https://fastapi.tiangolo.com/
- OpenAI API: https://platform.openai.com/docs
- OpenAI Agents SDK: https://platform.openai.com/docs/assistants/overview
- Qdrant: https://qdrant.tech/documentation/
- Neon Postgres: https://neon.tech/docs
- Docusaurus: https://docusaurus.io/docs
- React: https://react.dev/

### Learning Resources
- RAG Tutorial: https://www.pinecone.io/learn/retrieval-augmented-generation/
- Vector Databases Explained: https://www.qdrant.tech/articles/what-is-a-vector-database/
- OpenAI Embeddings Guide: https://platform.openai.com/docs/guides/embeddings

### Community Support
- FastAPI Discord: https://discord.gg/fastapi
- OpenAI Community: https://community.openai.com/
- Qdrant Discord: https://qdrant.to/discord
- Docusaurus Discord: https://discord.gg/docusaurus

---

**Plan Version**: 2.0 (Corrected)
**Last Updated**: December 7, 2024
**Plan Author**: Implementation Team
**Reviewed By**: Project Lead

---

## END OF IMPLEMENTATION PLAN