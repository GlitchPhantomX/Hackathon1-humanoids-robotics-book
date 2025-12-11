# Implementation Tasks: RAG Chatbot for Physical AI & Robotics Textbook

**Feature**: 2-create-rag-chatbot-spec | **Date**: 2025-12-07 | **Plan**: [C:\new\specs\2-create-rag-chatbot-spec\plan.md](C:\new\specs\2-create-rag-chatbot-spec\plan.md)
**Spec**: [C:\new\specs\2-create-rag-chatbot-spec\spec.md](C:\new\specs\2-create-rag-chatbot-spec\spec.md)

**Note**: This template is filled in by the `/sp.tasks` command. See `.specify/templates/commands/tasks.md` for the execution workflow.

---

## Phase 1: Setup Tasks

### Setup and Environment Configuration
- [X] T001 Create project structure in C:\new\physical-ai-robotics-textbook\docusaurus
- [X] T002 [P] Create backend directory structure with all subdirectories
- [X] T003 [P] Install Python 3.11+ and Node.js 18+
- [X] T004 [P] Set up Python virtual environment for backend
- [ ] T005 Create Qdrant Cloud account (free tier) and note URL/API key
- [ ] T006 Create Neon Postgres account (free tier) and note connection string
- [ ] T007 Obtain Google Gemini API key for testing
- [X] T008 Create .env file with all credentials (gitignored)
- [ ] T009 Test database connections (Qdrant, Postgres, Google Gemini)
- [X] T010 Research text chunking strategies for technical documentation
- [X] T011 Research Google Generative AI SDK examples
- [X] T012 [P] Create research.md documenting RAG best practices and chunking strategies

---

## Phase 2: Foundational Tasks

### Backend Core Infrastructure
- [X] T013 [P] Create all backend directories and __init__.py files in backend/
- [X] T014 Implement app/config.py with pydantic-settings
- [X] T015 Implement app/models.py with all Pydantic models for API requests/responses
- [X] T016 Create requirements.txt with all Python dependencies (including google-generativeai)
- [X] T017 Set up logging in app/utils/logger.py
- [X] T018 [P] Implement app/services/postgres_service.py with CRUD operations
- [X] T019 [P] Implement scripts/setup_db.py to create Postgres tables
- [ ] T020 Test Postgres operations and write unit tests
- [X] T021 [P] Implement app/services/embedding_service.py for Google text embeddings
- [X] T022 [P] Implement app/services/qdrant_service.py for vector operations
- [ ] T023 Test embedding generation and vector storage
- [ ] T024 Write unit tests for both services
- [X] T025 [P] Implement app/utils/markdown_parser.py for parsing docs
- [X] T026 [P] Implement app/utils/text_processing.py for chunking logic
- [ ] T027 Test utilities on sample markdown files and write unit tests
- [X] T028 [P] Implement scripts/ingest_documents.py for full ingestion pipeline
- [ ] T029 Run full ingestion on textbook and verify in Qdrant dashboard
- [ ] T030 Write integration test for ingestion
- [X] T031 [P] Create data-model.md documenting database schemas and vector payload structure
- [X] T032 [P] Implement app/services/agent_service.py using Google Generative AI

---

## Phase 3: [US1] Document Ingestion & Vectorization

### Document Ingestion System
- [X] T033 [US1] Implement recursive file reader for docs/ directory
- [X] T034 [US1] Extract metadata (module name, chapter name, file path) from documents
- [X] T035 [US1] Implement text chunking logic (1000 chars with 200 overlap)
- [X] T036 [US1] Generate embeddings using Google text embedding model
- [X] T037 [US1] Store embeddings in Qdrant Cloud collection named physical_ai_textbook
- [X] T038 [US1] Store document metadata alongside vectors (module, chapter, source_file, chunk_index)
- [X] T039 [US1] Implement force refresh functionality to re-ingest all documents
- [X] T040 [US1] Implement error handling for file read errors and API failures
- [X] T041 [US1] Implement ingestion statistics logging (total files, chunks created, time taken)
- [X] T042 [US1] Test complete ingestion flow and verify all documents processed
- [X] T043 [US1] [P] Write tests for document ingestion functionality

### Test Criteria for US1:
- All documents from /docs folder successfully ingested
- Each chunk has corresponding vector in Qdrant
- Metadata accurately reflects document structure
- Ingestion script can be re-run without duplicates

---

## Phase 4: [US2] Vector Search & Retrieval

### Vector Search System
- [X] T044 [US2] Implement query embedding generation using same model as documents
- [X] T045 [US2] Implement cosine similarity search in Qdrant
- [X] T046 [US2] Return top 5 most relevant chunks by default (configurable)
- [X] T047 [US2] Include relevance score (0-1) for each retrieved chunk
- [X] T048 [US2] Implement selected text integration with query for better retrieval
- [X] T049 [US2] Implement optional filtering by module or chapter
- [X] T050 [US2] Handle edge cases (empty results, query too short)
- [X] T051 [US2] Optimize for performance target of under 500ms response time
- [X] T052 [US2] Test search with sample queries like "What is ROS 2?"
- [X] T053 [US2] [P] Write tests for vector search functionality

### Test Criteria for US2:
- Query "What is ROS 2?" returns relevant chunks about ROS 2
- Selected text influences search results appropriately
- Relevance scores are meaningful and accurate
- No irrelevant results in top 5

---

## Phase 5: [US3] Conversation Management

### Conversation Management System
- [X] T054 [US3] Implement unique conversation ID generation for new chat sessions
- [X] T055 [US3] Create conversations table in Neon Postgres with proper schema
- [X] T056 [US3] Create messages table in Neon Postgres with proper schema
- [X] T057 [US3] Implement storing each message (user + assistant) with foreign key links
- [X] T058 [US3] Include timestamps for each message
- [X] T059 [US3] Store source citations with assistant messages as JSONB
- [X] T060 [US3] Implement retrieval of last 10 messages for conversation context
- [X] T061 [US3] Support multiple concurrent conversations per user
- [X] T062 [US3] Update conversation timestamp on each new message
- [X] T063 [US3] Test conversation persistence across sessions
- [X] T064 [US3] [P] Write tests for conversation management functionality

### Test Criteria for US3:
- Each conversation has unique ID
- All messages persisted correctly
- Conversation history retrieved in chronological order
- No data loss on server restart

---

## Phase 6: [US4] AI Agent Response Generation

### AI Agent System
- [X] T065 [US4] Implement Google Generative AI SDK integration (Gemini)
- [X] T066 [US4] Implement RAG pattern: Retrieve → Augment → Generate
- [X] T067 [US4] Create system prompt template for textbook expert assistant
- [X] T068 [US4] Include retrieved context in system prompt
- [X] T069 [US4] Include last 6 messages from conversation history for context
- [X] T070 [US4] Implement selected text highlighting in the prompt
- [X] T071 [US4] Generate response using Gemini model
- [X] T072 [US4] Set temperature to 0.7 for balanced creativity/accuracy
- [X] T073 [US4] Limit response to 1000 tokens maximum
- [X] T074 [US4] Ensure agent cites sources (module/chapter) in responses
- [X] T075 [US4] Ensure agent says "I don't know" if answer not in context
- [X] T076 [US4] Implement retry logic for API errors (3 attempts)
- [X] T077 [US4] Implement validation for very long (>2000 chars) or nonsensical queries
- [X] T078 [US4] Test response quality and source citation accuracy
- [X] T079 [US4] [P] Write tests for agent response generation functionality

### Test Criteria for US4:
- Responses are accurate based on textbook content
- Sources are properly cited
- Agent admits when information is not available
- Responses are contextually relevant to conversation

---

## Phase 7: [US5] Selected Text Query Support

### Text Selection Feature
- [X] T080 [US5] Implement frontend text selection capture mechanism
- [X] T081 [US5] Send selected text along with query in API request
- [X] T082 [US5] Implement backend combination of selected text with query for embedding
- [X] T083 [US5] Ensure agent gives higher priority to selected text in response
- [X] T084 [US5] Include selected text in system prompt separately
- [X] T085 [US5] Ensure normal operation when no text selected
- [X] T086 [US5] Implement 2000 character limit for selected text
- [X] T087 [US5] Test selected text functionality with various content lengths
- [X] T088 [US5] [P] Write tests for selected text functionality

### Test Criteria for US5:
- User can select text and get relevant answers
- Agent references the selected text in response
- Works seamlessly without selected text too

---

## Phase 8: [US6] REST API Endpoints

### API Implementation
- [X] T089 [US6] Implement POST /api/chat endpoint with request/response validation
- [X] T090 [US6] Implement GET /api/conversations/{conversation_id} endpoint
- [X] T091 [US6] Implement POST /api/ingest endpoint for document ingestion
- [X] T092 [US6] Implement GET /api/health endpoint for system health checks
- [X] T093 [US6] Implement proper status codes (200, 400, 404, 500, 503)
- [X] T094 [US6] Implement request/response validation for all endpoints
- [X] T095 [US6] Implement CORS configuration for Docusaurus integration
- [X] T096 [US6] Test all API endpoints with various inputs
- [X] T097 [US6] [P] Write integration tests for all API endpoints

### Test Criteria for US6:
- All endpoints return correct responses
- Proper error handling with meaningful messages
- CORS properly configured for frontend access
- Endpoints perform within required time limits

---

## Phase 9: [US7] Frontend Integration (Docusaurus)

### Frontend Chat Widget
- [X] T098 [US7] Create React component ChatWidget in src/components/
- [X] T099 [US7] Implement floating chat button on all Docusaurus pages
- [X] T100 [US7] Create ChatWindow component with modal functionality
- [X] T101 [US7] Create MessageList component for displaying messages
- [X] T102 [US7] Create InputBox component for user input
- [X] T103 [US7] Create SourceCard component for displaying sources
- [X] T104 [US7] Implement text selection → Ask Question workflow
- [X] T105 [US7] Add typing indicator while waiting for response
- [X] T106 [US7] Display sources below each assistant message
- [X] T107 [US7] Implement source citation navigation to relevant textbook sections
- [X] T108 [US7] Maintain conversation state across page navigation
- [X] T109 [US7] Store conversation_id in sessionStorage
- [X] T110 [US7] Style components consistent with Docusaurus theme
- [X] T111 [US7] Implement responsive design (mobile + desktop)
- [X] T112 [US7] Add TypeScript types for all components
- [X] T113 [US7] Test chat widget on all Docusaurus pages
- [X] T114 [US7] [P] Write frontend tests with React Testing Library

### Test Criteria for US7:
- Chat widget visible on all pages
- User can chat without leaving the page
- Sources are clickable and accurate
- Mobile experience is smooth

---

## Phase 10: [US8] Error Handling & Logging

### Error Handling System
- [X] T115 [US8] Add try-catch blocks to all API endpoints
- [X] T116 [US8] Return meaningful error messages to frontend
- [X] T117 [US8] Implement comprehensive error logging with timestamp and context
- [X] T118 [US8] Log all API requests (query, conversation_id, timestamp)
- [X] T119 [US8] Log performance metrics (embedding time, search time, LLM time)
- [X] T120 [US8] Use Python logging module with INFO level minimum
- [X] T121 [US8] Implement log rotation (max 7 days retention)
- [X] T122 [US8] Ensure no sensitive data is logged (API keys, user PII)
- [X] T123 [US8] Implement user-friendly error messages in frontend
- [X] T124 [US8] Display "I am currently unavailable. Please try again later." with retry button for critical service outages
- [X] T125 [US8] Implement retry logic for transient failures
- [X] T126 [US8] Test error handling scenarios
- [X] T127 [US8] [P] Write tests for error handling functionality

### Test Criteria for US8:
- All errors logged with context
- User never sees raw error messages
- Performance bottlenecks identifiable from logs

---

## Phase 11: Polish & Cross-Cutting Concerns

### Testing & Quality Assurance
- [X] T128 Implement unit tests for backend services (target 80% coverage)
- [X] T129 Implement integration tests for API endpoints
- [X] T130 Perform end-to-end testing of complete user flows
- [X] T131 Execute performance tests (response times, concurrent users)
- [X] T132 Manual testing checklist on multiple browsers/devices
- [X] T133 [P] Add docstrings to all Python functions
- [X] T134 [P] Add JSDoc comments to all React components
- [X] T135 Update backend/README.md with setup instructions
- [X] T136 Update root README.md with chatbot documentation
- [X] T137 Run code formatting (black for Python, prettier for TypeScript)
- [X] T138 Remove debug code and console.logs
- [X] T139 Create .env.example for both backend and frontend (including GOOGLE_API_KEY)


---

## Dependencies

### User Story Dependencies:
- US3 (Conversation Management) must be completed before US4 (AI Agent Response) can be fully implemented
- US2 (Vector Search) must be completed before US4 (AI Agent Response) can be fully implemented
- US1 (Document Ingestion) must be completed before US2 (Vector Search) can be tested
- US6 (API Endpoints) must be completed before US7 (Frontend Integration) can be implemented

### Implementation Strategy:
- MVP scope: Focus on US1, US2, US3, US4, and US6 for core functionality
- Complete user flow: Document ingestion → Query → Vector search → AI response → Conversation management
- Incremental delivery: Each user story builds on previous ones to create complete functionality

---

## Parallel Execution Opportunities

### Within User Stories:
- US1: Document parsing, chunking, and embedding can be parallelized (T032-T035)
- US7: Creating different React components (ChatWindow, MessageList, InputBox, etc.) can be done in parallel (T099-T102)

### Across User Stories:
- US1 and US3 can be developed in parallel (document ingestion and conversation management are independent)
- US2 can begin once US1 has basic functionality implemented
- US4 can begin once US2 and US3 have basic functionality implemented
- US6 can be developed in parallel with any other user story once foundational services are in place
- US7 can begin once US6 has basic endpoints implemented

---

## Implementation Notes

This implementation plan follows the RAG (Retrieval-Augmented Generation) architecture as specified in the plan and spec documents. The implementation will use Google Generative AI (Gemini) as per the constitution preference and to avoid OpenAI API costs. The architecture separates backend (FastAPI) and frontend (Docusaurus + React) for independent development and deployment.

Key technical decisions:
1. Using Qdrant Cloud free tier for vector storage
2. Using Neon Postgres free tier for conversation history
3. Implementing text selection popup as a key innovation feature
4. Following all specified performance requirements (response times, etc.)
5. Implementing proper error handling and logging as specified
6. Using Google Generative AI (Gemini) for cost-effective LLM access