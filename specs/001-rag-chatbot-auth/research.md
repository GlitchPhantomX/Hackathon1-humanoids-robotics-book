# Research Document: RAG Chatbot Integration With Auth + Advanced Features

## Overview
This document outlines the research findings and technical approach for implementing the authenticated RAG chatbot with advanced features including selected text restriction, streaming responses, and source citations.

## Key Implementation Areas

### 1. Authentication Enforcement

**Decision**: Integrate with existing auth-backend system using session-based authentication
**Rationale**: Leverages existing authentication infrastructure without reinventing security mechanisms
**Implementation approach**:
- Frontend: Check authentication state using existing auth context/hooks
- Backend: Validate session tokens in chat API endpoint
- Return 401 Unauthorized for unauthenticated requests

**Alternatives considered**:
- JWT tokens: Would require additional token management infrastructure
- OAuth integration: Overly complex for existing auth system
- Custom auth flow: Risky from security perspective

### 2. Selected Text Restriction

**Decision**: Implement context restriction in RAG pipeline
**Rationale**: Maintains RAG logic while enforcing content boundaries
**Implementation approach**:
- Modify chat API to accept optional `selected_text` parameter
- When provided, use this text as the exclusive context for the LLM
- If insufficient information, return appropriate response

**Alternatives considered**:
- Pre-filtering documents: Would require complex text matching
- Post-processing responses: Doesn't guarantee restriction to selected text
- Separate endpoint: Unnecessary complexity

### 3. Streaming Responses

**Decision**: Implement Server-Sent Events (SSE) for real-time streaming
**Rationale**: Well-supported in browsers, efficient for text streaming
**Implementation approach**:
- Backend: Use FastAPI's StreamingResponse with SSE format
- Frontend: Use EventSource API to handle streaming responses
- Maintain existing UI while progressively updating content

**Alternatives considered**:
- WebSockets: More complex setup, unnecessary for one-way streaming
- Chunked HTTP responses: Less browser support, harder to handle errors
- Polling: Inefficient and not real-time

### 4. Source Citations

**Decision**: Extract and return source metadata from RAG retrieval process
**Rationale**: Provides transparency and traceability for RAG responses
**Implementation approach**:
- Enhance RAG retrieval to return document metadata alongside content
- Format citations with title, page/section, and relevance score
- Display citations below responses in frontend

**Alternatives considered**:
- No citations: Reduces trust and verifiability
- Generic citations: Less useful for verification
- External reference links: Requires additional infrastructure

### 5. CLI Compatibility

**Decision**: Maintain separate API endpoints for web and CLI
**Rationale**: Ensures zero disruption to existing CLI functionality
**Implementation approach**:
- Keep existing CLI entry points unchanged
- Add new authenticated endpoints for web features
- Share core RAG logic between both interfaces

**Alternatives considered**:
- Single unified interface: Risk of breaking existing CLI
- Forking the codebase: Creates maintenance overhead
- Wrapper approach: Adds unnecessary complexity

## Technical Architecture

### Frontend Components
- Chatbot component with authentication check
- Modal integration for login/signup flow
- Streaming response display
- Selected text injection
- Source citation display

### Backend Services
- Authenticated chat API endpoint
- RAG service with context restriction
- Streaming response generator
- Source metadata extraction

### Integration Points
- Existing auth-backend for authentication
- Current RAG system for response generation
- Docusaurus for UI integration
- Existing CLI functionality

## Risk Assessment

### High Priority
- Authentication bypass: Ensure proper validation at API level
- Session timeout during streaming: Handle gracefully with re-authentication

### Medium Priority
- Large text selection: Implement size limits to prevent API overload
- Streaming interruption: Add retry mechanisms and error handling

### Low Priority
- Citation format consistency: Standardize metadata across document types

## Dependencies and Tools

### Required Dependencies
- FastAPI (existing in rag-chatbot)
- SSE support libraries (if needed)
- Existing RAG libraries (LangChain/LlamaIndex)

### Development Tools
- TypeScript for frontend components
- React hooks for state management
- Docusaurus plugin system for integration

## Implementation Phases

### Phase 1: Authentication
- Implement auth checks in frontend
- Add authentication validation to backend
- Integrate with existing auth modals

### Phase 2: Core RAG Features
- Add selected text restriction
- Implement streaming responses
- Add source citations

### Phase 3: UI Integration
- Integrate chatbot component with Docusaurus
- Ensure backward compatibility with CLI
- Test all features together

## Performance Considerations

- Streaming response latency: Target <500ms initial response
- Authentication overhead: Minimize additional round trips
- Memory usage: Efficient handling of large selected text
- UI responsiveness: Non-blocking during streaming