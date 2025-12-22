# Research: Urdu Translation Toggle

## Decision: Translation Technology Stack
**Rationale**: Using existing FastAPI backend with LLM service for translation aligns with project architecture and constitution requirements. TypeScript/React frontend integrates well with Docusaurus.
**Alternatives considered**:
- Pure client-side translation (violates security requirements)
- Different backend frameworks (would not integrate with existing infrastructure)

## Decision: Caching Strategy
**Rationale**: In-memory caching with user_id + chapter_id + language key provides optimal performance while meeting security requirements. Can be extended to Redis for production.
**Alternatives considered**:
- Database storage (higher latency)
- Local storage in browser (security concerns, not server-side cacheable)
- File-based storage (adequate for prototype)

## Decision: RTL Implementation
**Rationale**: CSS-based RTL with dynamic class switching provides clean separation of concerns and meets constitution requirements for RTL rendering.
**Alternatives considered**:
- JavaScript-based layout changes (more complex)
- Separate CSS files per language (increased bundle size)

## Decision: Authentication Integration
**Rationale**: Using existing session validation system ensures consistency with project security model.
**Alternatives considered**:
- JWT tokens (would require additional infrastructure)
- OAuth providers (over-engineering for this requirement)

## Decision: Markdown Preservation
**Rationale**: Using markdown parsing libraries that preserve structure during translation meets functional requirements.
**Alternatives considered**:
- HTML-based approach (loses markdown structure)
- Custom parsing (unnecessary complexity)