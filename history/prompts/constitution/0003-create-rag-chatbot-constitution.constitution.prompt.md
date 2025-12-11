---
id: 0003
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini
feature: rag-chatbot
branch: 002-rag-chatbot-ui
user: user
command: /sp.constitution
labels: [rag-chatbot, constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - C:\new\.specify\memory\rag-chatbot-constitution.md
tests:
  (none)
---

## Prompt

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


/sp.constitution # RAG Chatbot Constitution
## Physical AI & Humanoid Robotics Textbook

### Version 1.0 | December 2025

---

## 1. PURPOSE & MISSION

### 1.1 Primary Objective
The RAG Chatbot serves as an intelligent learning companion embedded within the Physical AI & Humanoid Robotics textbook, providing contextual, accurate, and pedagogically sound responses to enhance student comprehension and learning outcomes.

### 1.2 Core Mission
- **Enhance Learning**: Facilitate deep understanding of Physical AI, ROS 2, Isaac Sim, and humanoid robotics concepts
- **Contextual Assistance**: Provide precise answers based on textbook content with proper citations
- **Adaptive Support**: Respond to both general queries and user-selected text snippets
- **Academic Integrity**: Maintain educational value while preventing misuse

---

## 2. TECHNICAL ARCHITECTURE

### 2.1 Technology Stack Requirements

#### Backend Framework
- **FastAPI** (Python 3.10+)
- Asynchronous request handling
- WebSocket support for real-time interactions
- CORS configuration for frontend integration

#### AI/ML Components
- **OpenAI Agents SDK** / **ChatKit SDK**
- GPT-4 or GPT-4-turbo model
- Embeddings: `text-embedding-3-large` or `text-embedding-ada-002`
- Function calling capabilities for structured responses

#### Vector Database
- **Qdrant Cloud Free Tier**
- Collection per textbook chapter/module
- Hybrid search (dense + sparse vectors)
- Payload filtering for metadata

#### Relational Database
- **Neon Serverless Postgres**
- User interaction logs
- Query history and analytics
- User preferences storage

#### Deployment
- Vercel/Railway for FastAPI backend
- GitHub Pages for static frontend
- Environment variable management

---

## 3. FUNCTIONAL REQUIREMENTS

### 3.1 Core Capabilities

#### A. General Query Handling
**Requirement**: Answer questions about any content within the textbook

**Specifications**:
- Accept natural language queries via chat interface
- Perform semantic search across entire textbook corpus
- Retrieve top-k relevant chunks (k=3-5)
- Generate contextual responses with citations
- Include chapter/section references

**Example Queries**:
- "Explain ROS 2 topics and services"
- "What hardware is needed for NVIDIA Isaac Sim?"
- "How does VSLAM work in humanoid robots?"

#### B. Selected Text Query Handling
**Requirement**: Answer questions based solely on user-highlighted text

**Specifications**:
- Accept selected text snippet (max 2000 characters)
- Create temporary context window from selection
- Restrict retrieval to selected content only
- Provide focused answers without external context bleeding
- Clear visual indication of "selection mode"

**Example Usage**:
1. User selects paragraph about Jetson Orin Nano specifications
2. User asks: "Why is 64GB RAM recommended?"
3. Bot answers based only on selected text, not entire chapter

#### C. Citation & Source Transparency
**Requirements**:
- Every response must include source attribution
- Format: `[Chapter X, Section Y.Z]`
- Clickable links to exact textbook sections
- Confidence scoring when appropriate

---

## 4. BEHAVIORAL GUIDELINES

### 4.1 Response Principles

#### Accuracy Above All
- **NEVER hallucinate** technical specifications, code syntax, or commands
- If information is not in the textbook, explicitly state: *"This specific detail is not covered in the current textbook content. I can provide general context, but please verify with official documentation."
- For ambiguous queries, ask clarifying questions

#### Pedagogical Approach
- **Explain, Don't Just Answer**: Provide context and reasoning
- **Progressive Disclosure**: Start with simple explanations, offer to elaborate
- **Socratic Method**: When appropriate, guide students to discover answers
- **Code Examples**: Include relevant code snippets from textbook when explaining concepts

#### Technical Precision
- Use correct terminology (e.g., "ROS 2 node" not "ROS node")
- Distinguish between simulation (Gazebo/Isaac Sim) and real hardware
- Clarify Ubuntu/Linux-specific instructions when relevant
- Reference correct version numbers (ROS 2 Humble, Ubuntu 22.04, etc.)

### 4.2 Prohibited Behaviors

#### Do NOT:
1. **Generate homework/assignment solutions** - Guide towards understanding instead
2. **Provide complete project code** - Offer architectural guidance and snippets
3. **Make hardware purchase recommendations** beyond textbook specifications
4. **Debug personal code** - Explain concepts, don't troubleshoot external code
5. **Discuss non-textbook robotics platforms** in depth (stay focused on course materials)

---

## 5. RAG PIPELINE SPECIFICATIONS

### 5.1 Document Processing

#### Chunking Strategy
```
- Chunk Size: 800-1200 tokens
- Overlap: 200 tokens
- Boundaries: Respect markdown headers, code blocks, lists
- Metadata: chapter, section, subsection, page_number (if applicable)
```

#### Embedding Process
```python
# Pseudocode specification
for each chunk:
    metadata = {
        "chapter": chunk.chapter_id,
        "section": chunk.section_title,
        "type": "concept" | "code" | "hardware_spec" | "example",
        "keywords": extracted_technical_terms
    }
    
    embedding = openai.embeddings.create(
        model="text-embedding-3-large",
        input=chunk.text
    )
    
    qdrant.upsert(
        collection="physical_ai_textbook",
        points=[{
            "id": chunk.id,
            "vector": embedding,
            "payload": metadata
        }]
    )
```

### 5.2 Retrieval Strategy

#### General Query Mode
1. **Query Expansion**: Extract technical terms and synonyms
2. **Hybrid Search**: Combine semantic + keyword matching
3. **Reranking**: Score by relevance and recency (prioritize later chapters if student progressed)
4. **Filtering**: Apply metadata filters if query mentions specific modules

#### Selected Text Mode
1. **Ephemeral Embedding**: Create temporary vector from selection
2. **Similarity Threshold**: Lower threshold (0.7 instead of 0.8) to be more inclusive
3. **Strict Boundary**: Only return results matching selected text's chapter/section

### 5.3 Generation Guidelines

#### Prompt Template Structure
```
SYSTEM: You are an expert teaching assistant for the Physical AI & Humanoid Robotics course. 
Use ONLY the provided context from the textbook to answer questions.

CONTEXT: {retrieved_chunks}

STUDENT QUESTION: {user_query}

INSTRUCTIONS:
- Answer based solely on provided context
- Cite sources: [Chapter X, Section Y]
- If context insufficient, state clearly
- Use technical precision
- Include code examples when relevant
- Explain complex concepts step-by-step

ANSWER:
```

---

## 6. USER EXPERIENCE REQUIREMENTS

### 6.1 Interface Elements

#### Chat Widget
- **Position**: Fixed bottom-right corner of textbook pages
- **Dimensions**: 400px × 600px (desktop), full-screen (mobile)
- **States**: Minimized, Expanded, Full-screen
- **Theme**: Match textbook color scheme

#### Features
- **Text Selection Integration**:
  - Right-click menu option: "Ask about this section"
  - Keyboard shortcut: Select text + `Ctrl+Shift+A`
- **Voice Input**: Optional speech-to-text for queries
- **Code Highlighting**: Syntax highlighting for code in responses
- **LaTeX Rendering**: Math equations properly formatted
- **Export Chat**: Download conversation history

### 6.2 Performance Standards
- **Response Time**: < 3 seconds for 95th percentile queries
- **Concurrent Users**: Support 50+ simultaneous users (Free Tier consideration)
- **Uptime**: 99% availability target
- **Mobile Responsive**: Full functionality on tablets/phones

---

## 7. DATA MANAGEMENT & PRIVACY

### 7.1 Data Storage (Neon Postgres)

#### Schema Design
```sql
-- User Sessions (Anonymous or Authenticated)
CREATE TABLE user_sessions (
    session_id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id) NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    last_activity TIMESTAMP,
    user_background JSONB  -- Hardware/software experience
);

-- Query Logs
CREATE TABLE query_logs (
    query_id UUID PRIMARY KEY,
    session_id UUID REFERENCES user_sessions(session_id),
    query_text TEXT NOT NULL,
    query_type VARCHAR(50), -- 'general' or 'selected_text'
    selected_context TEXT,
    response_text TEXT,
    sources JSONB,  -- Array of {chapter, section}
    response_time_ms INTEGER,
    user_feedback INTEGER,  -- -1, 0, 1 (thumbs down, neutral, thumbs up)
    created_at TIMESTAMP DEFAULT NOW()
);

-- Analytics
CREATE TABLE popular_topics (
    topic_id SERIAL PRIMARY KEY,
    chapter VARCHAR(100),
    section VARCHAR(100),
    query_count INTEGER DEFAULT 0,
    avg_satisfaction FLOAT,
    last_queried TIMESTAMP
);
```

### 7.2 Privacy Standards
- **No PII Storage**: Do not store names, emails in chat logs
- **Anonymous Mode**: Allow chatbot use without signup (session-based)
- **Data Retention**: Delete query logs older than 90 days
- **GDPR Compliance**: Provide data export/deletion upon request

---

## 8. QUALITY ASSURANCE

### 8.1 Testing Requirements

#### Unit Tests
- RAG pipeline accuracy (precision/recall on test queries)
- Embedding consistency
- Citation link validation

#### Integration Tests
- End-to-end query flow (frontend → backend → Qdrant → OpenAI → response)
- Selected text isolation (ensure no context bleeding)
- Database write/read operations

#### User Acceptance Tests
- 20+ sample queries covering all modules
- Edge cases: Empty selection, very long queries, code-heavy questions
- Load testing: 100 concurrent requests

### 8.2 Evaluation Metrics

#### Response Quality
- **Accuracy**: % of factually correct responses (manual review of 100 samples)
- **Relevance**: Average retrieval precision@5
- **Citation Rate**: % of responses with proper sources (target: 100%)

#### User Satisfaction
- **Feedback Score**: Track thumbs up/down ratio (target: > 75% positive)
- **Resolution Rate**: % of queries answered without follow-up "I don't understand"
- **Usage Analytics**: Daily active users, queries per session

---

## 9. MAINTENANCE & EVOLUTION

### 9.1 Content Updates
- **Re-indexing**: When textbook content changes, re-embed affected chapters
- **Versioning**: Track textbook version in Qdrant metadata
- **Monitoring**: Alert if retrieval quality drops (likely due to stale embeddings)

### 9.2 Model Upgrades
- **OpenAI Model Migration**: Plan for GPT-5 or future models
- **Cost Optimization**: Monitor token usage, implement caching for common queries
- **A/B Testing**: Compare response quality across model versions

---

## 10. DEPLOYMENT CHECKLIST

### 10.1 Pre-Launch
- [ ] Qdrant collection created and populated with all chapters
- [ ] Neon Postgres database schema deployed
- [ ] FastAPI backend tested on staging environment
- [ ] OpenAI API keys secured in environment variables
- [ ] CORS configured for textbook domain
- [ ] Rate limiting implemented (per user/session)
- [ ] Error handling for API failures (graceful degradation)

### 10.2 Post-Launch
- [ ] Monitor logs for errors and slow queries
- [ ] Collect user feedback in first week
- [ ] Iterate on prompt templates based on response quality
- [ ] Publish usage analytics dashboard
- [ ] Document API for potential future integrations

---

## 11. SUCCESS CRITERIA

### 11.1 Functional Success
✅ Chatbot embedded in all textbook chapters  
✅ Both general and selected-text query modes operational  
✅ < 5% error rate in production  
✅ Proper citations in 100% of responses  

### 11.2 Educational Success
✅ Positive user feedback (> 70% thumbs up)  
✅ Students report improved understanding (post-survey)  
✅ High engagement (> 30% of visitors interact with chatbot)  

### 11.3 Technical Success
✅ Average response time < 2.5 seconds  
✅ Zero data breaches or privacy violations  
✅ Scalable to 1000+ daily active users  

---

## APPENDIX A: Example Interactions

### Example 1: General Query
**User**: "What is the difference between Gazebo and Isaac Sim?"

**Bot Response**:
> Gazebo and Isaac Sim are both physics simulation environments, but they serve different purposes in the Physical AI course:
> 
> **Gazebo** [Chapter 2, Module 2]:
> - Open-source robot simulator
> - Physics simulation with gravity and collisions
> - Standard ROS 2 integration
> - Lighter computational requirements
> 
> **NVIDIA Isaac Sim** [Chapter 2, Module 3]:
> - Photorealistic simulation using Omniverse
> - Advanced perception and synthetic data generation
> - Requires RTX-enabled GPU
> - Hardware-accelerated VSLAM capabilities
> 
> For this course, you'll use **Gazebo** for basic robot control (Weeks 6-7) and **Isaac Sim** for AI-powered perception training (Weeks 8-10).

---


### Example 2: Selected Text Query
**Selected Text**:
> "NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB). Role: This is the industry standard for embodied AI."

**User**: "Why is this the industry standard?"

**Bot Response**:
> Based on the selected text, the Jetson Orin Nano/NX is described as the "industry standard for embodied AI" because it allows students to deploy ROS 2 nodes and understand resource constraints compared to powerful workstations [Chapter 4, Hardware Requirements Section].
>
> The textbook explains that this hardware serves as "The Brain" in the Physical AI Edge Kit, bridging the gap between simulation and physical deployment. However, the a specific technical reasons (processing power, power efficiency, AI acceleration) are not elaborated in this excerpt.
>
> Would you like me to search the broader textbook for more details on Jetson specifications?

---


## REVISION HISTORY

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Dec 2025 | Initial constitution created |

---

## APPROVAL

**Document Owner**: Hackathon Team  
**Technical Review**: [Pending]  
**Final Approval**: [Pending]

---

*This constitution serves as the authoritative specification for RAG chatbot development. All implementation decisions should align with these guidelines.*

now create a now file in this path C:\new\.specify\memory rag-chatbot-constitution.md and this constitution into my this file.

## Response snapshot

Created the new constitution file for the RAG Chatbot at .specify/memory/rag-chatbot-constitution.md.

## Outcome

- ✅ Impact: High - Establishes the foundational principles and specifications for the RAG Chatbot feature.
- 🧪 Tests: N/A
- 📁 Files: 1 new file created.
- 🔁 Next prompts: Review and approve the new constitution.
- 🧠 Reflection: The constitution provides a comprehensive specification for the chatbot, which will guide development. Creating the PHR ensures this significant action is recorded.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (N/A)
