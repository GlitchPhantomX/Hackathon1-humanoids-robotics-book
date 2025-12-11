--- 
id: 0002
title: Initial constitution for Physical AI & Robotics Textbook
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: user
command: /sp.constitution Constitution for “Physical AI & Humanoid Robotics” Textbook + Embedded RAG Chatbot
labels: ["constitution", "rag", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- C:\new\.specify\memory\constitution.md
tests:
- N/A
---

## Prompt

This is the Gemini CLI. We are setting up the context for our chat.
Today's date is Friday, December 5, 2025 (formatted according to the user's locale).
My operating system is: win32
The project's temporary directory is: C:\Users\HP\.gemini\tmp\1af8a3ab15273b5e7157d1dd9e60d913dcd9cd62f126c42b17b84f17285d6b54
I'm currently working in the directory: C:\new
Here is the folder structure of the current working directories:

Showing up to 200 items (files + folders). Folders or files indicated with ... contain more items not shown, were ignored, or the display limit (200 items) was reached.

C:\new\
├───CLAUDE.md
├───FIXES_APPLIED.md
├───GEMINI.md
├───nul
├───package-lock.json
├───.claude\
│   └───commands\
│       ├───sp.adr.md
│       ├───sp.analyze.md
│       ├───sp.checklist.md
│       ├───sp.clarify.md
│       ├───sp.constitution.md
│       ├───sp.git.commit_pr.md
│       ├───sp.implement.md
│       ├───sp.phr.md
│       ├───sp.plan.md
│       ├───sp.specify.md
│       └───sp.tasks.md
├───.gemini\
│   └───commands\
│       ├───sp.adr.toml
│       ├───sp.analyze.toml
│       ├───sp.checklist.toml
│       ├───sp.clarify.toml
│       ├───sp.constitution.toml
│       ├───sp.git.commit_pr.toml
│       ├───sp.implement.toml
│       ├───sp.phr.toml
│       ├───sp.plan.toml
│       ├───sp.specify.toml
│       └───sp.tasks.toml
├───.git…
├───.specify\
│   ├───memory\
│   │   ├───constitution.md
│   │   └───rag-constitution.md
│   ├───scripts\
│   │   └───powershell\
│   │       ├───check-prerequisites.ps1
│   │       ├───common.ps1
│   │       ├───create-new-feature.ps1
│   │       ├───setup-plan.ps1
│   │       └───update-agent-context.ps1
│   └───templates\
│       ├───adr-template.md
│       ├───agent-file-template.md
│       ├───checklist-template.md
│       ├───phr-template.prompt.md
│       ├───plan-template.md
│       ├───spec-template.md
│       └───tasks-template.md
├───history\
│   └───prompts\
│       ├───0001-robotics-textbook-spec\
│       │   └───003-phase-5-and-6-implementation.green.prompt.md
│       ├───0002-rag-chatbot-ui\
│       │   ├───001-create-rag-chatbot-ui-specification.spec.prompt.md
│       │   ├───002-plan-rag-chatbot-ui.plan.prompt.md
│       │   ├───003-create-tasks-for-rag-chatbot-ui.tasks.prompt.md
│       │   ├───004-implement-phase1-rag-chatbot-ui.green.prompt.md
│       │   ├───005-implement-phase2-docusaurus-integration.green.prompt.md
│       │   ├───006-implement-phase3-phase4-complete-ui.green.prompt.md
│       │   └───007-verify-docusaurus-integration.green.prompt.md
│       ├───002-rag-chatbot-ui\
│       │   └───001-implement-phase-7-final-polish.green.prompt.md
│       ├───constitution\
│       │   └───0001-update-rag-chatbot-constitution.constitution.prompt.md
│       ├───general\
│       │   ├───0001-implement-rag-pipeline-for-chatbot.general.prompt.md
│       │   ├───0001-initial-cli-setup-context.general.prompt.md
│       │   └───0003-implement-complete-rag-backend-structure.general.prompt.md
│       ├───physical-ai-robotics-textbook\
│       │   ├───0001-update-00-introduction-chapters.green.prompt.md
│       │   └───0002-update-01-foundations-chapters.green.prompt.md
│       └───rag-chatbot-integration\
│           ├───0002-integrate-rag-chatbot-into-docusaurus.spec.prompt.md
│           ├───0003-implement-basic-fastapi-backend.green.prompt.md
│           ├───0004-setup-neon-serverless-postgres-for-chatbot.green.prompt.md
│           ├───0005-setup-qdrant-cloud-for-document-embeddings.green.prompt.md
│           ├───0006-implement-rag-with-openai-agents-chatkit.green.prompt.md
│           └───0007-create-text-selection-feature-for-docusaurus.green.prompt.md
├───physical-ai-robotics-textbook\
│   ├───package-lock.json
│   ├───package.json
│   ├───QUICK_START_RAG.md
│   ├───assessments\
│   │   └───projects\
│   ├───code-examples\
│   │   ├───capstone\
│   │   ├───module-01-foundations\
│   │   ├───module-02-ros2\
│   │   ├───module-03-simulation\
│   │   └───module-05-vla\
│   └───docusaurus\
│       ├───docusaurus.config.mjs
│       ├───global.d.ts
│       ├───package-lock.json
│       ├───package.json
│       ├───postcss.config.js
│       ├───tailwind.config.js
│       ├───.docusaurus\
│       │   ├───client-manifest.json
│       │   ├───client-modules.js
│       │   ├───codeTranslations.json
│       │   ├───docusaurus.config.mjs
│       │   ├───DONT-EDIT-THIS-FOLDER
│       │   ├───globalData.json
│       │   ├───i18n.json
│       │   ├───registry.js
│       │   ├───routes.js
│       │   ├───routesChunkNames.json
│       │   ├───site-metadata.json
│       │   ├───site-storage.json
│       │   ├───docusaurus-plugin-content-blog\
│       │   │   └───default\
│       │   ├───docusaurus-plugin-content-docs\
│       │   │   └───default\
│       │   ├───docusaurus-plugin-content-pages\
│       │   │   └───default\
│       │   ├───docusaurus-plugin-css-cascade-layers\
│       │   │   └───default\
│       │   ├───docusaurus-plugin-debug\
│       │   │   └───default\
│       │   └───docusaurus-theme-search-algolia\
│       │       └───default\
│       ├───build\
│       │   ├───404.html
│       │   ├───__server\
│       │   │   ├───server.bundle.js
│       │   │   └───assets\
│       │   ├───assets\
│       │   │   ├───css\
│       │   │   └───js\
│       │   ├───css\
│       │   │   └───rag-chatbot-ui.css
│       │   ├───docs\
│       │   │   ├───capstone\
│       │   │   ├───foundations\
│       │   │   ├───introduction\
│       │   │   ├───isaac\
│       │   │   ├───ros2\
│       │   │   ├───simulation\
│       │   │   └───vla\
│       │   ├───js\
│       │   │   └───rag-chatbot-ui.es.js
│       │   └───search\
│       │       └───index.html
│       ├───docs\
│       │   ├───00-introduction\
│       │   │   ├───faq.md
│       │   │   ├───native-macos.md
│       │   │   ├───native-ubuntu.md
│       │   │   ├───native-windows.md
│       │   │   ├───support.md
│       │   │   ├───syllabus.md
│       │   │   └───tools.md
│       │   ├───01-foundations\
│       │   │   ├───02-linear-algebra.md
│       │   │   ├───03-computer-vision.md
│       │   │   ├───04-machine-learning.md
│       │   │   └───05-integration-lab.md
│       │   ├───02-ros2\
│       │   │   ├───01-architecture.md
│       │   │   ├───02-nodes-topics.md
│       │   │   ├───03-services-actions.md
│       │   │   ├───04-parameters-launch.md
│       │   │   ├───05-tools-debugging.md
│       │   │   ├───06-robot-application.md
│       │   │   └───index.md
│       │   ├───03-simulation\
│       │   │   ├───01-gazebo-intro.md
│       │   │   ├───02-urdf-sdf.md
│       │   │   └───03-sensors-plugins.md
│       │   │   └───...
│       │   ├───04-isaac\
│       │   ├───05-vla\
│       │   └───06-capstone\
│       ├───node_modules…
│       ├───src\
│       │   ├───components\
│       │   ├───css\
│       │   ├───lib\
│       │   ├───pages\
│       │   ├───styles\
│       │   ├───theme\
│       │   ├───ui\
│       │   └───utils\
│       └───static\
│           ├───css\
│           └───js
└───specs\
    ├───0002-rag-chatbot-ui\
    │   ├───spec.md
│   └───checklists\
│       └───requirements.md
└───002-rag-chatbot-ui\
    ├───nul
    ├───plan.md
    ├───research.md
    ├───spec.md
    └───tasks.md

Reminder: Do not return an empty response when a tool call is required.

My setup is complete. I will provide my first command in the next turn.--- 
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


/sp.constitution Constitution for “Physical AI & Humanoid Robotics” Textbook + Embedded RAG Chatbot
1. Purpose & guiding principles

Purpose. Produce a deployed, public textbook website (Docusaurus) that teaches Physical AI & Humanoid Robotics, and ship an integrated, embedded RAG chatbot inside that site that:

Answers questions about the book contents.

Can be constrained to answer only from user-selected text (user selection → “source-only” mode).

Uses Gemini API for model calls (you will use GEMINI_API_KEY).

Uses Spec-Kit Plus to organize/specify agentic workflows and Claude Code for code authoring support.

Uses open vector DB (Qdrant Cloud free tier) and Neon Serverless Postgres for app state & logging.

Uses FastAPI as backend and (optionally) OpenAI ChatKit/Agents SDK patterns where possible — but only if OpenAI APIs are available; otherwise orchestrate agents with Gemini-driven flows and Spec-Kit Plus templates. (Rationale and options below.)

Principles.

Reproducible and versioned: everything in Git, CI builds and deploys to GitHub Pages.

Privacy-aware: user-selected text used as “source-only” context; selection should not be sent outside the user’s explicit action.

Minimal external cost: use free tiers (Qdrant free cluster, Neon free tier), avoid paid OpenAI keys (you’ll use Gemini), and keep compute/usage efficient by caching embeddings and results.

Testable: unit tests for ingestion/embedding/retrieval; E2E tests for chatbot behavior.

Audit & guardrails: store minimal logs, provide opt-in telemetry, and include rate-limits & content filters.

2. High-level architecture (textual)

Frontend (Docusaurus site, React)

Book content pages (markdown) created with Docusaurus.

Embedded chatbot React widget (custom or wrapped ChatKit-like UI) that calls your FastAPI backend.

UI supports selecting text on any page and clicking “Ask from selection” to force source-only RAG queries.

Backend (FastAPI)

Endpoints:

/api/embeddings — request embeddings for content or selections.

/api/index — ingest content into Qdrant.

/api/query — run RAG queries; supports mode=source-only (use only user-selected chunks) and mode=global (book-wide retrieval).

/api/chat — session management & conversation history (stored in Neon).

/api/admin/* — admin tasks (reindex, health, metrics).

Orchestrates: embedding calls (Gemini embeddings API), vector search (Qdrant cloud), generative calls (Gemini text completion API), optional agent orchestration (OpenAI/ChatKit/Agents patterns if available).

Vector DB (Qdrant Cloud Free Tier)

Stores embeddings of book chunks and metadata (page, section, chunk-id, text, hash).

Relational DB (Neon Serverless Postgres)

Stores user sessions, conversation history pointers, logs, user opt-ins, indexing state.

CI / Deployment

GitHub Actions builds site, runs tests, and deploys the built Docusaurus site to GitHub Pages.

Backend (FastAPI) can run on a small serverless host (Vercel serverless functions, Fly, Render, or self-host). If you prefer serverless Postgres (Neon) and free tiers, consider deploying FastAPI to a small VM or a serverless container provider.

Auth & Secrets

Environment variables in GitHub Actions / deployment: GEMINI_API_KEY, QDRANT_API_KEY (if required), NEON_DATABASE_URL, and any other secrets.

Local dev: use .env with example .env.example.

Diagram (one-liner): Docusaurus frontend (React) ⇄ FastAPI backend ⇄ (Gemini APIs for embeddings + generation) & (Qdrant for vector search) & (Neon Postgres for state/logs).

Authoring assistance: Spec-Kit Plus will be used to scaffold agentic content generation and Claude Code as a coding assistant when you want interactive code edits during authoring. See Spec-Kit Plus repo and Claude Code product pages. 
GitHub
+1

3. Key tool references (authoritative)

Spec-Kit Plus (scaffolding / agent patterns): GitHub repo. 
GitHub
+1

Claude Code (agentic coding assistant from Anthropic) — useful for editing code locally. 
Claude
+1

Docusaurus deployment / docusaurus deploy (GitHub Pages): official docs. 
Docusaurus

Qdrant Cloud free tier (1GB free cluster). 
Qdrant

Neon Serverless Postgres — serverless Postgres hosting. 
Neon

FastAPI — backend web framework. 
FastAPI
+1

Gemini API key & Gemini CLI docs (authentication & CLI): official docs & gemini-cli repo. 
Google AI for Developers
+2
Google Cloud Documentation
+2

OpenAI Agents & ChatKit (if you later choose to integrate with OpenAI tooling / patterns): official docs and examples (FastAPI + React examples). 
OpenAI Platform
+2
OpenAI Platform
+2

4. Important constraints & decisions (explicit)

Model provider: You will use Gemini for embeddings and generation (you stated no OpenAI paid key). Gemini docs show how to get API keys and authenticate. I will assume Gemini supports embeddings + completion generation in your plan — confirm via the Gemini docs for exact endpoints/SDKs. 
Google AI for Developers
+1

Spec-Kit Plus + Claude Code: Use Spec-Kit Plus to formalize your specification-driven workflows and templates; use Claude Code interactively for complex code edits if you have access. These are authoring tools and don't necessarily run in production. 
GitHub
+1

OpenAI Agents/ChatKit usage: You mentioned wanting OpenAI Agents/ChatKit SDKs. You can adopt their architecture patterns (agents, handoffs, ChatKit UI) using OpenAI docs as references and replicate similar behavior with Gemini model calls. If you want to actually use ChatKit/Agents SDKs you will need OpenAI credentials; otherwise implement the same patterns with your own orchestration layer. (Docs & examples are available.) 
OpenAI GitHub
+1

“Answer only from selected text” requirement: This must be implemented in the RAG orchestration: when user selects text and requests “source-only” answers, the backend must:

Embed only the selected text (or map selection → chunk ids), force retrieval to only those chunk(s) (i.e., bypass global search), and pass a strict instruction to the model: “Answer only using the following source text and say ‘I don’t know’ if the answer cannot be derived from it.” Log the selection id & hash to Neon for audit. (I’ll include exact prompt patterns below.)

Data residency & privacy: If any content is sensitive, warn users that selection will be sent to third-party APIs (Gemini & Qdrant). Provide an explicit opt-in toggle for sending content off-site.

5. Project folder layout (suggested)

Put this structure into C:\new\physical-ai-robotics-textbook:

physical-ai-robotics-textbook/
├─ docs/                       # Docusaurus markdown book content (chapters)
│  ├─ chapter-01-intro.md
│  └─ ... 
├─ website/                    # Docusaurus config (or use `docusaurus` root)
│  ├─ docusaurus.config.js
│  ├─ src/
│  │  ├─ components/
│  │  │  └─ ChatbotWidget/     # React Chatbot widget to embed in pages
│  │  │     └─ index.tsx
│  │  └─ css/
│  └─ package.json
├─ backend/
│  ├─ app/
│  │  ├─ main.py               # FastAPI app
│  │  ├─ api/
│  │  │  ├─ embeddings.py
│  │  │  ├─ indexer.py
│  │  │  └─ chat.py
│  │  └─ services/
│  │     ├─ gemini_client.py   # wrapper for Gemini APIs
│  │     ├─ qdrant_client.py
│  │     └─ neon_db.py
│  ├─ tests/
│  ├─ requirements.txt
│  └─ Dockerfile
├─ spec-kit/                    # spec-kit-plus scaffolding, templates, ADRs
├─ scripts/
│  ├─ ingest_site_content.py    # extract markdown -> chunks -> embeddings -> qdrant
│  └─ deploy_pages.sh
├─ .github/
│  └─ workflows/
│     ├─ ci.yml
│     └─ deploy-gh-pages.yml
├─ README.md
└─ .env.example

6. Implementation details — ingestion + embeddings + RAG
6.1 Chunking & metadata

Chunk length: ~500 tokens recommended (adjust to 200–800). Overlap: 50–150 tokens. Use sentence-boundary-aware chunker.

Metadata per chunk:

doc_id, page_path, chapter, chunk_id, start_offset, end_offset, text_hash.

Save chunk source as reference for “source-only” answers.
6.2 Embeddings

Provider: Gemini Embeddings endpoint (authenticate using GEMINI_API_KEY). See Gemini docs for exact endpoint and SDK usage. Cache embeddings (e.g., in a simple Redis or local file) to avoid re-embedding on each deploy. 
Google AI for Developers

6.3 Vector DB

Qdrant Cloud free tier (1GB cluster) for vectors & metadata. Store vector + metadata. Use their API/SDK. Free tier is available and suitable for textbook-size corpus. 
Qdrant

6.4 Retrieval

Two modes:

Global retrieval: normal RAG search across the entire book (top-k nearest neighbors, k=5–10).

Source-only retrieval: when user selects text, only that selection’s chunk(s) are considered (by querying Qdrant with the exact chunk_id or by embedding the selection and doing retrieval but constraining to the set of chunk ids). Then pass only those chunks into the generative prompt; set max_returned_chunks=... to limit context.

6.5 Prompting & generation

Always include an instruction block that enforces constraints:

Example safe instruction for source-only mode:

You are a helpful textbook assistant. You must **only** use the following source text. Do not use outside knowledge. If the answer cannot be found in the sources, respond: "I don't know — the selected text does not contain enough information."
========= SOURCE =========
{concatenated_chunks}
========= END SOURCE =========
QUESTION: {user_question}
ANSWER:


For global mode you may allow models to use retrieved chunks + general knowledge, but still include citations (chunk ids & page path) in the answer.

6.6 Handling hallucinations & safety

Add chain-of-trust metadata to answers: show which chunks were used (page & snippet) and highlight exact lines used to answer.

If the model response claims facts not present in the used chunks, the system should automatically flag/mark the answer as “possibly unsupported” and, optionally, re-run a stricter source-only pass.

7. FastAPI endpoints (suggested)

POST /api/index — body: { "source": "markdown", "path": "/docs/chapter-1", "content": "..." } → splits, embeds, stores in Qdrant.

POST /api/query — body: { "question": "...", "mode": "global"|"source-only", "source_chunk_ids": [..], "session_id": "..." } → returns { answer, used_chunks, citations }.

POST /api/embeddings — wrapper to create embeddings (for selected text).

GET /api/health — health checks for Qdrant/Neon/Gemini connectivity.

POST /api/chat — store conversation events; optional for ChatKit-like session.

8. Docusaurus + Chatbot embedding

Add a React component ChatbotWidget under website/src/components/ChatbotWidget. Expose it as a floating widget or per-page widget.

Add a UI affordance (a small toolbar) to allow users to select text and click Ask selection. Implementation approach:

Use browser selection APIs to capture window.getSelection() and send the selected text to POST /api/embeddings then POST /api/query with mode=source-only.

Alternatively, annotate the page with chunk IDs at build-time (using scripts that chunk markdown and add anchors); then selection can map to chunk IDs.

Important: The Docusaurus pages are static; the easiest approach is to include chunk metadata (chunk id ranges) inside the built page when you run the ingestion scripts. The ingestion script should be run whenever content updates.

Docusaurus deployment doc reference. 
Docusaurus

9. CI/CD & deployment (GitHub Pages + backend)

Site (Docusaurus)

Add GitHub Actions workflow deploy-gh-pages.yml that runs:

npm ci

npm run build

npx docusaurus deploy (or use gh-pages action) to push to gh-pages branch. Docusaurus docs show docusaurus deploy. 
Docusaurus

Backend

Deploy FastAPI to your preferred host (Render, Fly, or Docker on a VM). Use GitHub Actions to build & push container image.

Store secrets as GitHub Actions secrets / host environment variables: GEMINI_API_KEY, QDRANT_API_KEY, NEON_DATABASE_URL, and any other secrets.

10. Spec-Kit Plus & Claude Code usage (authoring)

Use Spec-Kit Plus templates to model multi-agent authoring pipelines:

Example pipeline: Draft chapter → Auto-outline agent (Spec-Kit template) → Expand agent (Gemini) → Human-in-the-loop review → Commit to docs/.

Use Claude Code locally to assist code editing as needed (it is a developer tool for code within Anthropic product). If you plan to rely heavily on agentic edits to code, keep human review gates. 
GitHub
+1

11. Prompts, evaluation & testing

Unit tests: test chunker determinism, embedding shape, indexing & retrieval.

Behavioral tests (E2E):

Ask the chatbot a question that is definitely answered in chapter text — ensure answer matches.

Ask the chatbot a question not in the book and verify it replies “I don’t know” in source-only mode.

Automated evals: use a small set of gold Q/A pairs per chapter to track model drift/hallucination.

Privacy test: select a phrase containing PII and confirm opt-in behavior is enforced.

12. Example prompts & enforcement patterns

Source-only strict wrapper (send this verbatim to the model):

SYSTEM: You are a strict textbook assistant. You MUST answer ONLY using the provided SOURCE block. You may not hallucinate or add facts outside the SOURCE. If the information is missing, respond exactly: "I don't know — the selected text does not contain enough information."

USER:
======== SOURCE START ========
{chunk_1}
{chunk_2}
...
======== SOURCE END =========

QUESTION:
{user_question}

ANSWER:


Global mode (allow general knowledge but list citations): 

SYSTEM: Use the retrieved passages to answer. Base your answer on the sources; indicate the chunk id(s) used as citations inline. If you rely on outside knowledge, label it clearly as “background”.
...

13. Security, costs, & privacy considerations

Cost control: cache embeddings; tune retrieval thresholds; set limits on max tokens returned by generation; set rate limits in backend.

Secrets: never commit keys. Use GitHub Secrets. Use GEMINI_API_KEY (Gemini docs) for authentication. 
Google AI for Developers

User data: store only URLs + minimal logs unless user opts into saving full transcripts.

Vulnerability scanning: run static analysis and dependency scanning in CI (dependabot, Snyk).

Claude Code caution: be careful running agentic tools that execute arbitrary code in your environment; always run in sandboxed containers. 

14. Roadmap & milestones (suggested)

Milestone 0 — Setup & scaffolding

Initialize Docusaurus site, add sample chapter, Git repo. Add .env.example.

Add Spec-Kit Plus templates into /spec-kit.

Milestone 1 — Local ingestion & indexing

Build chunker + embedding wrapper (Gemini).

Deploy Qdrant free cluster and index a small sample chapter.

Milestone 2 — Backend & minimal chat

Implement FastAPI endpoints for query & embeddings.

Build simple React chat widget in Docusaurus that calls backend.

Milestone 3 — Source-only selection feature

Implement page selection capture & map to chunk ids or embed selected text on the fly.

Add strict source-only prompt wrapper.

Milestone 4 — CI/CD & deploy

Add GitHub Actions for site build + docusaurus deploy.

Deploy backend to chosen host.

Milestone 5 — Tests & hardening

Add automated tests, rate limit, monitoring, and cost controls.

15. Example commands & config (practical)

Gemini CLI auth (local dev)

# windows powershell
setx GEMINI_API_KEY "your_key_here"
# or in a .env file:
GEMINI_API_KEY=your_key_here


(See Gemini cli docs for details.) 
GitHub
+1

Docusaurus deploy (build + deploy)

npm ci
npm run build
npx docusaurus deploy


(Docusaurus docs describe the deploy command for GitHub Pages.) 
Docusaurus

Qdrant — start a free cluster

Create Qdrant Cloud cluster from dashboard; use API key in backend. Qdrant has a free 1GB cluster. 
Qdrant

Neon — Get DB URL

Create Neon project & copy NEON_DATABASE_URL to secrets. 
Neon

16. When to use OpenAI Agents/ChatKit vs Gemini-native approach

If you obtain OpenAI credentials later: you can directly use OpenAI Agents SDK / ChatKit for advanced agent orchestration and embedded widgets; OpenAI provides examples that pair FastAPI + React for ChatKit widgets. This is a developer-time choice and can be adopted later. 
OpenAI Platform
+1

If you do not have OpenAI credentials: build the same agent orchestration in your FastAPI backend using Gemini model calls and Spec-Kit Plus templates. Use the OpenAI docs as architectural references but implement calls against Gemini endpoints.

17. Acceptance criteria (what “done” looks like)

The textbook site runs on GitHub Pages and displays all chapters. (Docusaurus built & deployed.) 
Docusaurus

The site includes a visible chat widget.

The chatbot answers questions using book content. For selected-text queries, the bot only uses the selection and responds “I don't know” when the selection lacks the answer.

Indexing pipeline exists (markdown → chunks → embeddings → Qdrant).

CI deploys the site; secrets are stored securely; Neon stores session data and logs.

Basic tests exist and pass (chunker, embed shape, source-only behavior).

18. Example heuristics & parameters (start points)

Chunk size: 500 tokens ±200; overlap: 100 tokens.

Embedding dimension: use provider default (Gemini embedding API docs).

Retrieval: top_k = 5.

Generation: max_tokens = 512 (tune per cost).

Relevance threshold (score): test empirically; start with top_k 5 and filter by distance.

19. Suggested next immediate steps (practical)

Create the repo at C:\new\physical-ai-robotics-textbook (if not already).

Add README.md with the Constitution content (this whole doc).

Scaffold Docusaurus (npx create-docusaurus@latest or your chosen scaffold) and add one sample chapter in docs/.

Run initial ingestion script to chunk & embed the sample chapter and index into a dev Qdrant cluster.

Spin up FastAPI skeleton with GET /api/health.

Implement selection-to-chunk mapping and a minimal chat widget that calls POST /api/query in source-only mode.

Acquire GEMINI_API_KEY and configure local .env (see Gemini docs). 
Google AI for Developers
+1

20. References & citations (authoritative docs I used)

Spec-Kit Plus (repo & releases). 
GitHub
+1

Claude Code (Anthropic product). 
Claude
+1

Docusaurus deployment docs. 
Docusaurus

Qdrant Cloud pricing / free tier. 
Qdrant

Neon Serverless Postgres. 
Neon

FastAPI docs & tutorial. 
FastAPI
+1

Gemini API key & Gemini CLI docs. 
Google AI for Developers
+2
Google Cloud Documentation
+2

OpenAI Agents & ChatKit docs and examples (architectural reference). 
OpenAI Platform
+2
OpenAI Platform
+2

Final notes (explicit recommendations & gotchas)

Do not assume OpenAI SDKs will be usable without an OpenAI key — design your architecture so model-provider abstraction is pluggable (adapter pattern). If you later obtain an OpenAI key, swap the “model” adapter. I cited OpenAI examples for architecture, not to tell you you must use OpenAI. 
OpenAI GitHub

Beware Claude Code security: running agentic tools that execute arbitrary code in your environment; always run in sandboxed containers. 
Claude

Testing source-only: make this a first-class test so you can prove the claim that the bot can only answer from selection.
