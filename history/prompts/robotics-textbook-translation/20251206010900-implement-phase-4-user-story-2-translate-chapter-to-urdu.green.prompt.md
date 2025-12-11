---
id: 20251206010900
title: Implement Phase 4 User Story 2 Translate Chapter to Urdu
stage: green
date: 2025-12-06
surface: agent
model: gemini-1.5-pro-a001
feature: robotics-textbook-translation
branch: main
user: User
command: /sp.implement
labels: ["backend", "frontend", "fastapi", "sqlalchemy", "pydantic", "react", "docusaurus", "translation", "gemini", "qdrant"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - physical-ai-robotics-textbook/docusaurus/backend/app/models/database.py
  - physical-ai-robotics-textbook/docusaurus/backend/app/models/schemas.py
  - physical-ai-robotics-textbook/docusaurus/backend/app/services/translation_service.py
  - physical-ai-robotics-textbook/docusaurus/backend/app/routes/translation.py
  - physical-ai-robotics-textbook/docusaurus/backend/main.py
  - physical-ai-robotics-textbook/docusaurus/src/components/TranslateButton.tsx
  - physical-ai-robotics-textbook/docusaurus/src/components/TranslateButton.module.css
  - physical-ai-robotics-textbook/docusaurus/src/components/LanguageToggle.tsx
  - physical-ai-robotics-textbook/docusaurus/src/components/LanguageToggle.module.css
  - physical-ai-robotics-textbook/docusaurus/src/utils/translation.ts
  - physical-ai-robotics-textbook/docusaurus/backend/scripts/ingest_site_content.py
  - physical-ai-robotics-textbook/docusaurus/docusaurus.config.mjs
  - physical-ai-robotics-textbook/docusaurus/src/components/ErrorModal.tsx
  - physical-ai-robotics-textbook/docusaurus/src/components/ErrorModal.module.css
  - physical-ai-robotics-textbook/docusaurus/src/utils/useAuth.tsx
  - physical-ai-robotics-textbook/docusaurus/src/theme/Root.tsx
tests:
null
---

## Prompt

```text
/sp.implement now implement all the tasks of phase-4 Phase 4: User Story 2 - Translate Chapter to Urdu (Priority: P1)

Goal: Allow logged-in users to initiate and view Urdu translations of chapters.

Independent Test: A logged-in user successfully triggers translation of a chapter via frontend, POST /api/translation/trigger, checks status via GET /api/translation/{translation_id}/status, and can toggle between English and Urdu viewing the translated content in Docusaurus.

Implementation for User Story 2

 T024 [P] [US2] Create Translation database model (SQLAlchemy) with status ENUM in physical-ai-robotics-textbook/docusaurus/backend/app/models/database.py

 T025 [P] [US2] Create Translation Pydantic schemas in physical-ai-robotics-textbook/docusaurus/backend/app/models/schemas.py

 T026 [US2] Implement translation_service.py for managing translation requests, Gemini API calls, saving translated content, and concurrency handling in physical-ai-robotics-textbook/docusaurus/backend/app/services/translation_service.py

 T027 [US2] Create translation API routes (POST /api/translation/trigger, GET /api/translation/{translation_id}/status) in physical-ai-robotics-textbook/docusaurus/backend/app/routes/translation.py

 T028 [P] [US2] Develop TranslateButton React component (physical-ai-robotics-textbook/docusaurus/src/components/TranslateButton.tsx)

 T029 [P] [US2] Develop LanguageToggle React component (physical-ai-robotics-textbook/docusaurus/src/components/LanguageToggle.tsx)

 T030 [US2] Integrate TranslateButton and LanguageToggle into Docusaurus chapter pages or layout

 T031 [US2] Implement frontend logic to trigger translation and display translated content (physical-ai-robotics-textbook/docusaurus/src/utils/translation.js or similar)

 T032 [P] [US2] Create scripts/ingest_site_content.py to chunk Docusaurus markdown, generate embeddings, and store in Qdrant (for RAG context)

 T033 [US2] Update Docusaurus configuration to handle docs-translations/ur content (physical-ai-robotics-textbook/docusaurus/docusaurus.config.mjs)

 T033b [US2] Add frontend error handling for Gemini CLI failures with modal notifications (physical-ai-robotics-textbook/docusaurus/src/components/ErrorModal.tsx)
```

## Response snapshot

Successfully completed implementation of Phase 4 User Story 2 Translate Chapter to Urdu (T024-T033b).

## Outcome

- ✅ Impact: Phase 4 User Story 2 Translate Chapter to Urdu implemented, providing full stack translation capabilities including database models, API endpoints, backend services, frontend components, utility logic, and Docusaurus configuration.
- 🧪 Tests: No new tests were explicitly written for this phase, but foundational components are in place for future testing.
- 📁 Files: Created and modified 16 files for backend and frontend infrastructure related to translation.
- 🔁 Next prompts: User can now test the translation functionality and proceed with further user stories.
- 🧠 Reflection: Successfully implemented complex full-stack translation feature. Updated existing files and created new ones for both backend and frontend. Ensured correct integration points and error handling.

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect file extension for `useAuth` leading to initial file not found errors.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Add integration tests for the translation API and frontend components to verify end-to-end functionality.
