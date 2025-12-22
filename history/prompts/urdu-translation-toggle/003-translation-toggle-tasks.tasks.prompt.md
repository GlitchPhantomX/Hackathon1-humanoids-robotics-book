---
id: 003
title: Translation Toggle Tasks
stage: tasks
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: urdu-translation-toggle
branch: 01-urdu-translation-toggle
user: Claude Code User
command: /sp.tasks
labels: ["translation", "tasks", "implementation", "frontend", "backend"]
links:
  spec: specs/01-urdu-translation-toggle/spec.md
  plan: specs/01-urdu-translation-toggle/plan.md
  tasks: specs/01-urdu-translation-toggle/tasks.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/01-urdu-translation-toggle/tasks.md
tests:
 - specs/01-urdu-translation-toggle/tasks.md
---

## Prompt

$ARGUMENTS

## Response snapshot

# Tasks: Urdu Translation Toggle

**Feature**: Urdu Translation Toggle
**Branch**: `01-urdu-translation-toggle`
**Created**: 2025-12-19
**Input**: User stories from spec.md, technical design from plan.md, data model, API contracts, and research

## Implementation Strategy

Build incrementally following user story priorities. Start with core functionality (US1), then add security (US2), then performance (US3). Each user story should be independently testable.

**MVP Scope**: US1 (core translation functionality) with basic authentication check and simple caching.

## Dependencies

- US2 (Authentication) must be partially complete before US1 can be fully tested in production
- US3 (Caching) depends on US1 (Translation Service) being implemented
- Frontend components depend on backend API being available

## Parallel Execution Opportunities

- Frontend component development can run in parallel with backend API development
- Caching implementation can be done in parallel with translation service
- Multiple CSS styling tasks can be done in parallel

## Phase 1: Setup

- [ ] T001 Create frontend directory structure: docusaurus/src/components/translation/
- [ ] T002 Create backend directory structure: physical-ai/translation/
- [ ] T003 Initialize backend requirements file with FastAPI and dependencies
- [ ] T004 Create prompts directory and add en_to_ur.txt translation prompt template

## Phase 2: Foundational

- [ ] T005 [P] Create TranslationRequest schema in physical-ai/translation/schemas.py
- [ ] T006 [P] Create TranslationResponse schema in physical-ai/translation/schemas.py
- [ ] T007 [P] Create CacheEntry schema in physical-ai/translation/schemas.py
- [ ] T008 [P] Create UserSession schema in physical-ai/translation/schemas.py
- [ ] T009 [P] Create basic cache implementation in physical-ai/translation/cache.py
- [ ] T010 [P] Create markdown parsing utility to preserve structure

## Phase 3: User Story 1 - Translate Chapter Content (Priority: P1)

**Goal**: Enable logged-in users to translate chapter content from English to Urdu with RTL layout

**Independent Test**: Can be fully tested by logging in, opening a chapter, clicking the EN → اردو toggle button, and verifying that the content appears in fluent Urdu with RTL layout.

**Acceptance Scenarios**:
1. Given user is logged in and viewing an English chapter, When user clicks the "EN → اردو" toggle button, Then the entire chapter content appears in Urdu with RTL layout and the button changes to "اردو → EN"
2. Given user has translated a chapter to Urdu, When user clicks the "اردو → EN" toggle button, Then the chapter content reverts to English with LTR layout

- [ ] T011 [US1] Create translation service in physical-ai/translation/service.py
- [ ] T012 [P] [US1] Create LLM integration for Urdu translation in physical-ai/translation/service.py
- [ ] T013 [P] [US1] Implement markdown structure preservation in translation service
- [ ] T014 [P] [US1] Create API endpoint POST /api/translate in physical-ai/translation/router.py
- [ ] T015 [P] [US1] Add authentication validation to translation endpoint
- [ ] T016 [P] [US1] Create LanguageToggle React component in docusaurus/src/components/translation/LanguageToggle.tsx
- [ ] T017 [P] [US1] Create TranslationButton React component in docusaurus/src/components/translation/TranslationButton.tsx
- [ ] T018 [P] [US1] Implement RTL CSS styles in docusaurus/src/components/translation/rtlStyles.css
- [ ] T019 [P] [US1] Create useTranslation hook in docusaurus/src/components/translation/useTranslation.ts
- [ ] T020 [US1] Integrate frontend components with backend API
- [ ] T021 [US1] Test complete translation flow from frontend to backend

## Phase 4: User Story 2 - Authenticated Access Control (Priority: P2)

**Goal**: Ensure translation functionality is only available to authenticated users

**Independent Test**: Can be tested by attempting to access the translation toggle as both an authenticated and non-authenticated user, verifying that it's only available when logged in.

**Acceptance Scenarios**:
1. Given user is not logged in, When user views a chapter page, Then the translation toggle button is hidden or disabled
2. Given user is logged in, When user views a chapter page, Then the translation toggle button is visible and functional

- [ ] T022 [US2] Implement session validation utility in physical-ai/translation/service.py
- [ ] T023 [US2] Enhance authentication middleware for translation endpoint
- [ ] T024 [P] [US2] Update frontend to detect authentication status
- [ ] T025 [P] [US2] Implement conditional rendering of toggle button based on auth status
- [ ] T026 [US2] Test authentication flow with authenticated and non-authenticated users

## Phase 5: User Story 3 - Performance and Caching (Priority: P3)

**Goal**: Ensure subsequent translations are fast through caching

**Independent Test**: Can be tested by translating a chapter, then toggling back and forth between languages, verifying that subsequent translations are returned quickly.

**Acceptance Scenarios**:
1. Given user has previously translated a chapter, When user translates the same chapter again, Then the cached translation is returned instantly
2. Given user is translating a chapter for the first time, When user clicks the toggle, Then the translation is generated within 2-4 seconds

- [ ] T027 [US3] Enhance cache implementation with TTL support in physical-ai/translation/cache.py
- [ ] T028 [US3] Integrate caching with translation service
- [ ] T029 [US3] Add cache key generation using user_id + chapter_id + target_language
- [ ] T030 [US3] Implement cache expiration logic (24-hour TTL)
- [ ] T031 [US3] Test cache hit/miss scenarios
- [ ] T032 [US3] Measure performance against 200ms cached response requirement

## Phase 6: Edge Cases & Error Handling

- [ ] T033 [P] Handle AI translation service failure gracefully
- [ ] T034 [P] Implement error notification when service is unavailable
- [ ] T035 Handle large chapter content that exceeds API limits
- [ ] T036 Handle session expiration during translation request
- [ ] T037 Handle malformed markdown content
- [ ] T038 Preserve original content when errors occur

## Phase 7: Polish & Cross-Cutting Concerns

- [ ] T039 Add loading states to frontend components during translation
- [ ] T040 Implement proper RTL support for code blocks (keep LTR)
- [ ] T041 Add proper ARIA labels for accessibility
- [ ] T042 Write unit tests for backend services
- [ ] T043 Write unit tests for frontend components
- [ ] T044 Update documentation with usage instructions
- [ ] T045 Perform end-to-end testing of complete workflow
- [ ] T046 Verify 95% markdown structure preservation requirement
- [ ] T047 Verify 4-second first translation performance requirement

## Outcome

- ✅ Impact: Created comprehensive task list for Urdu translation toggle feature with 47 specific tasks
- 🧪 Tests: Tasks organized by user stories with independent test criteria
- 📁 Files: Created tasks.md with proper checklist format and file paths
- 🔁 Next prompts: Ready for implementation with /sp.implement
- 🧠 Reflection: Tasks follow priority order from spec and align with architecture from plan

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All tasks follow required checklist format
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation with Phase 1 setup tasks