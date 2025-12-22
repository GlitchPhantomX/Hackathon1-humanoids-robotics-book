# Tasks: Urdu Translation Toggle

**Feature**: Urdu Translation Toggle
**Branch**: `01-urdu-translation-toggle`
**Created**: 2025-12-19
**Input**: User stories from spec.md, technical design from plan.md, data model, API contracts, and research

---

## Implementation Strategy

Build incrementally following user story priorities. Start with core functionality (US1), then add security (US2), then performance (US3). Each user story should be independently testable.

**MVP Scope**: US1 (core translation functionality) with basic authentication check and simple caching.

---

## Dependencies

* US2 (Authentication) must be partially complete before US1 can be fully tested in production
* US3 (Caching) depends on US1 (Translation Service) being implemented
* Frontend components depend on backend API being available

---

## Parallel Execution Opportunities

* Frontend component development can run in parallel with backend API development
* Caching implementation can be done in parallel with translation service
* CSS / RTL styling tasks can be done in parallel

---

## Phase 1: Setup

* [x] T001 Create frontend directory structure: `docusaurus/src/components/translation/`
* [x] T002 Create backend directory structure: `physical-ai-robotics-textbook/translation/`
* [x] T003 Initialize backend requirements file with FastAPI and dependencies
* [x] T004 Create prompts directory and add `en_to_ur.txt` translation prompt template
* [x] T004a Define translation quality rules in prompt (no code translation, preserve markdown)
* [x] T004b Define fallback behavior in prompt (return original English on failure)

---

## Phase 2: Foundational

* [x] T005 [P] Create `TranslationRequest` schema in `schemas.py`
* [x] T006 [P] Create `TranslationResponse` schema in `schemas.py`
* [x] T007 [P] Create `CacheEntry` schema in `schemas.py`
* [x] T008 [P] Create `UserSession` schema in `schemas.py`
* [x] T009 [P] Create basic cache implementation in `cache.py`
* [x] T010 [P] Create markdown parsing utility to preserve structure

---

## Phase 3: User Story 1 – Translate Chapter Content (Priority: P1)

**Goal**: Enable logged-in users to translate chapter content from English to Urdu with RTL layout

**Independent Test**: Login → open chapter → click EN → اردو → verify Urdu + RTL → toggle back

### Acceptance Scenarios

1. Given user is logged in and viewing an English chapter, when user clicks the toggle, then the entire chapter appears in Urdu with RTL layout and button switches to `اردو → EN`
2. Given Urdu is active, when user toggles back, then English content and LTR layout are restored

* [x] T011 [US1] Create translation service in `service.py`
* [x] T012 [P] [US1] Implement LLM integration for Urdu translation
* [x] T013 [P] [US1] Enforce markdown preservation rules in translation output
* [x] T014 [P] [US1] Create API endpoint `POST /api/translation/urdu`
* [x] T015 [P] [US1] Add authentication validation to translation endpoint
* [x] T016 [P] [US1] Create `LanguageToggle.tsx`
* [x] T017 [P] [US1] Create `TranslationButton.tsx`
* [x] T018 [P] [US1] Implement RTL styles (`rtl.css`) with LTR-safe code blocks
* [x] T019 [P] [US1] Create `useTranslation.ts` hook
* [x] T020 [US1] Integrate frontend with backend API
* [x] T021 [US1] Test full translation flow

---

## Phase 4: User Story 2 – Authenticated Access Control (Priority: P2)

**Goal**: Ensure translation is available only to authenticated users

### Acceptance Scenarios

1. Non-authenticated users do not see the toggle
2. Authenticated users see and can use the toggle

* [x] T022 [US2] Implement session validation utility
* [x] T023 [US2] Enforce auth middleware for translation endpoint
* [x] T024 [P] [US2] Detect authentication state on frontend
* [x] T025 [P] [US2] Conditionally render toggle button
* [x] T026 [US2] Test auth-based visibility and access

---

## Phase 5: User Story 3 – Performance & Caching (Priority: P3)

**Goal**: Ensure fast repeat translations using cache

### Acceptance Scenarios

1. Cached translation returns <200ms
2. First translation completes within 2–4 seconds

* [x] T027 [US3] Add TTL support to cache (24h)
* [x] T028 [US3] Integrate cache lookup before LLM call
* [x] T029 [US3] Implement cache key: `user_id + chapter_id + language`
* [x] T030 [US3] Implement cache expiration logic
* [x] T031 [US3] Test cache hit/miss behavior
* [x] T032 [US3] Benchmark performance requirements

---

## Phase 6: Edge Cases & Error Handling

* [x] T033 [P] Graceful fallback on translation service failure
* [x] T034 [P] User-facing error notification (non-blocking)
* [x] T035 Handle large chapters (chunking / pagination)
* [x] T036 Handle session expiration during request
* [x] T037 Safely handle malformed markdown
* [x] T038 Preserve original content on any failure
* [x] T038a Add logging for translation failures

---

## Phase 7: Polish, Quality & Compliance

* [x] T039 Add loading & disabled states to toggle button
* [x] T040 Ensure RTL correctness for lists, tables, and headings
* [x] T041 Add ARIA labels & keyboard accessibility
* [x] T042 Write backend unit tests
* [x] T043 Write frontend unit tests
* [x] T044 Update user & developer documentation
* [x] T045 Perform full end-to-end test
* [x] T046 Verify ≥95% markdown structure preservation
* [x] T047 Verify performance & UX requirements
