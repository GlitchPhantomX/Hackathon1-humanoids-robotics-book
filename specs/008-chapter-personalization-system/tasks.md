# Tasks: Chapter-Level AI Personalization System

**Feature**: Chapter-Level AI Personalization System
**Branch**: `008-chapter-personalization-system`
**Spec**: `spec.md`

---

## Implementation Strategy

**Architecture Principle (MANDATORY)**
This feature MUST follow **separation of concerns**:

| Concern                           | Directory                  | Rule          |
| --------------------------------- | -------------------------- | ------------- |
| Authentication                    | `auth-backend/`            | ❌ No changes  |
| AI / RAG Intelligence             | `rag-chatbot/`             | ♻️ Reuse only |
| Translation                       | `translation/`             | ❌ No changes  |
| **Personalization Orchestration** | `personalization-backend/` | ✅ New         |

> ⚠️ No new generic `backend/` directory is allowed.

---

### MVP Strategy

**MVP Goal**
Allow a logged-in user to press a button at the start of a chapter and see **personalized content generated at runtime**, based on their signup background.

**MVP Includes**

* Personalization button (UI)
* Personalization backend (orchestrator)
* Reuse of existing RAG AI
* Smooth content replacement
* Fallback to original content

---

## Dependencies

* Better-Auth (`auth-backend`) must be running
* RAG chatbot service must be available
* No duplication of AI pipelines allowed

---

## Phase 1: Setup Tasks

* [X] **T001** Create personalization backend directory structure

  ```
  personalization-backend/
    ├── api/personalize/
    ├── services/
    ├── models/
    ├── templates/ai/
    └── tests/
  ```

* [X] **T002** Set up frontend directories (additive only)

  ```
  docusaurus/src/components
  docusaurus/src/services
  ```

* [X] **T003 [P]** Reuse existing AI/RAG dependencies from `rag-chatbot`
  *(No new OpenAI / LLM SDK installation)*

* [X] **T004 [P]** Configure environment variables for personalization backend

* [X] **T005** Create AI prompt templates directory

  ```
  personalization-backend/templates/ai/
  ```

* [X] **T006** Configure secure communication between:

  * personalization-backend → auth-backend
  * personalization-backend → rag-chatbot

---

## Phase 2: Foundational Tasks

* [X] **T007** Define personalization request/response models
  `personalization-backend/models/personalization.py`

* [X] **T008** Implement AI personalization service (RAG reuse only)
  `personalization-backend/services/ai_personalization.py`

* [X] **T009 [P]** Implement markdown section chunking utility
  *(Reuse existing rag-chatbot logic if available)*

* [X] **T010 [P]** Create structured AI prjun p/.ompt template
  `personalization-backend/templates/ai/personalization_prompt.txt`

* [X] **T011 [P]** Define standardized error response models

---

## Phase 3: [US1] Core Personalization Functionality

### User Story (US1)

> As a logged-in student, I want to personalize a chapter so that it matches my background and learning level.

### Independent Acceptance Criteria

* Logged-in user can click a button at chapter start
* Content visibly changes without page reload
* Change reflects user background
* Original content remains intact
* Graceful fallback exists

---

### Frontend Tasks (Docusaurus)

* [X] **T012 [US1]** Create `PersonalizeButton.tsx`
  `docusaurus/src/components/PersonalizeButton.tsx`

* [X] **T013 [US1]** Implement personalization state management

* [X] **T014 [US1]** Apply orange & white professional styling

* [X] **T015 [US1]** Fetch user profile from `auth-backend`

* [X] **T016 [US1]** Create personalization API client
  `docusaurus/src/services/personalization.ts`

* [X] **T017 [US1]** Capture current chapter markdown at runtime

* [X] **T018 [US1]** Add loading state (target <3s)

* [X] **T019 [US1]** Replace content with smooth animation

* [X] **T020 [US1]** Add subtle indicator for "Personalized View"

* [X] **T021 [US1]** Implement fallback to original content on failure

> ⚠️ Markdown source files must NEVER be modified.

---

### Backend Tasks (Personalization Backend)

* [X] **T022 [US1]** Create endpoint
  `POST /api/personalize/chapter`
  `personalization-backend/api/personalize/chapter.py`

* [X] **T023 [US1]** Validate session via `auth-backend`
  *(No auth logic duplication)*

* [X] **T024 [US1]** Fetch user profile using
  `GET /api/user/profile`

* [X] **T025 [US1]** Combine user profile + chapter content

* [X] **T026 [US1]** Invoke RAG-based AI personalization service

* [X] **T027 [US1]** Preserve chapter structure & learning objectives

* [X] **T028 [US1]** Enforce <3s response target

* [X] **T029 [US1]** Implement safe fallback to original content

* [ ] **T030 [US1]** Create integration test for full personalization flow

---

## Phase 4: [US2] Supporting Personalization Stories

### Supported Adaptations

* Beginner vs Advanced explanations
* GPU vs CPU-safe guidance
* ROS-experienced vs ROS-beginner paths

---

* [X] **T031 [US2]** Extend AI prompt for experience-level tuning
* [X] **T032 [US2]** Add GPU constraint handling in prompt
* [X] **T033 [US2]** Add ROS experience adaptation in prompt
* [X] **T034 [US2]** Implement experience-level logic in AI service
* [X] **T035 [US2]** Implement hardware constraint logic
* [X] **T036 [US2]** Implement ROS-based content filtering
* [X] **T037 [US2]** Validate learning objectives are preserved
* [X] **T038 [US2]** Tests: experience-based personalization
* [X] **T039 [US2]** Tests: GPU constraint handling
* [X] **T040 [US2]** Tests: ROS experience handling

---

## Phase 5: [US3] Performance & Reliability

* [X] **T041 [US3]** Measure API response timing
* [X] **T042 [US3]** Add AI timeout handling
* [X] **T043 [US3]** Implement retry logic
* [X] **T044 [US3]** Add structured error logging
* [X] **T045 [US3]** Improve frontend loading UX
* [X] **T046 [US3]** Prevent layout shift during updates
* [X] **T047 [US3]** Add animation safety guards
* [X] **T048 [US3]** Performance tests (<3s)
* [X] **T049 [US3]** Error recovery tests
* [X] **T050 [US3]** API health monitoring

---

## Phase 6: Documentation, Security & Finalization

* [X] **T051** Document personalization feature
  `docusaurus/docs/personalization.md`

* [X] **T052 [P]** Backend unit tests
  `personalization-backend/tests/`

* [X] **T053 [P]** AI service unit tests

* [X] **T054 [P]** Frontend unit tests

* [X] **T055** End-to-end personalization test

* [X] **T056 [P]** Add input validation

* [X] **T057 [P]** Add auth security checks

* [X] **T058 [P]** Add rate limiting

* [X] **T059 [P]** Add frontend error boundaries

* [X] **T060 [P]** Accessibility compliance

* [X] **T061 [P]** Integration tests

* [X] **T062 [P]** Optional i18n compatibility

* [X] **T063 [P]** Logging & observability

* [X] **T064 [P]** Deployment config

* [X] **T065 [P]** Health check endpoints

* [X] **T066 [P]** OpenAPI documentation

* [X] **T067 [P]** Rollback & safety plan

* [X] **T068 [P]** Final validation & demo readiness

---

## MVP DELIVERY CHECKLIST

✅ Button at chapter start
✅ Logged-in personalization
✅ AI-based content adaptation
✅ RAG reuse
✅ No file deletion
✅ Professional UI
✅ Hackathon Bonus #6 satisfied

---

## Final Directive (For Claude / Spec-Kit Plus)

> Implement this feature strictly as an **additive system**, reusing existing authentication and AI infrastructure, without deleting or modifying any existing files.

---
