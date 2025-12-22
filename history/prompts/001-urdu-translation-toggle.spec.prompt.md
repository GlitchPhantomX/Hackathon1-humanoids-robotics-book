---
id: 001
title: Urdu Translation Toggle Specification
stage: spec
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: urdu-translation-toggle
branch: 01-urdu-translation-toggle
user: Claude Code User
command: /sp.specify
labels: ["translation", "accessibility", "urdu", "multilingual"]
links:
  spec: specs/01-urdu-translation-toggle/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/01-urdu-translation-toggle/spec.md
 - specs/01-urdu-translation-toggle/checklists/requirements.md
tests:
 - specs/01-urdu-translation-toggle/checklists/requirements.md
---

## Prompt

# 📐 Specification: Chapter-Level Urdu Translation Toggle

**Spec ID:** `0007-urdu-translation-toggle`
**Project:** Physical AI & Humanoid Robotics Textbook
**Platform:** Docusaurus
**Backend Framework:** FastAPI
**Status:** Approved for Implementation
**Bonus Alignment:** Hackathon Requirement #7 (+50 points)

---

## 1. Problem Statement

The textbook is currently authored and rendered exclusively in English. This limits accessibility for Urdu-speaking learners and prevents the project from demonstrating **AI-native, multilingual content delivery**, a key hackathon evaluation criterion.

There is currently **no mechanism** for:

* Per-chapter language switching
* Dynamic translation using AI
* RTL (Right-to-Left) rendering
* Auth-gated content transformation

---

## 2. Objective

Enable **authenticated users** to translate **entire chapter content** from **English to Urdu** by pressing a **single toggle button** at the **start of each chapter page**, without modifying source markdown files.

The system must:

* Preserve markdown structure
* Apply RTL layout automatically
* Cache translations
* Integrate cleanly with existing authentication and AI infrastructure

---

## 3. In-Scope Requirements

### 3.1 Functional Scope

✅ Chapter-level translation (not paragraph-level)
✅ English → Urdu only (for this spec)
✅ Logged-in users only
✅ Toggle-based UI
✅ AI-generated translation
✅ RTL layout switching
✅ Translation caching

---

### 3.2 Out-of-Scope

❌ Editing translated text
❌ Saving Urdu as source markdown
❌ Guest user translation
❌ Multiple language selector (future spec)
❌ Offline translation

---

## 4. User Stories

### US-01: Authenticated Translation Access

> As a logged-in user, I want to translate the current chapter into Urdu so I can read it in my native language.

### US-02: Toggle Control

> As a user, I want a single button that toggles English ↔ Urdu without page reload.

### US-03: Visual Feedback

> As a user, I want clear visual indication (color + direction) of the active language.

---

## 5. UI / UX Specification

### 5.1 Button Placement

* Location: **Top of every chapter page**
* Appears **after title**, **before content**
* Hidden or disabled if user is not authenticated

---

### 5.2 Button Behavior

| State   | Label     | Color  | Layout |
| ------- | --------- | ------ | ------ |
| Default | EN → اردو | White  | LTR    |
| Active  | اردو → EN | Orange | RTL    |

---

### 5.3 Styling Rules

* Button must look native to the Docusaurus theme
* Smooth transition (200–300ms)
* Fully responsive
* WCAG-compliant contrast

---

## 6. Frontend Architecture

### 6.1 File Location (Mandatory)

```
docusaurus/
└── src/
    └── components/
        └── translation/
            ├── LanguageToggle.tsx
            ├── TranslationButton.tsx
            ├── useTranslation.ts
            └── rtl.css
```

---

### 6.2 Responsibilities

#### `LanguageToggle.tsx`

* Detect authentication state
* Detect current route / chapter ID
* Maintain active language state
* Apply RTL mode when Urdu is active

#### `useTranslation.ts`

* Call backend API
* Handle loading & error states
* Resolve cached vs fresh translations

---

## 7. Backend Architecture

### 7.1 File Location (Mandatory)

```
physical-ai/
└── translation/
    ├── router.py
    ├── service.py
    ├── cache.py
    ├── schemas.py
    └── prompts/
        └── en_to_ur.txt
```

---

### 7.2 API Contract

#### Endpoint

```
POST /api/translation/urdu
```

#### Request Schema

```json
{
  "chapter_id": "01-ros2/03-services-actions",
  "content_markdown": "<raw markdown>",
  "source_language": "en",
  "target_language": "ur"
}
```

#### Response Schema

```json
{
  "translated_markdown": "<urdu markdown>",
  "cached": true
}
```

---

## 8. Translation Rules (Hard Constraints)

The AI translation engine **must**:

* Preserve:

  * Markdown headings
  * Code blocks
  * Tables
  * Links
* Translate:

  * Paragraph text
  * Headings
  * Lists
* Never:

  * Translate code
  * Remove content
  * Add explanations

---

## 9. Caching Strategy

### 9.1 Cache Key

```
user_id + chapter_id + target_language
```

### 9.2 Behavior

* Cache checked **before** LLM call
* Cached result returned instantly
* Cache storage: disk / DB / KV (implementation choice)

---

## 10. RTL (Right-to-Left) Rendering

When Urdu is active:

* Entire content container:

  ```html
  dir="rtl"
  ```
* Code blocks forced to `ltr`
* Tables aligned right
* Inline math unchanged

RTL must be:

* Reversible
* Isolated
* Non-destructive

---

## 11. Security Requirements

* API accessible **only** to authenticated users
* Validate:

  * User session
  * Chapter access
* No markdown files are modified on disk
* Translation is a **derived view**, not persisted content

---

## 12. Performance Requirements

| Metric            | Target   |
| ----------------- | -------- |
| Cached response   | < 200ms  |
| First translation | < 2–4s   |
| Toggle latency    | Instant  |
| No page reloads   | Required |

---

## 13. Testing & Acceptance Criteria

### 13.1 Automated Tests

* Toggle visibility (auth / no auth)
* EN ↔ UR switching
* Cache hit vs miss
* RTL correctness
* Markdown integrity

---

### 13.2 Acceptance Criteria (Judges)

✅ User logs in
✅ Opens any chapter
✅ Clicks **EN → اردو**
✅ Entire chapter appears in fluent Urdu
✅ Button turns orange
✅ Layout switches to RTL

---

## 14. Reusability & Extension

This specification is designed to be **language-agnostic** and may later support:

* Arabic
* Persian
* Pashto
* Full multilingual selector

---

## 15. Implementation Readiness

This specification is:

* ✔ Compatible with **Spec-Kit Plus**
* ✔ Suitable for **Claude Code CLI**
* ✔ Aligned with Hackathon Bonus #7
* ✔ Architecturally consistent with existing project

---

 now create a specification into my specs folder and be carefull do not remove and override any other specs file just create a new folder for this

## Response snapshot

# Feature Specification: Urdu Translation Toggle

**Feature Branch**: `01-urdu-translation-toggle`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Enable authenticated users to translate entire chapter content from English to Urdu by pressing a single toggle button at the start of each chapter page, without modifying source markdown files. The system must preserve markdown structure, apply RTL layout automatically, cache translations, and integrate cleanly with existing authentication and AI infrastructure."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Translate Chapter Content (Priority: P1)

As a logged-in user, I want to translate the current chapter into Urdu so I can read it in my native language. When I click the toggle button, the entire chapter content should be translated from English to Urdu with proper RTL layout.

**Why this priority**: This is the core functionality that directly addresses the accessibility need for Urdu-speaking learners and is the primary value proposition of the feature.

**Independent Test**: Can be fully tested by logging in, opening a chapter, clicking the EN → اردو toggle button, and verifying that the content appears in fluent Urdu with RTL layout.

**Acceptance Scenarios**:

1. **Given** user is logged in and viewing an English chapter, **When** user clicks the "EN → اردو" toggle button, **Then** the entire chapter content appears in Urdu with RTL layout and the button changes to "اردو → EN"
2. **Given** user has translated a chapter to Urdu, **When** user clicks the "اردو → EN" toggle button, **Then** the chapter content reverts to English with LTR layout

---

### User Story 2 - Authenticated Access Control (Priority: P2)

As a system, I need to ensure that translation functionality is only available to authenticated users so that the feature is properly gated.

**Why this priority**: Security and access control are critical requirements that must be enforced before the core translation functionality can be safely provided.

**Independent Test**: Can be tested by attempting to access the translation toggle as both an authenticated and non-authenticated user, verifying that it's only available when logged in.

**Acceptance Scenarios**:

1. **Given** user is not logged in, **When** user views a chapter page, **Then** the translation toggle button is hidden or disabled
2. **Given** user is logged in, **When** user views a chapter page, **Then** the translation toggle button is visible and functional

---

### User Story 3 - Performance and Caching (Priority: P3)

As a user, I want subsequent translations to be fast so that I don't experience delays when toggling between languages.

**Why this priority**: Performance is critical for user experience, and caching ensures that repeated translations are fast and don't overuse AI resources.

**Independent Test**: Can be tested by translating a chapter, then toggling back and forth between languages, verifying that subsequent translations are returned quickly.

**Acceptance Scenarios**:

1. **Given** user has previously translated a chapter, **When** user translates the same chapter again, **Then** the cached translation is returned instantly
2. **Given** user is translating a chapter for the first time, **When** user clicks the toggle, **Then** the translation is generated within 2-4 seconds

---

### Edge Cases

- What happens when the AI translation service is unavailable or fails?
- How does the system handle very large chapters that might exceed API limits?
- What if the user's session expires during a translation request?
- How does the system handle malformed markdown that can't be properly translated?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a toggle button labeled "EN → اردو" that appears at the top of each chapter page for authenticated users
- **FR-002**: System MUST translate entire chapter content from English to Urdu when the toggle is activated
- **FR-003**: System MUST apply RTL (right-to-left) layout to the content container when Urdu is active
- **FR-004**: System MUST preserve markdown structure including headings, code blocks, tables, and links during translation
- **FR-005**: System MUST cache translations using a key combining user_id, chapter_id, and target_language
- **FR-006**: System MUST only allow authenticated users to access translation functionality
- **FR-007**: System MUST validate user session and chapter access before processing translation requests
- **FR-008**: System MUST maintain toggle state during the user's session
- **FR-009**: System MUST ensure code blocks remain in LTR direction even when Urdu layout is active
- **FR-010**: System MUST return cached translations instantly when available

### Key Entities

- **Translation Request**: Contains chapter_id, content_markdown, source_language, and target_language for API processing
- **Translation Response**: Contains translated_markdown and cached status information
- **User Session**: Authentication state that determines access to translation functionality
- **Cache Entry**: Maps user_id + chapter_id + target_language to translated content with expiration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate any chapter to Urdu within 4 seconds for first-time translations
- **SC-002**: Subsequent translations of the same chapter return within 200ms due to caching
- **SC-003**: 100% of authenticated users can successfully toggle between English and Urdu on any chapter page
- **SC-004**: 95% of translated content maintains proper markdown structure including headings, code blocks, and tables
- **SC-005**: Users experience proper RTL layout when viewing Urdu content with code blocks remaining in LTR direction

## Outcome

- ✅ Impact: Created complete specification for Urdu translation toggle feature with user stories, requirements, and success criteria
- 🧪 Tests: Created validation checklist to ensure spec quality
- 📁 Files: Created spec.md and requirements checklist in specs/01-urdu-translation-toggle/
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification successfully captures user needs for multilingual accessibility while maintaining technical constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Specification meets all quality criteria
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase