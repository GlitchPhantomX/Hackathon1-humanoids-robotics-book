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

- What happens when the AI translation service is unavailable or fails? (Answer: Show original English content with error notification)
- How does the system handle very large chapters that might exceed API limits? (Answer: Show error and suggest chapter splitting)
- What if the user's session expires during a translation request? (Answer: Cancel translation and show error)
- How does the system handle malformed markdown that can't be properly translated? (Answer: Preserve original and show error)

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
- **Cache Entry**: Maps user_id + chapter_id + target_language to translated content with 24-hour expiration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate any chapter to Urdu within 4 seconds for first-time translations
- **SC-002**: Subsequent translations of the same chapter return within 200ms due to caching
- **SC-003**: 100% of authenticated users can successfully toggle between English and Urdu on any chapter page
- **SC-004**: 95% of translated content maintains proper markdown structure including headings, code blocks, and tables
- **SC-005**: Users experience proper RTL layout when viewing Urdu content with code blocks remaining in LTR direction

## Clarifications

### Session 2025-12-19

- Q: What is the cache expiration policy for translations? → A: Cache translations for 24 hours
- Q: How should the system handle translation service failures? → A: Show original English content with error notification
- Q: What happens if the user's session expires during a translation request? → A: Cancel translation and show error
- Q: How should the system handle very large chapters that might exceed API limits? → A: Show error and suggest chapter splitting
- Q: How should the system handle malformed markdown that can't be properly translated? → A: Preserve original and show error