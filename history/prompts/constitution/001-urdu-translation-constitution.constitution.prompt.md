---
id: 1
title: "Urdu Translation Constitution"
stage: constitution
date: "2025-12-19"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels: ["translation", "urdu", "docusaurus", "authentication"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Urdu Translation Constitution

## User Input
```
# 📘 Constitution: Chapter-Level Urdu Translation Toggle System

**Project:** Physical AI & Humanoid Robotics Textbook
**Platform:** Docusaurus
**Bonus Track:** Requirement #7 (Translation → Urdu)

---

## 1. Purpose & Educational Objective

The goal of this feature is to allow **authenticated users** to dynamically translate **entire chapter content** from **English → Urdu** by pressing a **single toggle button** located at the **start of each chapter page**.

This feature:

* Enhances accessibility for Urdu-speaking learners
* Demonstrates **AI-native textbook capabilities**
* Qualifies the project for **+50 bonus points**
* Integrates seamlessly with existing **authentication, RAG, and reusable intelligence systems**

---

## 2. Functional Requirements

### 2.1 Eligibility Rules

| Condition                  | Behavior                                      |
| -------------------------- | --------------------------------------------- |
| User **not logged in**     | Translation toggle is hidden or disabled      |
| User **logged in**         | Translation toggle is visible and interactive |
| Translation already exists | Cached translation is reused                  |
| Translation not available  | Backend generates Urdu translation            |

---

### 2.2 User Interaction

At the **top of every chapter page**, the user sees:

* A **Language Toggle Button**
* Label: `EN → اردو`
* Default State:

  * **English active**
  * Button background: **White**
* Active Urdu State:

  * **Urdu content displayed**
  * Button background: **Orange**
  * Layout switches to **RTL (right-to-left)**

Toggling back restores the **original English content**.

---

## 3. UX & UI Requirements (Frontend)

### 3.1 Component Location

All frontend logic must reside inside:

```
physical-ai-robotics-textbook/
└── docusaurus/
    └── src/
        └── components/
            └── translation/
                ├── LanguageToggle.tsx
                ├── TranslationButton.tsx
                └── rtlStyles.css
```

---

### 3.2 Component Responsibilities

#### `LanguageToggle.tsx`

* Detects authentication state
* Determines current language (`en` or `ur`)
* Triggers translation request
* Applies RTL layout when Urdu is active

#### `TranslationButton.tsx`

* Visual toggle button
* Orange (active) / White (inactive)
* Accessible (ARIA-compliant)
* Mobile-responsive

---

### 3.3 Styling Rules

| Element      | Requirement                      |
| ------------ | -------------------------------- |
| Button Color | White (EN), Orange (UR)          |
| Font         | Same typography, Urdu-compatible |
| Direction    | `ltr` for EN, `rtl` for UR       |
| Animation    | Subtle transition (200–300ms)    |

---

## 4. Backend Architecture (Translation Engine)

### 4.1 Backend Location

All translation backend logic must live inside:

```
physical-ai-robotics-textbook/
└── physical-ai/
    └── translation/
        ├── router.py
        ├── service.py
        ├── cache.py
        └── schemas.py
```

---

### 4.2 API Contract

**Endpoint:**

```
POST /api/translate
```

**Request Payload:**

```json
{
  "chapter_id": "01-ros2/03-services-actions",
  "source_language": "en",
  "target_language": "ur",
  "content": "<markdown-content>"
}
```

**Response:**

```json
{
  "translated_content": "<urdu-markdown>",
  "cached": true
}
```

---

### 4.3 Translation Engine

* Uses LLM-based translation (Urdu-native)
* Preserves:

  * Markdown structure
  * Code blocks
  * Headings
  * Tables
* No hallucination
* No content deletion

Backend may be implemented using:

* FastAPI
* Existing AI infrastructure already present in the project

---

## 5. Caching & Performance Rules

| Rule        | Description                       |
| ----------- | --------------------------------- |
| Cache Key   | `user_id + chapter_id + language` |
| Storage     | Local disk / DB / KV store        |
| TTL         | Optional (configurable)           |
| Cache First | Always check cache before LLM     |

This ensures:

* Low latency
* Minimal token usage
* Deterministic translations

---

## 6. RTL (Right-to-left) Rendering Rules

When Urdu is active:

* Entire markdown container switches to `dir="rtl"`
* Code blocks remain `ltr`
* Tables auto-align right
* Inline math unaffected

RTL rules must be isolated and reversible.

---

## 7. Security & Content Integrity

* Only authenticated users may trigger translation
* Translation API must validate:

  * User session
  * Chapter ownership
* No original markdown files are overwritten
* Translations are **derived views**, not source edits

---

## 8. Testing & Validation Criteria

### Automated Tests (Required)

* Toggle visibility (auth vs non-auth)
* EN ↔ UR switching
* RTL layout correctness
* Cache hit vs miss
* Markdown preservation
* Performance under repeated toggles

### Manual Acceptance Criteria

* Button feels native to the book
* Urdu is fluent and technical
* No page reloads
* No layout breaking

---

## 9. Reusability & Bonus Scoring Al create my consitution for this requirement and be cafeful do not override and remove any other consitution file just create a new one file in this path C:\new - Copy\.specify\memory
```

## Response Summary

A comprehensive constitution for the Urdu translation system has been created at `.specify/memory/constitution.md`. The constitution includes:

- Detailed technical architecture for multi-language translation
- Component specifications for LanguageContext, LanguageToggle, and TranslatedContent
- Backend API design for translation services
- Integration approach with Docusaurus
- Authentication requirements
- Implementation phases and success criteria

The constitution addresses all requirements including authentication, RTL support, caching, and proper styling preservation.