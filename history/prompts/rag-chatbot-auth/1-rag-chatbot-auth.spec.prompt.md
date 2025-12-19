---
id: 1
title: "RAG Chatbot Auth Spec"
stage: spec
date: "2025-12-18"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "rag-chatbot-auth"
branch: "001-rag-chatbot-auth"
user: "user"
command: "/sp.specify"
labels: ["specification", "rag-chatbot", "authentication"]
links:
  spec: "specs/001-rag-chatbot-auth/spec.md"
  ticket: null
  adr: null
  pr: null
files: []
tests: []
---

# Prompt History Record

## PROMPT_TEXT

```
## RAG Chatbot Integration With Auth + Advanced Features

---

## 1️⃣ PURPOSE & SCOPE

Enhance the existing **RAG Chatbot** in `rag-chatbot` backend + Docusaurus frontend to include:

1. **Authentication enforcement** using existing signup/login system
2. **Answer only from selected text**
3. **Streaming responses**
4. **Source citations**
5. Maintain **existing UI** and **existing CLI chatbot behavior**

---

## 2️⃣ SYSTEM CONSTRAINTS

* ❌ Do not create new backend directories
* ❌ Do not break existing CLI chatbot
* ✅ Use existing `auth-backend` signup/login
* ✅ Floating icon and chatbot UI unchanged
* ✅ Backward-compatible RAG logic

---

## 3️⃣ FEATURE SPECIFICATIONS

---

### 🔹 FEATURE 1: Authentication Enforcement

**Objective**

* Chatbot accessible **only after successful login/signup**.

**Frontend (Docusaurus / React)**

* When user clicks chatbot icon:

  1. Check login state via existing auth context/hooks.
  2. If not logged in → open `SignupModal` (or `LoginModal`)

     * No chatbot query allowed until user authenticates.
  3. After successful login/signup → enable chatbot input.

**Backend (RAG Chatbot API)**

* No additional authentication logic needed; rely on existing API tokens/session management.
* API should return `401 Unauthorized` if request from non-logged-in user (optional extra layer).

**UX Flow**

```
User selects text -> Clicks chatbot icon -> Auth check:
   [ Not logged in ] -> SignupModal / LoginModal appears -> After signup/login -> Chatbot opens with selected text
   [ Logged in ] -> Chatbot opens immediately
```

---

### 🔹 FEATURE 2: Answer Only From Selected Text

**Backend API `/chat`**

* Accept optional payload:

```json
{
  "message": "Explain inverse kinematics",
  "selected_text": "Inverse kinematics is the process of computing joint angles..."
}
```

* If `selected_text` provided → **restrict answer strictly to that text**.
* If not enough info → respond:

  > "The selected text does not contain enough information to answer this question."

**Frontend**

* Selected text auto-injected into input when user clicks chatbot icon.
* Floating icon appears on text selection (existing behavior maintained).

---

### 🔹 FEATURE 3: Streaming Responses

**Backend**

* Implement token streaming via **SSE or chunked responses**.
* Endpoint `/chat` should support streaming while preserving RAG logic.

**Frontend**

* Display partial responses progressively.
* Loading indicator transitions smoothly.
* Compatible with current chatbot UI.

---

### 🔹 FEATURE 4: Source Citations

**Backend Response Format**

```json
{
  "answer": "...",
  "sources": [
    {
      "title": "Chapter 3 – Humanoid Kinematics",
      "page": "Section 3.2",
      "score": 0.87
    }
  ]
}
```

**Rules**

* RAG-based answers → mandatory sources
* Selected-text answers → optional sources

**Frontend**

* Sources displayed below answer, minimally invasive.

---

### 🔹 FEATURE 5: Rate Limiting (Optional)

* Already handled in backend auth logic, can skip if not needed.
* Ensure no abuse of chatbot API.

---

## 4️⃣ BACKWARD COMPATIBILITY

| Component                   | Status      |
| --------------------------- | ----------- |
| Existing CLI chatbot        | ✅ unchanged |
| Auth backend / signup-login | ✅ unchanged |
| Docusaurus UI               | ✅ unchanged |
| RAG logic                   | ✅ unchanged |

---

## 5️⃣ QUALITY REQUIREMENTS

* Modular, documented code
* Clean integration with existing `rag-chatbot`
* Preserve current folder structure and UI
* Clear error messages
* Maintain existing hooks and React context

---

## 6️⃣ ACCEPTANCE CRITERIA

✅ Chatbot opens only after login/signup
✅ Selected text is injected automatically
✅ Streaming responses functional
✅ Source citations shown for RAG answers
✅ Existing CLI chatbot unchanged
✅ UI not broken

---

## 7️⃣ CLAUDE CODE READY PROMPT

> **Use this EXACT prompt for Claude Code CLI:**

```
Implement the above specification using Spec-Kit Plus.
Do not break existing CLI  c
```