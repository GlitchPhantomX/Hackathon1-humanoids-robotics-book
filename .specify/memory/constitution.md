

# 📜 Constitution: AI-Native Chapter Personalization System

**Project:** Physical AI & Humanoid Robotics Textbook
**Platform:** Docusaurus
**Bonus Track:** Requirement #6 (Personalization using signup data)

---

## 1. Purpose & Educational Objective

This constitution defines the **Chapter-Level Personalization System** for the AI-Native textbook built using Docusaurus, Spec-Kit Plus, and Claude.

The goal is to allow a **logged-in user** to **personalize the content of each chapter** by pressing a button at the **start of the chapter**, using the user's **signup background information**, without duplicating chapters or modifying original source files.

This feature satisfies **Hackathon Bonus Requirement #6 (50 points)**.

---

## 2. Non-Destructive Guarantee (Critical)

* ❌ No existing file will be deleted
* ❌ No existing logic will be removed
* ❌ No schema fields will be removed
* ❌ No authentication flow will be altered

All changes must be **additive only**.

---

## 3. User Data Source (Authoritative)

User personalization data MUST be sourced **only** from the existing Better-Auth powered database.

### 3.1 Existing Fields (Already Correct ✅)

The following fields are **authoritative** and MUST be used:

* `softwareBackground`
* `hardwareBackground`
* `programmingLanguages`
* `roboticsExperience`
* `aiMlExperience`
* `hasRosExperience`
* `hasGpuAccess`
* `learningGoals`

These fields are **sufficient for personalization** and must not be altered or removed.

---

## 4. Optional (Recommended) Field Extensions

*(Non-breaking, additive only)*

These fields are **optional but recommended** for richer personalization:

```ts
preferredLearningStyle: enum[
  'theoretical',
  'hands-on',
  'project-based'
]

timeAvailability: enum[
  'low',
  'medium',
  'high'
]
```

⚠️ These fields:

* Must be nullable
* Must not affect existing users
* Must be optional in UI

---

## 5. Personalization Trigger (UI Contract)

### 5.1 Button Placement (Mandatory)

Each chapter MUST include a button rendered **at the very top of the page**:

```
🎯 Personalize this Chapter
```

### 5.2 Visibility Rules

* Button is **visible only to logged-in users**
* Button is **hidden for anonymous users**
* Button does **not appear mid-chapter**

---

## 6. UI / UX Constitution (Strict)

### 6.1 Visual Design

* 🎨 Color Palette: **Orange & White**
* ✨ Smooth, professional animations
* 🧼 Clean spacing, no clutter
* 📐 Proper alignment (no floating elements)

### 6.2 Animation Rules

* Subtle fade / slide animation on content replace
* No page reload
* No layout shift

### 6.3 Professional Constraints

* No "chat-like" UI
* No emoji overload
* Must feel like an **academic AI textbook**

---

## 7. Content Transformation Rules

### 7.1 Source of Truth

* Original chapter markdown remains **unchanged**
* Personalization is **runtime-generated**
* Original content can always be restored

### 7.2 Allowed Transformations

Personalization MAY:

* Simplify or deepen explanations
* Add step-by-step guidance
* Add hardware-specific notes
* Add ROS / GPU warnings
* Adjust difficulty level

Personalization MUST NOT:

* Change learning objectives
* Remove core concepts
* Contradict original content
* Hallucinate new curriculum

---

## 8. Personalization Modes (Logic Contract)

Claude must derive a **Personalization Profile**:

| Signal         | Effect                              |
| -------------- | ----------------------------------- |
| Beginner       | More explanation, definitions       |
| Advanced       | Shorter explanations, optimizations |
| No GPU         | CPU alternatives + warnings         |
| ROS Experience | Skip ROS basics                     |
| Learning Goals | Emphasize relevant sections         |

---

## 9. AI Prompt Constitution (Mandatory)

Claude must follow this **strict prompt format**:

```
You are an AI textbook personalization engine.

Chapter: <chapter_id>

User Profile:
- Software Level: <value>
- Robotics Experience: <value>
- ROS Experience: <true/false>
- GPU Access: <true/false>
- Learning Goals: <text>

Rules:
- Do not remove concepts
- Adjust explanation depth
- Maintain academic tone
- Keep structure recognizable
- No hallucinations

Return personalized markdown content only.
```

---

## 10. Backend Contract

### 10.1 New Endpoint (Additive Only)

```
POST /api/personalize/chapter
```

Input:

* Chapter ID
* Raw chapter markdown
* Authenticated session

Output:

* Personalized markdown

### 10.2 Security

* Requires valid Better-Auth session
* No anonymous access
* No stored personalized copies (stateless)

---

## 11. Frontend Contract (Docusaurus)

### 11.1 Component Injection

Each chapter MUST import and render:

```md
import PersonalizeButton from '@site/src/components/PersonalizeButton';
```

The component:

* Fetches user profile
* Calls personalization API
* Replaces content in-place
* Animates transition

---

## 12. Reusability & Bonus Scoring Alignment

This system demonstrates:

* ✅ True AI-Native content
* ✅ No duplicated chapters
* ✅ Real use of signup data
* ✅ Runtime personalization
* ✅ Professional UI
* ✅ Scalable architecture

---

## 13. Success Definition

The feature must allow authenticated users to personalize chapter content using their signup background information without duplicating chapters or modifying source files.


