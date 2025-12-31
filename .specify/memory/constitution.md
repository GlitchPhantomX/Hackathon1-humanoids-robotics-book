

# 📜 Constitution: AI-Native Chapter Personalization & Multi-Language Translation System

**Project:** Physical AI & Humanoid Robotics Textbook
**Platform:** Docusaurus
**Bonus Track:** Requirement #6 (Personalization using signup data) + Multi-Language Translation System

---

## 1. Purpose & Educational Objective

This constitution defines the **Chapter-Level Personalization System** and **Multi-Language Translation System** for the AI-Native textbook built using Docusaurus, Spec-Kit Plus, and Claude.

The goals are:
- **Personalization:** Allow a **logged-in user** to **personalize the content of each chapter** by pressing a button at the **start of the chapter**, using the user's **signup background information**, without duplicating chapters or modifying original source files.
- **Translation:** Add multi-language support (Urdu and Hindi) to the Physical AI & Humanoid Robotics textbook while maintaining exact styling, structure, and all existing functionality.

These features satisfy **Hackathon Bonus Requirement #6 (50 points)** and **Multi-Language Translation Bonus (50 points)**.

---

## 2. Non-Destructive Guarantee (Critical)

* ❌ No existing file will be deleted
* ❌ No existing logic will be removed
* ❌ No schema fields will be removed
* ❌ No authentication flow will be altered
* ❌ No existing components will be modified
* ❌ No existing styling will be changed

All changes must be **additive only**.

---

## 3. Language Support Requirements

### 3.1 Supported Languages
- **Primary Languages**: English (existing), Urdu, Hindi
- **Translation Scope**: All chapter content in `/docs` folder
- **Styling Preservation**: Exact same orange/white color scheme and component structure
- **File Structure**: Mirror English docs structure for each language

### 3.2 Folder Structure
```
docusaurus/
├── docs/                          # English (existing - DO NOT MODIFY)
│   ├── 00-introduction/
│   ├── 01-ros2/
│   ├── 02-simulation/
│   └── ...
├── i18n/
│   ├── ur/                        # NEW: Urdu translations
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/
│   │           ├── 00-introduction/
│   │           │   ├── 01-welcome.md
│   │           │   ├── 02-prerequisites.md
│   │           │   └── ...
│   │           ├── 01-ros2/
│   │           └── ...
│   └── hi/                        # NEW: Hindi translations
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── 00-introduction/
│               └── ...
└── src/
    └── components/
        └── TranslateButton.tsx    # NEW COMPONENT
```

---

## 4. Translation UI Components Required

### 4.1 Language Dropdown (Navbar)
Location: src/components/Navbar or docusaurus.config.mjs
Position: Left of Login/Signup buttons with spacing
Options:
  - English (EN) 🇬🇧
  - اردو (UR) 🇵🇰
  - हिंदी (HI) 🇮🇳

Behavior:
  - Persists selection across page navigation
  - Changes entire UI (sidebar, navbar, content)
  - Maintains user position in chapter structure

### 4.2 Translation Toggle Button
Location: Start of each chapter (after title, before content)
Component: <TranslateButton />
Style: Orange gradient matching PersonalizeButton
Function: Switch between EN/UR/HI for current page

---

## 5. Translation Requirements

### 5.1 Content Translation Rules
Source: docs/00-introduction/01-welcome.md
Target: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md

MUST PRESERVE:
✓ All frontmatter (sidebar_position, title, description)
✓ All import statements (ReadingTime, PersonalizeButton)
✓ All component usage (<PersonalizeButton/>, <ReadingTime/>)
✓ All className attributes (main-heading, underline-class, border-line, etc.)
✓ All HTML structure (<div>, <h2>, etc.)
✓ All code blocks with syntax highlighting
✓ All links and navigation
✓ All exercise boxes (:::tip, :::caution)

MUST TRANSLATE:
✓ All text content
✓ Headings and subheadings
✓ Exercise descriptions
✓ Table content
✓ Navigation labels

### 5.2 Styling Consistency
Colors (MUST MATCH):
  - Primary: Orange (#FF6B35 or current orange)
  - Secondary: White (#FFFFFF)
  - Backgrounds: Current theme colors
  - Gradients: Current orange gradients

Components (MUST REUSE):
  - .main-heading
  - .second-heading
  - .third-heading
  - .underline-class
  - .border-line
  - .full-content
  - .summary-content
  - All existing CSS classes

---

## 6. Docusaurus Configuration for Translation

```javascript
// docusaurus.config.mjs
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'hi'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',  // Right-to-left
        htmlLang: 'ur-PK',
      },
      hi: {
        label: 'हिंदी',
        direction: 'ltr',
        htmlLang: 'hi-IN',
      },
    },
  },

  navbar: {
    items: [
      // ... existing items
      {
        type: 'localeDropdown',
        position: 'right',
        dropdownItemsAfter: [],
        className: 'language-dropdown',
      },
      // Login/Signup buttons (with left spacing)
    ],
  },
};
```

---

## 7. TranslateButton Component Implementation

```typescript
// src/components/TranslateButton.tsx
import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function TranslateButton() {
  const {i18n} = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();

  const switchLanguage = (locale: string) => {
    const currentPath = location.pathname;
    const newPath = currentPath.replace(/^\/(en|ur|hi)/, `/${locale}`);
    history.push(newPath);
  };

  return (
    <div className="translate-button-container">
      <button
        className="translate-button"
        onClick={() => switchLanguage('ur')}
      >
        اردو میں پڑھیں
      </button>
      <button
        className="translate-button"
        onClick={() => switchLanguage('hi')}
      >
        हिंदी में पढ़ें
      </button>
    </div>
  );
}
```

---

## 8. User Data Source (Authoritative)

User personalization data MUST be sourced **only** from the existing Better-Auth powered database.

### 8.1 Existing Fields (Already Correct ✅)

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

## 9. Optional (Recommended) Field Extensions

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

## 10. Personalization Trigger (UI Contract)

### 10.1 Button Placement (Mandatory)

Each chapter MUST include a button rendered **at the very top of the page**:

```
🎯 Personalize this Chapter
```

### 10.2 Visibility Rules

* Button is **visible only to logged-in users**
* Button is **hidden for anonymous users**
* Button does **not appear mid-chapter**

---

## 11. UI / UX Constitution (Strict)

### 11.1 Visual Design

* 🎨 Color Palette: **Orange & White**
* ✨ Smooth, professional animations
* 🧼 Clean spacing, no clutter
* 📐 Proper alignment (no floating elements)

### 11.2 Animation Rules

* Subtle fade / slide animation on content replace
* No page reload
* No layout shift

### 11.3 Professional Constraints

* No "chat-like" UI
* No emoji overload
* Must feel like an **academic AI textbook**

---

## 12. Content Transformation Rules

### 12.1 Source of Truth

* Original chapter markdown remains **unchanged**
* Personalization is **runtime-generated**
* Original content can always be restored

### 12.2 Allowed Transformations

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

## 13. Personalization Modes (Logic Contract)

Claude must derive a **Personalization Profile**:

| Signal         | Effect                              |
| -------------- | ----------------------------------- |
| Beginner       | More explanation, definitions       |
| Advanced       | Shorter explanations, optimizations |
| No GPU         | CPU alternatives + warnings         |
| ROS Experience | Skip ROS basics                     |
| Learning Goals | Emphasize relevant sections         |

---

## 14. AI Prompt Constitution (Mandatory)

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

## 15. Backend Contract

### 15.1 New Endpoint (Additive Only)

```
POST /api/personalize/chapter
```

Input:

* Chapter ID
* Raw chapter markdown
* Authenticated session

Output:

* Personalized markdown

### 15.2 Security

* Requires valid Better-Auth session
* No anonymous access
* No stored personalized copies (stateless)

---

## 16. Frontend Contract (Docusaurus)

### 16.1 Component Injection

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

## 17. Translation Guidelines

### 17.1 Chapter Structure (Example)
```markdown
# English: docs/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "Welcome to Physical AI & Humanoid Robotics"
---

# Urdu: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس میں خوش آمدید"
---

# Hindi: i18n/hi/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md
---
sidebar_position: 1
title: "फिजिकल एआई और ह्यूमनॉइड रोबोटिक्स में आपका स्वागत है"
---
```

### 17.2 Technical Terms
```yaml
Keep in English (with translation in parentheses):
  - ROS 2 (ROS 2)
  - NVIDIA Isaac (NVIDIA Isaac)
  - URDF (URDF - یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ)
  - Python (Python)
  - Node (Node - نوڈ)
  - Topic (Topic - ٹاپک)
```

---

## 18. Critical Rules (DO NOT VIOLATE)

```yaml
PROTECTED FILES (DO NOT MODIFY):
  - docs/** (all English content)
  - auth-backend/** (authentication system)
  - rag-chatbot/** (existing chatbot)
  - src/components/Chatbot.tsx
  - src/components/PersonalizeButton.tsx
  - All existing styling files

MUST PRESERVE:
  - All authentication flows
  - All existing routes
  - All existing components
  - All database schemas
  - All environment variables
  - Build configuration

NEW FILES ONLY:
  - i18n/ur/** (Urdu translations)
  - i18n/hi/** (Hindi translations)
  - src/components/TranslateButton.tsx
  - Updated docusaurus.config.mjs (i18n section only)
  - Updated navbar styling for dropdown spacing
```

---

## 19. Testing Checklist

```markdown
- [ ] All English content unchanged
- [ ] Urdu content displays with RTL layout
- [ ] Hindi content displays with LTR layout
- [ ] Language dropdown works in navbar
- [ ] TranslateButton appears in all chapters
- [ ] Sidebar translates with language
- [ ] Orange/white color scheme consistent
- [ ] All components render correctly
- [ ] Navigation preserves chapter position
- [ ] Authentication still works
- [ ] Chatbot still works
- [ ] Build succeeds without errors
- [ ] All styling classes preserved
- [ ] Personalization button still works
- [ ] User data is still accessible for personalization
```

---

## 20. Reusability & Bonus Scoring Alignment

This system demonstrates:

* ✅ True AI-Native content
* ✅ No duplicated chapters
* ✅ Real use of signup data
* ✅ Runtime personalization
* ✅ Professional UI
* ✅ Scalable architecture
* ✅ Multi-language support
* ✅ RTL language support for Urdu
* ✅ Seamless language switching

---

## 21. Success Definition

The feature must allow authenticated users to personalize chapter content using their signup background information without duplicating chapters or modifying source files, AND provide multi-language translation support (Urdu and Hindi) while maintaining all existing functionality and styling.


