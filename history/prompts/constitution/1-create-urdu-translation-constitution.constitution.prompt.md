---
id: 1
title: "Create Urdu Translation Constitution"
stage: "constitution"
date_iso: "2025-12-15"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "7-reusable-intelligence-system"
user: "user"
command: "sp.constitution"
labels: ["constitution", "translation", "urdu", "docusaurus"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Create Urdu Translation Constitution

## PROMPT_TEXT

```
now create a new consitution in this path C:\new - Copy\.specify\memory be remember do not remove any other file and do not override # Constitution: Multi-Language Translation System
## Hackathon I - Requirement 7: Urdu Translation Feature

---

## Document Information

**Project:** Physical AI & Humanoid Robotics Textbook
**Location:** `C:\new - Copy\physical-ai-robotics-textbook`
**Requirement:** Requirement 7 - Urdu Translation (50 bonus points)
**Focus:** Multi-language content translation with preserved styling
**Status:** 🌐 Ready for Implementation
**Created:** December 15, 2025

---

## Mission Statement

Implement a **professional, production-ready multi-language translation system** for the Docusaurus textbook that allows logged-in users to seamlessly translate all chapter content into Urdu (and other languages) while **perfectly preserving** all existing styling, layout, and functionality.

**Target Outcome:** Earn full 50 bonus points for exceptional implementation that:
- Provides instant language switching via navbar toggle
- Preserves all CSS styling and layout
- Translates all content accurately
- Works seamlessly across all chapters
- Requires zero modifications to existing files

---

## Requirement Analysis

### Hackathon Requirement (Exact Text)
```
"7. Participants can receive up to 50 extra bonus points if
    the logged user can translate the content in Urdu in the
    chapters by pressing a button at the start of each chapter."
```

### Our Enhanced Implementation
```
✅ Language toggle in navbar (more accessible than per-chapter button)
✅ Dropdown with English, Urdu, and 1 more language (Arabic/Hindi)
✅ Translation applies to ALL chapters automatically
✅ Preserves ALL existing styling and layout
✅ Works only for logged-in users
✅ Instant switching, no page reload
✅ Professional UI/UX
```

---

## Technical Architecture

### System Overview

```
┌─────────────────────────────────────────────────────┐
│                  Docusaurus Navbar                  │
│  [Logo] [Docs] [Blog] [Language Toggle ▼] [Auth]   │
│                           │                          │
│                           ├── English ✓              │
│                           ├── اردو (Urdu)           │
│                           └── العربية (Arabic)       │
└─────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────┐
│              Translation System                      │
│                                                      │
│  ┌──────────────┐    ┌──────────────┐              │
│  │  Original    │    │  Translated  │              │
│  │  Content     │───▶│  Content     │              │
│  │  (English)   │    │  (Urdu/Ar)   │              │
│  └──────────────┘    └──────────────┘              │
│         │                    │                      │
│         └────────┬───────────┘                      │
│                  ▼                                   │
│         ┌────────────────┐                          │
│         │  Same Styling  │                          │
│         │  Same Layout   │                          │
│         │  Same Classes  │                          │
│         └────────────────┘                          │
└─────────────────────────────────────────────────────┘
```

---

## Project Structure

### Current Structure (Preserved)
```
C:\new - Copy\physical-ai-robotics-textbook\
├── docusaurus/
│   ├── docs/                     # Chapters (PRESERVED)
│   │   ├── 00-introduction/
│   │   ├── 01-ros2/
│   │   ├── 02-simulation/
│   │   ├── 03-isaac/
│   │   ├── 04-vla/
│   │   └── 05-capstone/
│   ├── src/                      # Source files (ENHANCED)
│   │   ├── components/
│   │   └── css/
│   ├── docusaurus.config.mjs     # Config (MODIFIED)
│   └── package.json              # Dependencies (MODIFIED)
│
├── backend/                       # Backend API (ENHANCED)
└── auth-backend/                  # Auth system (EXISTS)
```

### New Structure (Added)
```
docusaurus/
├── src/
│   ├── components/
│   │   ├── LanguageToggle/           # NEW FOLDER
│   │   │   ├── index.tsx             # Language toggle component
│   │   │   └── styles.module.css     # Toggle styles
│   │   │
│   │   └── TranslatedContent/        # NEW FOLDER
│   │       ├── index.tsx             # Content wrapper
│   │       └── styles.module.css     # Content styles
│   │
│   ├── context/                      # NEW FOLDER
│   │   └── LanguageContext.tsx       # Language state management
│   │
│   └── translations/                 # NEW FOLDER
│       ├── en/                       # English translations
│       │   ├── 00-introduction/
│       │   ├── 01-ros2/
│       │   └── ...
│       ├── ur/                       # Urdu translations
│       │   ├── 00-introduction/
│       │   ├── 01-ros2/
│       │   └── ...
│       └── ar/                       # Arabic translations
│           ├── 00-introduction/
│           └── ...
│
└── static/
    └── fonts/                        # NEW FOLDER
        └── urdu/                     # Urdu fonts
            └── NotoNastaliqUrdu-Regular.ttf
```

---

## Component Specifications

### 1. Language Context (State Management)

**File:** `docusaurus/src/context/LanguageContext.tsx`

**Purpose:** Global state for selected language

**Implementation:**
```typescript
import React, { createContext, useContext, useState, useEffect } from 'react'

interface LanguageContextType {
  language: 'en' | 'ur' | 'ar'
  setLanguage: (lang: 'en' | 'ur' | 'ar') => void
  isAuthenticated: boolean
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined)

export function LanguageProvider({ children }: { children: React.ReactNode }) {
  const [language, setLanguageState] = useState<'en' | 'ur' | 'ar'>('en')
  const [isAuthenticated, setIsAuthenticated] = useState(false)

  // Check auth status on mount
  useEffect(() => {
    async function checkAuth() {
      try {
        const response = await fetch('/api/auth/session')
        const data = await response.json()
        setIsAuthenticated(data.isAuthenticated)
      } catch (error) {
        console.error('Auth check failed:', error)
        setIsAuthenticated(false)
      }
    }
    checkAuth()
  }, [])

  // Load saved language preference
  useEffect(() => {
    if (isAuthenticated) {
      const saved = localStorage.getItem('preferred-language')
      if (saved && ['en', 'ur', 'ar'].includes(saved)) {
        setLanguageState(saved as 'en' | 'ur' | 'ar')
      }
    }
  }, [isAuthenticated])

  const setLanguage = (lang: 'en' | 'ur' | 'ar') => {
    if (!isAuthenticated) {
      alert('Please log in to use translation feature')
      return
    }
    setLanguageState(lang)
    localStorage.setItem('preferred-language', lang)
  }

  return (
    <LanguageContext.Provider value={{ language, setLanguage, isAuthenticated }}>
      {children}
    </LanguageContext.Provider>
  )
}

export function useLanguage() {
  const context = useContext(LanguageContext)
  if (!context) {
    throw new Error('useLanguage must be used within LanguageProvider')
  }
  return context
}
```

---

### 2. Language Toggle Component

**File:** `docusaurus/src/components/LanguageToggle/index.tsx`

**Purpose:** Navbar dropdown for language selection

**Design:**
```
┌─────────────────────────────┐
│  🌐 English ▼               │
├─────────────────────────────┤
│  ✓ English                  │
│    اردو (Urdu)             │
│    العربية (Arabic)         │
└─────────────────────────────┘
```

**Implementation:**
```typescript
import React, { useState, useRef, useEffect } from 'react'
import { useLanguage } from '@site/src/context/LanguageContext'
import styles from './styles.module.css'

const languages = {
  en: { name: 'English', flag: '🇬🇧', nativeName: 'English' },
  ur: { name: 'Urdu', flag: '🇵🇰', nativeName: 'اردو' },
  ar: { name: 'Arabic', flag: '🇸🇦', nativeName: 'العربية' },
}

export default function LanguageToggle() {
  const { language, setLanguage, isAuthenticated } = useLanguage()
  const [isOpen, setIsOpen] = useState(false)
  const dropdownRef = useRef<HTMLDivElement>(null)

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false)
      }
    }
    document.addEventListener('mousedown', handleClickOutside)
    return () => document.removeEventListener('mousedown', handleClickOutside)
  }, [])

  const handleLanguageSelect = (lang: 'en' | 'ur' | 'ar') => {
    setLanguage(lang)
    setIsOpen(false)
  }

  return (
    <div className={styles.languageToggle} ref={dropdownRef}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        disabled={!isAuthenticated}
        title={isAuthenticated ? 'Select Language' : 'Login to change language'}
      >
        <span className={styles.globe}>🌐</span>
        <span className={styles.currentLang}>
          {languages[language].nativeName}
        </span>
        <span className={styles.arrow}>▼</span>
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          {Object.entries(languages).map(([code, lang]) => (
            <button
              key={code}
              className={`${styles.option} ${language === code ? styles.active : ''}`}
              onClick={() => handleLanguageSelect(code as 'en' | 'ur' | 'ar')}
            >
              <span className={styles.flag}>{lang.flag}</span>
              <span className={styles.name}>{lang.nativeName}</span>
              {language === code && <span className={styles.check}>✓</span>}
            </button>
          ))}
        </div>
      )}

      {!isAuthenticated && (
        <div className={styles.tooltip}>
          Login required for translation
        </div>
      )}
    </div>
  )
}
```

**Styles:** `docusaurus/src/components/LanguageToggle/styles.module.css`
```css
.languageToggle {
  position: relative;
  display: inline-block;
}

.toggleButton {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  background: var(--ifm-navbar-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  color: var(--ifm-navbar-link-color);
  transition: all 0.2s;
}

.toggleButton:hover:not(:disabled) {
  background: var(--ifm-color-emphasis-100);
  border-color: var(--ifm-color-primary);
}

.toggleButton:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.globe {
  font-size: 18px;
}

.currentLang {
  font-weight: 500;
}

.arrow {
  font-size: 10px;
  margin-left: 4px;
}

.dropdown {
  position: absolute;
  top: calc(100% + 8px);
  right: 0;
  min-width: 180px;
  background: var(--ifm-navbar-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  overflow: hidden;
}

.option {
  display: flex;
  align-items: center;
  gap: 12px;
  width: 100%;
  padding: 12px 16px;
  background: none;
  border: none;
  cursor: pointer;
  font-size: 14px;
  color: var(--ifm-font-color-base);
  transition: background 0.2s;
  text-align: left;
}

.option:hover {
  background: var(--ifm-color-emphasis-100);
}

.option.active {
  background: var(--ifm-color-primary-lightest);
  color: var(--ifm-color-primary);
  font-weight: 600;
}

.flag {
  font-size: 20px;
}

.name {
  flex: 1;
}

.check {
  color: var(--ifm-color-primary);
  font-weight: bold;
}

.tooltip {
  position: absolute;
  top: calc(100% + 8px);
  right: 0;
  padding: 8px 12px;
  background: var(--ifm-color-gray-900);
  color: white;
  font-size: 12px;
  border-radius: 6px;
  white-space: nowrap;
  pointer-events: none;
  z-index: 1001;
}

/* RTL support for Urdu/Arabic */
[dir='rtl'] .dropdown {
  left: 0;
  right: auto;
}

[dir='rtl'] .option {
  text-align: right;
}
```

---

### 3. Translated Content Wrapper

**File:** `docusaurus/src/components/TranslatedContent/index.tsx`

**Purpose:** Wraps MDX content and applies translations

**Implementation:**
```typescript
import React, { useEffect, useState } from 'react'
import { useLanguage } from '@site/src/context/LanguageContext'
import styles from './styles.module.css'

interface TranslatedContentProps {
  children: React.ReactNode
  originalContent: any
  chapterId: string
}

export default function TranslatedContent({
  children,
  originalContent,
  chapterId
}: TranslatedContentProps) {
  const { language } = useLanguage()
  const [translatedData, setTranslatedData] = useState<any>(null)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    async function loadTranslation() {
      if (language === 'en') {
        setTranslatedData(null)
        return
      }

      setLoading(true)
      try {
        // Load translation JSON
        const translation = await import(
          `@site/src/translations/${language}/${chapterId}.json`
        )
        setTranslatedData(translation.default)
      } catch (error) {
        console.error(`Failed to load ${language} translation:`, error)
        setTranslatedData(null)
      } finally {
        setLoading(false)
      }
    }

    loadTranslation()
  }, [language, chapterId])

  // If English or no translation, show original
  if (language === 'en' || !translatedData) {
    return <div className={styles.content}>{children}</div>
  }

  // Apply RTL for Urdu/Arabic
  const dir = language === 'ur' || language === 'ar' ? 'rtl' : 'ltr'
  const fontClass = language === 'ur' ? styles.urduFont :
                    language === 'ar' ? styles.arabicFont : ''

  return (
    <div
      className={`${styles.content} ${fontClass}`}
      dir={dir}
      data-language={language}
    >
      {loading ? (
        <div className={styles.loading}>Loading translation...</div>
      ) : (
        <TranslatedHTML content={translatedData} />
      )}
    </div>
  )
}

// Component to render translated HTML
function TranslatedHTML({ content }: { content: any }) {
  return (
    <div dangerouslySetInnerHTML={{ __html: content.html }} />
  )
}
```

**Styles:** `docusaurus/src/components/TranslatedContent/styles.module.css`
```css
.content {
  /* Preserve all existing styling */
}

.urduFont {
  font-family: 'Noto Nastaliq Urdu', serif;
  line-height: 2;
  letter-spacing: 0.02em;
}

.arabicFont {
  font-family: 'Noto Naskh Arabic', serif;
  line-height: 1.8;
}

.loading {
  text-align: center;
  padding: 40px;
  color: var(--ifm-color-emphasis-600);
  font-size: 16px;
}

/* RTL adjustments */
[dir='rtl'] {
  text-align: right;
}

[dir='rtl'] ul,
[dir='rtl'] ol {
  padding-right: 20px;
  padding-left: 0;
}

[dir='rtl'] .main-heading,
[dir='rtl'] .second-heading,
[dir='rtl'] .third-heading,
[dir='rtl'] .fourth-heading {
  text-align: right;
}

/* Preserve code blocks LTR */
[dir='rtl'] pre,
[dir='rtl'] code {
  direction: ltr;
  text-align: left;
}

/* Preserve Mermaid diagrams LTR */
[dir='rtl'] .mermaid {
  direction: ltr;
}
```

---

## Translation Data Structure

### JSON Format

**File:** `docusaurus/src/translations/ur/00-introduction/index.json`

```json
{
  "meta": {
    "title": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف",
    "description": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے ساتھ شروعات",
    "language": "ur",
    "chapter": "00-introduction",
    "lastUpdated": "2025-12-15"
  },
  "content": {
    "headings": {
      "introduction": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف",
      "learning-objectives": "سیکھنے کے مقاصد",
      "prerequisites": "ضروری شرائط",
      "module-overview": "ماڈیول کا جائزہ"
    },
    "paragraphs": {
      "intro-p1": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس ٹیکسٹ بک کے تعارفی ماڈیول میں خوش آمدید۔ یہ ماڈیول آپ کو اس شعبے، اس کی اہمیت، اور بنیادی تصورات کا جائزہ فراہم کرے گا جو آپ کو کورس میں آگے بڑھتے ہوئے سمجھنے کی ضرورت ہوگی۔"
    },
    "lists": {
      "learning-objectives": [
        "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے ساتھ اس کے تعلق کی تعریف کریں",
        "ہیومنائیڈ روبوٹ سسٹم کے اہم اجزاء کو سمجھیں",
        "ہیومنائیڈ روبوٹکس میں اہم چیلنجز اور مواقع کی شناخت کریں",
        "فزیکل اے آئی تحقیق کی بین الشعبہ نوعیت کو پہچانیں",
        "روبوٹکس پروگرامنگ کے لیے اپنے ترقیاتی ماحول کو ترتیب دیں"
      ]
    }
  },
  "html": "<h1 class='main-heading'>فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف</h1><div class='underline-class'></div>..."
}
```

---

## Translation Generation System

### Backend API for Translation

**File:** `backend/routers/translation.py` (NEW)

```python
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
import openai
import json

router = APIRouter(prefix="/api/translation", tags=["translation"])

class TranslationRequest(BaseModel):
    content: str
    source_lang: str = "en"
    target_lang: str
    chapter_id: str

@router.post("/translate")
async def translate_content(request: TranslationRequest):
    """
    Translate content using OpenAI GPT-4
    """
    try:
        prompt = f"""
        Translate the following educational content from {request.source_lang} to {request.target_lang}.

        CRITICAL RULES:
        1. Preserve ALL HTML tags and classes exactly as they are
        2. Only translate the text content between tags
        3. Keep all code blocks, diagrams, and technical terms in English
        4. Maintain the same structure and formatting
        5. Use professional, educational language

        Content to translate:
        {request.content}

        Return ONLY the translated HTML, no explanations.
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a professional technical translator specializing in educational content."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3
        )

        translated = response.choices[0].message.content

        return {
            "success": True,
            "translated_content": translated,
            "target_lang": request.target_lang,
            "chapter_id": request.chapter_id
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/available-languages")
async def get_available_languages():
    """
    Get list of available translation languages
    """
    return {
        "languages": [
            {"code": "en", "name": "English", "nativeName": "English"},
            {"code": "ur", "name": "Urdu", "nativeName": "اردو"},
            {"code": "ar", "name": "Arabic", "nativeName": "العربية"}
        ]
    }
```

---

## Integration with Docusaurus

### Update Docusaurus Config

**File:** `docusaurus/docusaurus.config.mjs`

```javascript
import {themes as prismThemes} from 'prism-react-renderer';

export default {
  // ... existing config

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'ar'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
      ar: {
        label: 'العربية',
        direction: 'rtl',
        htmlLang: 'ar-SA',
      },
    },
  },

  // Custom fields for translation
  customFields: {
    translationEnabled: true,
    requiresAuth: true,
  },

  // ... rest of config
}
```

---

### Add Language Provider to Root

**File:** `docusaurus/src/theme/Root.js`

```javascript
import React from 'react';
import { LanguageProvider } from '@site/src/context/LanguageContext';

export default function Root({children}) {
  return (
    <LanguageProvider>
      {children}
    </LanguageProvider>
  );
}
```

---

### Swizzle Navbar to Add Language Toggle

**File:** `docusaurus/src/theme/Navbar/index.js`

```javascript
import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import LanguageToggle from '@site/src/components/LanguageToggle';
import styles from './styles.module.css';

export default function Navbar(props) {
  return (
    <>
      <OriginalNavbar {...props} />
      <div className={styles.languageToggleContainer}>
        <LanguageToggle />
      </div>
    </>
  );
}
```

---

## Urdu Font Setup

### Add Noto Nastaliq Urdu Font

**File:** `docusaurus/src/css/custom.css`

```css
/* Import Urdu font */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/urdu/NotoNastaliqUrdu-Regular.ttf') format('truetype');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

/* Import Arabic font */
@font-face {
  font-family: 'Noto Naskh Arabic';
  src: url('https://fonts.googleapis.com/css2?family=Noto+Naskh+Arabic:wght@400;500;600;700&display=swap');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

/* Urdu content styling */
[data-language="ur"] {
  font-family: 'Noto Nastaliq Urdu', serif;
  line-height: 2;
  letter-spacing: 0.02em;
}

/* Arabic content styling */
[data-language="ar"] {
  font-family: 'Noto Naskh Arabic', serif;
  line-height: 1.8;
}

/* RTL support */
[dir="rtl"] {
  text-align: right;
}

[dir="rtl"] .navbar__item {
  margin-left: 0;
  margin-right: var(--ifm-navbar-item-padding-horizontal);
}
```

---

## Pre-generated Translations

### Translation JSON Files

For each chapter, create JSON files with pre-translated content:

```
src/translations/
├── ur/
│   ├── 00-introduction/
│   │   ├── index.json
│   │   ├── 01-welcome.json
│   │   ├── 02-prerequisites.json
│   │   ├── 03-hardware-requirements.json
│   │   ├── 04-how-to-use.json
│   │   └── 05-syllabus.json
│   ├── 01-ros2/
│   │   ├── index.json
│   │   └── ...
│   └── ...
│
└── ar/
    ├── 00-introduction/
    └── ...
```

**Sample Translation Mapping:**

| English | Urdu | Arabic |
|---------|------|--------|
| Introduction | تعارف | مقدمة |
| Learning Objectives | سیکھنے کے مقاصد | أهداف التعلم |
| Prerequisites | ضروری شرائط | المتطلبات |
| Physical AI | فزیکل اے آئی | الذكاء الاصطناعي المادي |
| Humanoid Robotics | ہیومنائیڈ روبوٹکس | روبوتات البشرية |

---

## Authentication Integration

### Check User Auth Before Translation

**File:** `docusaurus/src/context/LanguageContext.tsx` (Enhanced)

```typescript
useEffect(() => {
  async function checkAuthAndLoadPreference() {
    try {
      // Check if user is logged in
      const authResponse = await fetch('http://localhost:8001/api/auth/session', {
        credentials: 'include'
      })

      const authData = await authResponse.json()

      if (authData.isAuthenticated) {
        setIsAuthenticated(true)

        // Load user's language preference from backend
        const prefResponse = await fetch('http://localhost:8001/api/user/preferences', {
          credentials: 'include'
        })

        const prefData = await prefResponse.json()
        if (prefData.language) {
          setLanguageState(prefData.language)
        }
      } else {
        setIsAuthenticated(false)
        setLanguageState('en') // Default to English
      }
    } catch (error) {
      console.error('Auth check failed:', error)
      setIsAuthenticated(false)
    }
  }

  checkAuthAndLoadPreference()
}, [])
```

---

## Implementation Steps

### Phase 1: Setup (Day 1)
1. Create folder structure
2. Install dependencies
3. Add fonts
4. Update configs

### Phase 2: Core Components (Day 2)
1. Create LanguageContext
2. Create LanguageToggle
3. Create TranslatedContent
4. Integrate with Navbar

### Phase 3: Translations (Day 3-5)
1. Generate Urdu translations (Chapter 00)
2. Generate Urdu translations (Chapter 01-05)
3. Generate Arabic translations (optional)
4. Test all translations

### Phase 4: Testing (Day 6)
1. Test language switching
2. Test RTL layout
3. Test authentication
4. Fix any issues

---

## Success Criteria

✅ Language toggle in navbar
✅ 3 languages: English, Urdu, Arabic
✅ Works only for logged-in users
✅ Preserves ALL styling
✅ Perfect RTL support
✅ All chapters translated
✅ Instant switching
✅ No errors
✅ Professional UI
✅ 50/50 bonus points earned!

---

**Status:** 📋 Complete Constitution Ready
**Ready For:** Implementation
**Bonus Points:** 50/50 🏆

---

*End of Constitution Document*
```

## RESPONSE_TEXT

Created a new constitution file for the Multi-Language Translation System with Urdu support. The constitution includes:

1. Complete technical architecture for language translation system
2. Component specifications for LanguageContext, LanguageToggle, and TranslatedContent
3. Backend API for translation services
4. Integration with Docusaurus
5. Urdu font setup and RTL support
6. Authentication integration
7. Implementation phases and success criteria

The constitution was successfully written to `C:\new - Copy\.specify\memory\constitution.md` with all the required specifications for implementing the Urdu translation feature for the Physical AI & Humanoid Robotics Textbook.

## OUTCOME

Successfully created a comprehensive constitution document for the Urdu translation system that meets all requirements from the hackathon specification.

## EVALUATION

The constitution fully addresses the requirement for Urdu translation functionality with additional support for Arabic. It preserves all existing styling and layout while providing a professional, production-ready solution that can earn the 50 bonus points specified in the hackathon requirements.