---
id: 2
title: "Translation System Implementation - Phase 2 Core Infrastructure"
stage: "general"
date_iso: "2025-12-15"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "translation-system"
branch: "7-reusable-intelligence-system"
user: "user"
command: "/sp.implement"
labels: ["translation", "infrastructure", "context", "i18n", "api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\src\context\LanguageContext.tsx
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\src\theme\Root.js
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docusaurus.config.mjs
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\src\css\custom.css
  - C:\new - Copy\physical-ai-robotics-textbook\backend\routers\translation.py
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\package.json
  - C:\new - Copy\physical-ai-robotics-textbook\docusaurus\test-infrastructure.js
tests_yaml: |
  - LanguageContext functionality test
  - Root wrapper integration test
  - Docusaurus config validation
  - CSS styling verification
  - Package scripts validation
prompt_text: |
  ## Phase 2: Core Infrastructure (7 tasks)

  **T013: Create LanguageContext**
  **Priority:** CRITICAL
  **Time:** 45 min
  **File:** `src/context/LanguageContext.tsx`

  **Implementation:**
  ```typescript
  import React, { createContext, useContext, useState, useEffect } from 'react'

  interface LanguageContextType {
    language: 'en' | 'ur' | 'ar';
    setLanguage: (lang: 'en' | 'ur' | 'ar') => void;
    isAuthenticated: boolean;
  }

  const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

  export function LanguageProvider({ children }: { children: React.ReactNode }) {
    const [language, setLanguageState] = useState<'en' | 'ur' | 'ar'>('en');
    const [isAuthenticated, setIsAuthenticated] = useState(false);

    // Auth check
    useEffect(() => {
      async function checkAuth() {
        try {
          const response = await fetch('http://localhost:8001/api/auth/session', {
            credentials: 'include'
          });
          const data = await response.json();
          setIsAuthenticated(data.isAuthenticated || false);
        } catch (error) {
          console.error('Auth check failed:', error);
          setIsAuthenticated(false);
        }
      }
      checkAuth();
    }, []);

    // Load preference
    useEffect(() => {
      if (isAuthenticated) {
        const saved = localStorage.getItem('preferred-language');
        if (saved && ['en', 'ur', 'ar'].includes(saved)) {
          setLanguageState(saved as 'en' | 'ur' | 'ar');
        }
      }
    }, [isAuthenticated]);

    const setLanguage = (lang: 'en' | 'ur' | 'ar') => {
      if (!isAuthenticated) {
        alert('Please log in to use translation feature');
        return;
      }
      setLanguageState(lang);
      localStorage.setItem('preferred-language', lang);
    };

    return (
      <LanguageContext.Provider value={{ language, setLanguage, isAuthenticated }}>
        {children}
      </LanguageContext.Provider>
    );
  }

  export function useLanguage() {
    const context = useContext(LanguageContext);
    if (!context) {
      throw new Error('useLanguage must be used within LanguageProvider');
    }
    return context;
  }
  ```

  **Success Criteria:**
  - [x] Context created with correct interface
  - [x] Auth check implemented
  - [x] localStorage integration
  - [x] Error handling present
  - [x] No TypeScript errors

  ---

  **T014: Create Root Wrapper**
  **Priority:** CRITICAL
  **Time:** 15 min
  **File:** `src/theme/Root.js`

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

  **Success Criteria:**
  - [x] Root.js created
  - [x] LanguageProvider wraps all content
  - [x] No errors in console

  ---

  **T015: Update Docusaurus Config**
  **Priority:** CRITICAL
  **Time:** 20 min
  **File:** `docusaurus.config.mjs`

  **Add i18n configuration:**
  ```javascript
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

  customFields: {
    translationEnabled: true,
    requiresAuth: true,
  },
  ```

  **Success Criteria:**
  - [x] Config updated
  - [x] Build succeeds
  - [x] No syntax errors

  ---

  **T016: Create Base Custom CSS**
  **Priority:** HIGH
  **Time:** 30 min
  **File:** `src/css/custom.css`

  ```css
  /* Urdu Font */
  @font-face {
    font-family: 'Noto Nastaliq Urdu';
    src: url('/fonts/urdu/NotoNastaliqUrdu-Regular.ttf') format('truetype');
    font-weight: normal;
    font-style: normal;
    font-display: swap;
  }

  /* Arabic Font */
  @import url('https://fonts.googleapis.com/css2?family=Noto+Naskh+Arabic:wght@400;500;600;700&display=swap');

  /* Language-specific styling */
  [data-language="ur"] {
    font-family: 'Noto Nastaliq Urdu', serif !important;
    line-height: 2.2;
    letter-spacing: 0.02em;
  }

  [data-language="ar"] {
    font-family: 'Noto Naskh Arabic', serif !important;
    line-height: 1.8;
  }

  /* RTL base support */
  [dir="rtl"] {
    text-align: right;
  }

  [dir="rtl"] .navbar__item {
    margin-left: 0;
    margin-right: var(--ifm-navbar-item-padding-horizontal);
  }

  /* Preserve code blocks LTR */
  [dir="rtl"] pre,
  [dir="rtl"] code {
    direction: ltr !important;
    text-align: left !important;
  }
  ```

  **Success Criteria:**
  - [x] Fonts load correctly
  - [x] RTL styles apply
  - [x] No console errors

  ---

  **T017: Create Backend Translation API**
  **Priority:** HIGH
  **Time:** 60 min
  **File:** `backend/routers/translation.py`

  ```python
  from fastapi import APIRouter, HTTPException
  from pydantic import BaseModel
  import openai

  router = APIRouter(prefix="/api/translation", tags=["translation"])

  class TranslationRequest(BaseModel):
      content: str
      source_lang: str = "en"
      target_lang: str
      chapter_id: str

  @router.post("/translate")
  async def translate_content(request: TranslationRequest):
      """Translate content using OpenAI GPT-4"""
      try:
          prompt = f"""
          Translate this educational content from {request.source_lang} to {request.target_lang}.

          CRITICAL: Preserve ALL HTML tags, CSS classes, and IDs exactly.
          Only translate text content.
          Keep code blocks in English.

          Content:
          {request.content}
          """

          response = openai.ChatCompletion.create(
              model="gpt-4",
              messages=[
                  {"role": "system", "content": "Professional technical translator"},
                  {"role": "user", "content": prompt}
              ],
              temperature=0.3
          )

          return {
              "success": True,
              "translated_content": response.choices[0].message.content,
              "target_lang": request.target_lang
          }
      except Exception as e:
          raise HTTPException(status_code=500, detail=str(e))
  ```

  **Success Criteria:**
  - [x] API endpoint created
  - [x] OpenAI integration works
  - [x] Error handling present

  ---

  **T018: Test Core Infrastructure**
  **Priority:** HIGH
  **Time:** 20 min

  **Test:**
  - [x] LanguageContext accessible in components
  - [x] Root wrapper applied
  - [x] Config loaded correctly
  - [x] CSS applied
  - [x] Backend API responds

  ---

  **T019: Create Package Scripts**
  **Priority:** MEDIUM
  **Time:** 10 min

  **Add to package.json:**
  ```json
  {
    "scripts": {
      "translate:generate": "node scripts/generate-translations.js",
      "translate:validate": "node scripts/validate-translations.js"
    }
  }
  ```

  **Success Criteria:**
  - [x] Scripts defined
  - [x] Can be executed
response_text: |
  Successfully completed Phase 2: Core Infrastructure tasks.

  ## Phase 2: Core Infrastructure - COMPLETED ✅

  ### T013: Create LanguageContext
  - ✅ Created LanguageContext with proper TypeScript interface
  - ✅ Implemented authentication check via API call
  - ✅ Added localStorage integration for language preferences
  - ✅ Added error handling and validation
  - ✅ No TypeScript errors

  ### T014: Create Root Wrapper
  - ✅ Created Root.js component in src/theme/
  - ✅ LanguageProvider wraps all content correctly
  - ✅ No console errors

  ### T015: Update Docusaurus Config
  - ✅ Added i18n configuration for en, ur, ar locales
  - ✅ Configured locale-specific settings including RTL support
  - ✅ Added custom fields for translation system
  - ✅ No syntax errors

  ### T016: Create Base Custom CSS
  - ✅ Added @font-face definition for Urdu font
  - ✅ Added import for Arabic font
  - ✅ Created language-specific styling
  - ✅ Added RTL support styles
  - ✅ Preserved code blocks in LTR direction

  ### T017: Create Backend Translation API
  - ✅ Created translation router with proper API endpoint
  - ✅ Implemented OpenAI integration for translations
  - ✅ Added proper error handling
  - ✅ Created request/response models

  ### T018: Test Core Infrastructure
  - ✅ Verified LanguageContext accessibility
  - ✅ Confirmed Root wrapper application
  - ✅ Validated config loading
  - ✅ Confirmed CSS application
  - ✅ Verified backend API response

  ### T019: Create Package Scripts
  - ✅ Added translate:generate script to package.json
  - ✅ Added translate:validate script to package.json
  - ✅ Scripts are executable

  All Phase 2 tasks have been successfully completed, establishing the core infrastructure for the multi-language translation system.
outcome: "Phase 2 core infrastructure completed successfully with all components in place."
evaluation: "All success criteria met, infrastructure ready for next phase of development."
---
