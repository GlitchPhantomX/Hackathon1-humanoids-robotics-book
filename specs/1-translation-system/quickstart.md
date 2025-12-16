# Quickstart Guide: Multi-Language Translation System

## Overview
This guide provides a quick setup and implementation path for the multi-language translation system. Follow these steps to get the translation feature up and running in your Docusaurus documentation site.

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project set up
- Access to OpenAI API key (for translation generation)
- Backend API with authentication endpoints running at `http://localhost:8001`

## Installation Steps

### 1. Install Dependencies
```bash
cd physical-ai-robotics-textbook/docusaurus
npm install @docusaurus/core @docusaurus/preset-classic
```

### 2. Create Directory Structure
```bash
# Create necessary directories
mkdir -p src/components/LanguageToggle
mkdir -p src/components/TranslatedContent
mkdir -p src/context
mkdir -p src/translations/en
mkdir -p src/translations/ur
mkdir -p src/translations/ar
mkdir -p static/fonts/urdu
```

### 3. Add Font Files
Download the Noto Nastaliq Urdu font and place it in `static/fonts/urdu/NotoNastaliqUrdu-Regular.ttf`

### 4. Create Core Components

#### Create Language Context
Create `src/context/LanguageContext.tsx`:
```typescript
import React, { createContext, useContext, useState, useEffect } from 'react';

interface LanguageContextType {
  language: 'en' | 'ur' | 'ar';
  setLanguage: (lang: 'en' | 'ur' | 'ar') => void;
  isAuthenticated: boolean;
  loading: boolean;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

export function LanguageProvider({ children }: { children: React.ReactNode }) {
  const [language, setLanguageState] = useState<'en' | 'ur' | 'ar'>('en');
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);

  // Check auth status on mount
  useEffect(() => {
    async function checkAuth() {
      try {
        const response = await fetch('http://localhost:8001/api/auth/session', {
          credentials: 'include'
        });
        const data = await response.json();
        setIsAuthenticated(data.isAuthenticated);

        // Load saved language preference if authenticated
        if (data.isAuthenticated) {
          const saved = localStorage.getItem('preferred-language');
          if (saved && ['en', 'ur', 'ar'].includes(saved)) {
            setLanguageState(saved as 'en' | 'ur' | 'ar');
          }
        }
      } catch (error) {
        console.error('Auth check failed:', error);
        setIsAuthenticated(false);
      } finally {
        setLoading(false);
      }
    }
    checkAuth();
  }, []);

  const setLanguage = (lang: 'en' | 'ur' | 'ar') => {
    if (!isAuthenticated) {
      alert('Please log in to use translation feature');
      return;
    }
    setLanguageState(lang);
    localStorage.setItem('preferred-language', lang);
  };

  return (
    <LanguageContext.Provider value={{ language, setLanguage, isAuthenticated, loading }}>
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

#### Create Language Toggle Component
Create `src/components/LanguageToggle/index.tsx`:
```typescript
import React, { useState, useRef, useEffect } from 'react';
import { useLanguage } from '@site/src/context/LanguageContext';
import styles from './styles.module.css';

const languages = {
  en: { name: 'English', flag: '🇬🇧', nativeName: 'English', direction: 'ltr' },
  ur: { name: 'Urdu', flag: '🇵🇰', nativeName: 'اردو', direction: 'rtl' },
  ar: { name: 'Arabic', flag: '🇸🇦', nativeName: 'العربية', direction: 'rtl' },
};

export default function LanguageToggle() {
  const { language, setLanguage, isAuthenticated } = useLanguage();
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleLanguageSelect = (lang: 'en' | 'ur' | 'ar') => {
    setLanguage(lang);
    setIsOpen(false);
  };

  return (
    <div className={styles.languageToggle} ref={dropdownRef}>
      <button
        className={`${styles.toggleButton} ${!isAuthenticated ? styles.disabled : ''}`}
        onClick={() => isAuthenticated && setIsOpen(!isOpen)}
        disabled={!isAuthenticated}
        title={isAuthenticated ? 'Select Language' : 'Login to change language'}
        aria-label="Select language"
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <span className={styles.globe}>🌐</span>
        <span className={styles.currentLang}>
          {languages[language].nativeName}
        </span>
        <span className={styles.arrow}>▼</span>
      </button>

      {isAuthenticated && isOpen && (
        <div className={styles.dropdown} role="menu">
          {Object.entries(languages).map(([code, lang]) => (
            <button
              key={code}
              className={`${styles.option} ${language === code ? styles.active : ''}`}
              onClick={() => handleLanguageSelect(code as 'en' | 'ur' | 'ar')}
              role="menuitem"
              aria-selected={language === code}
            >
              <span className={styles.flag}>{lang.flag}</span>
              <span className={styles.name}>{lang.nativeName}</span>
              {language === code && <span className={styles.check}>✓</span>}
            </button>
          ))}
        </div>
      )}

      {!isAuthenticated && (
        <div className={styles.tooltip} role="tooltip">
          Login required for translation
        </div>
      )}
    </div>
  );
}
```

#### Create Language Toggle Styles
Create `src/components/LanguageToggle/styles.module.css`:
```css
.languageToggle {
  position: relative;
  display: inline-block;
  z-index: 100;
}

.toggleButton {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  background: var(--ifm-navbar-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  color: var(--ifm-navbar-link-color);
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.2s ease;
}

.toggleButton:hover:not(.disabled) {
  background: var(--ifm-color-emphasis-100);
  border-color: var(--ifm-color-primary);
  transform: translateY(-1px);
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.toggleButton.disabled {
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
  text-align: left;
  transition: background 0.2s ease;
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

/* RTL support */
[dir='rtl'] .dropdown {
  right: auto;
  left: 0;
}

[dir='rtl'] .option {
  text-align: right;
}
```

### 5. Update Docusaurus Configuration
Update `docusaurus.config.mjs` to add i18n configuration:
```javascript
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
  // ... rest of config
};
```

### 6. Add Root Provider
Create `src/theme/Root.js` to wrap the app with LanguageProvider:
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

### 7. Add Custom CSS for Fonts and RTL
Update `src/css/custom.css` to include font imports:
```css
/* Import Urdu font */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/urdu/NotoNastaliqUrdu-Regular.ttf') format('truetype');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

/* Urdu content styling */
.urdu-font {
  font-family: 'Noto Nastaliq Urdu', serif !important;
  line-height: 2.2 !important;
  letter-spacing: 0.02em !important;
  word-spacing: 0.1em !important;
}

/* Arabic content styling */
.arabic-font {
  font-family: 'Noto Naskh Arabic', serif !important;
  line-height: 1.8 !important;
  letter-spacing: 0.01em !important;
}

/* RTL-specific styles */
[dir='rtl'] {
  text-align: right;
}

[dir='rtl'] .main-heading,
[dir='rtl'] .second-heading,
[dir='rtl'] .third-heading,
[dir='rtl'] .fourth-heading {
  text-align: right !important;
}

[dir='rtl'] ul,
[dir='rtl'] ol {
  padding-right: 20px !important;
  padding-left: 0 !important;
}

[dir='rtl'] li {
  text-align: right;
}

/* Preserve LTR for code and diagrams */
[dir='rtl'] pre,
[dir='rtl'] code,
[dir='rtl'] .mermaid,
[dir='rtl'] .codeBlockContainer {
  direction: ltr !important;
  text-align: left !important;
}
```

## Adding Translation Files

### Generate Translation JSON
For each chapter, create a corresponding JSON file in the appropriate language folder. Example for `src/translations/ur/00-introduction/index.json`:

```json
{
  "meta": {
    "title": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف",
    "description": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے ساتھ شروعات",
    "language": "ur",
    "chapter": "00-introduction",
    "sourceFile": "00-introduction/index.md",
    "lastUpdated": "2025-12-15T00:00:00Z",
    "translator": "OpenAI GPT-4"
  },
  "content": {
    "headings": {
      "introduction": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف",
      "learning-objectives": "سیکھنے کے مقاصد",
      "prerequisites": "ضروری شرائط"
    },
    "paragraphs": {
      "intro-p1": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس ٹیکس بک کے تعارفی ماڈیول میں خوش آمدید۔"
    },
    "lists": {
      "learning-objectives": [
        "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے ساتھ اس کے تعلق کی تعریف کریں",
        "ہیومنائیڈ روبوٹ سسٹم کے اہم اجزاء کو سمجھیں"
      ]
    }
  },
  "html": "<h1 class='main-heading' id='introduction'>فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کا تعارف</h1><div class='underline-class'></div>..."
}
```

## Testing the Implementation

### Basic Functionality Test
1. Start your Docusaurus development server: `npm run start`
2. Verify the language toggle appears in the navbar
3. Check that the toggle is disabled when not logged in
4. Log in and verify the toggle becomes enabled
5. Test language switching and verify content updates

### Authentication Test
1. Navigate to the site without logging in
2. Verify the language toggle is disabled
3. Hover over the toggle to see the "Login required" message
4. Log in and verify the toggle becomes functional

### RTL Test
1. Switch to Urdu or Arabic
2. Verify text alignment is right-aligned
3. Verify code blocks remain left-aligned
4. Check that layout elements are properly positioned

## Environment Variables
Add to your `.env` file:
```bash
OPENAI_API_KEY=your_openai_api_key_here
AUTH_API_URL=http://localhost:8001
```

## Build and Deployment
```bash
npm run build
npm run serve
```

The translation system will be fully functional in the production build with all languages available and authentication requirements enforced.