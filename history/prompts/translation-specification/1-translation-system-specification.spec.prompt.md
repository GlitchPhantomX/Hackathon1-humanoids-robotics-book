---
id: 1
title: "translation-system-specification"
stage: "spec"
date: "2025-12-30"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "translation-specification"
branch: "main"
user: "user"
command: "/sp.specify"
labels: ["specification", "translation", "i18n", "docusaurus"]
links:
  spec: "specs/translation-specification/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/translation-specification/spec.md"
tests: []
outcome: "Successfully created comprehensive multi-language translation system specification for Physical AI & Humanoid Robotics textbook"
evaluation: "Specification includes all required components: user scenarios, functional requirements, technical architecture, implementation plan, and quality assurance measures for Urdu and Hindi language support"
---

# Prompt

```
# Multi-Language Translation System Specification

## Document Information
- **Project**: Physical AI & Humanoid Robotics Textbook
- **Feature**: Multi-Language Translation System
- **Version**: 1.0.0
- **Date**: December 30, 2025
- **Author**: System Architect
- **Status**: Implementation Ready

---

## 1. Executive Summary

### 1.1 Overview
Implementation of a comprehensive multi-language translation system for the Physical AI & Humanoid Robotics educational platform. The system will support English (existing), Urdu, and Hindi languages with seamless switching capabilities while maintaining exact styling, functionality, and user experience.

### 1.2 Business Objectives
- **Primary**: Enable Urdu translation for 50 bonus hackathon points
- **Secondary**: Add Hindi for broader accessibility
- **Tertiary**: Establish scalable i18n infrastructure

### 1.3 Success Criteria
```yaml
Must Have:
  - ✓ Complete Urdu translation of all chapters
  - ✓ Translation button at start of each chapter
  - ✓ Language dro   │   ├── TranslateButton.tsx     # NEW
│   │   │   ├── LanguageDropdown.tsx    # NEW (optional)
│   │   │   ├── PersonalizeButton.tsx   # PROTECTED
│   │   │   ├── Chatbot.tsx             # PROTECTED
│   │   │   └── ...
│   │   │
│   │   ├── css/
│   │   │   ├── custom.css              # MODIFY: Add RTL styles
│   │   │   └── translation.css         # NEW
│   │   │
│   │   └── utils/
│   │       └── i18n.ts                 # NEW: Translation utilities
│   │
│   ├── static/
│   │   └── fonts/                      # NEW: Custom fonts
│   │       ├── NotoNastaliqUrdu/
│   │       └── NotoSansDevanagari/
│   │
│   └── auth-backend/                   # PROTECTED: Do not modify
│       └── ...
│
└── rag-chatbot/                        # PROTECTED: Do not modify
    └── ...
```

---

## 3. Implementation Specifications

### 3.1 Docusaurus Configuration

#### 3.1.1 docusaurus.config.mjs Updates

```javascript
// File: docusaurus/docusaurus.config.mjs

import {themes as prismThemes} from 'prism-react-renderer';

export default {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master the fundamentals of intelligent physical systems',
  favicon: 'img/favicon.ico',
  url: 'https://your-domain.com',
  baseUrl: '/',

  // ==================== NEW: i18n Configuration ====================
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'hi'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
        calendar: 'gregory',
        path: 'en',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
        calendar: 'gregory',
        path: 'ur',
      },
      hi: {
        label: 'हिंदी',
        direction: 'ltr',
        htmlLang: 'hi-IN',
        calendar: 'gregory',
        path: 'hi',
      },
    },
  },
  // ================================================================

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-repo/edit/main/',
          // NEW: Multi-instance docs support
          routeBasePath: 'docs',
          path: 'docs',
        },
        theme: {
          customCss: [
            './src/css/custom.css',
            './src/css/translation.css', // NEW
          ],
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        // ==================== NEW: Language Dropdown ====================
        {
          type: 'localeDropdown',
          position: 'right',
          dropdownItemsAfter: [],
          className: 'language-dropdown-custom',
          queryString: '?persistLocale=true',
        },
        // ================================================================
        // Add spacing before auth buttons
        {
          type: 'html',
          position: 'right',
          value: '<div style="width: 20px;"></div>',
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
          className: 'navbar-login-btn',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'navbar-signup-btn',
        },
      ],
    },

    // Color mode configuration
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },

  // ==================== NEW: Custom Webpack Config ====================
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          target: 'es2017',
        },
        module: {
          type: isServer ? 'commonjs' : 'es6',
        },
      },
    }),
  },
  // ====================================================================

  // ==================== NEW: Plugin Configuration ====================
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs',
        path: 'docs',
        routeBasePath: 'docs',
        sidebarPath: require.resolve('./sidebars.js'),
        remarkPlugins: [],
        rehypePlugins: [],
      },
    ],
  ],
  // ===================================================================
};
```

#### 3.1.2 Package.json Updates

```json
{
  "name": "physical-ai-robotics-textbook",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "serve": "docusaurus serve",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids",
    "start:ur": "docusaurus start --locale ur",
    "start:hi": "docusaurus start --locale hi",
    "build:ur": "docusaurus build --locale ur",
    "build:hi": "docusaurus build --locale hi"
  },
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@docusaurus/theme-live-codeblock": "^3.0.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.1.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.0.0",
    "@docusaurus/types": "^3.0.0",
    "typescript": "^5.2.2"
  },
  "browserslist": {
    "production": [
      ">0.5%",
      "not dead",
      "not op_mini all"
    ],
    "development": [
      "last 1 chrome version",
      "last 1 firefox version",
      "last 1 safari version"
    ]
  },
  "engines": {
    "node": ">=18.0"
  }
}
```

### 3.2 Component Specifications

#### 3.2.1 TranslateButton Component

```typescript
// File: src/components/TranslateButton.tsx

import React, { useState, useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  compact?: boolean;
  showAllLanguages?: boolean;
}

export default function TranslateButton({
  compact = false,
  showAllLanguages = true
}: TranslateButtonProps): JSX.Element {
  const { i18n } = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();
  const [currentLocale, setCurrentLocale] = useState(i18n.currentLocale);

  useEffect(() => {
    setCurrentLocale(i18n.currentLocale);
  }, [i18n.currentLocale]);

  const switchLanguage = (targetLocale: string) => {
    if (targetLocale === currentLocale) return;

    // Get current path without locale prefix
    const pathWithoutLocale = location.pathname.replace(
      /^\/(en|ur|hi)\//,
      '/'
    );

    // Construct new path with target locale
    const newPath = targetLocale === 'en'
      ? pathWithoutLocale
      : `/${targetLocale}${pathWithoutLocale}`;

    // Save preference to localStorage
    localStorage.setItem('preferredLocale', targetLocale);

    // Navigate to new path
    history.push(newPath);
  };

  const languages = [
    { code: 'en', label: 'English', nativeLabel: 'English', flag: '🇬🇧' },
    { code: 'ur', label: 'Urdu', nativeLabel: 'اردو', flag: '🇵🇰' },
    { code: 'hi', label: 'Hindi', nativeLabel: 'हिंदी', flag: '🇮🇳' },
  ];

  const otherLanguages = languages.filter(lang => lang.code !== currentLocale);

  if (compact) {
    return (
      <div className={styles.translateButtonCompact}>
        {otherLanguages.map((lang) => (
          <button
            key={lang.code}
            className={styles.compactButton}
            onClick={() => switchLanguage(lang.code)}
            aria-label={`Switch to ${lang.label}`}
          >
            {lang.flag} {lang.nativeLabel}
          </button>
        ))}
      </div>
    );
  }

  return (
    <div className={styles.translateButtonContainer}>
      <div className={styles.translateButtonWrapper}>
        <div className={styles.currentLanguage}>
          <span className={styles.languageIcon}>🌐</span>
          <span className={styles.languageText}>
            {currentLocale === 'en' && 'Reading in English'}
            {currentLocale === 'ur' && 'اردو میں پڑھ رہے ہیں'}
            {currentLocale === 'hi' && 'हिंदी में पढ़ रहे हैं'}
          </span>
        </div>

        {showAllLanguages && (
          <div className={styles.languageButtons}>
            {otherLanguages.map((lang) => (
              <button
                key={lang.code}
                className={styles.languageButton}
                onClick={() => switchLanguage(lang.code)}
                aria-label={`Switch to ${lang.label}`}
              >
                <span className={styles.buttonFlag}>{lang.flag}</span>
                <span className={styles.buttonText}>
                  {lang.code === 'ur' && 'اردو میں پڑھیں'}
                  {lang.code === 'hi' && 'हिंदी में पढ़ें'}
                  {lang.code === 'en' && 'Read in English'}
                </span>
              </button>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
```

#### 3.2.2 TranslateButton Styles

```css
/* File: src/components/TranslateButton.module.css */

.translateButtonContainer {
  margin: 2rem 0;
  padding: 1.5rem;
  background: linear-gradient(135deg, #fff5f0 0%, #ffffff 100%);
  border-radius: 12px;
  border: 2px solid #ff6b35;
  box-shadow: 0 4px 6px rgba(255, 107, 53, 0.1);
}

.translateButtonWrapper {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.currentLanguage {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 1rem;
  color: #333;
  font-weight: 500;
}

.languageIcon {
  font-size: 1.5rem;
}

.languageText {
  font-size: 1.1rem;
}

.languageButtons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
}

.languageButton {
  flex: 1;
  min-width: 150px;
  padding: 0.75rem 1.5rem;
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  box-shadow: 0 2px 4px rgba(255, 107, 53, 0.2);
}

.languageButton:hover {
  background: linear-gradient(135deg, #ff8c42 0%, #ff6b35 100%);
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(255, 107, 53, 0.3);
}

.languageButton:active {
  transform: translateY(0);
}

.buttonFlag {
  font-size: 1.2rem;
}

.buttonText {
  font-size: 1rem;
}

/* Compact variant */
.translateButtonCompact {
  display: inline-flex;
  gap: 0.5rem;
  margin: 0.5rem 0;
}

.compactButton {
  padding: 0.5rem 1rem;
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  color: white;
  border: none;
  border-radius: 6px;
  font-size: 0.9rem;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  gap: 0.3rem;
}

.compactButton:hover {
  background: linear-gradient(135deg, #ff8c42 0%, #ff6b35 100%);
  transform: scale(1.05);
}

/* RTL Support */
[dir='rtl'] .translateButtonWrapper {
  direction: rtl;
}

[dir='rtl'] .currentLanguage {
  flex-direction: row-reverse;
}

[dir='rtl'] .languageButton {
  flex-direction: row-reverse;
}

/* Responsive Design */
@media (max-width: 768px) {
  .translateButtonContainer {
    padding: 1rem;
  }

  .languageButtons {
    flex-direction: column;
  }

  .languageButton {
    min-width: 100%;
  }
}

/* Dark Mode Support */
[data-theme='dark'] .translateButtonContainer {
  background: linear-gradient(135deg, #2a2a2a 0%, #1a1a1a 100%);
  border-color: #ff8c42;
}

[data-theme='dark'] .currentLanguage {
  color: #e0e0e0;
}

/* Animation */
@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateY(-10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.translateButtonContainer {
  animation: slideIn 0.3s ease-out;
}
```

### 3.3 CSS Specifications

#### 3.3.1 RTL Support Styles

```css
/* File: src/css/translation.css */

/* ====================== RTL Support ====================== */

/* Urdu RTL Support */
[dir='rtl'] {
  direction: rtl;
  text-align: right;
}

[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

[dir='rtl'] .navbar__item {
  padding-right: 0;
  padding-left: var(--ifm-navbar-item-padding-horizontal);
}

[dir='rtl'] .menu {
  padding-right: 0;
  padding-left: var(--ifm-menu-link-padding-horizontal);
}

[dir='rtl'] .menu__link {
  padding-right: 0;
  padding-left: var(--ifm-menu-link-padding-horizontal);
}

[dir='rtl'] .pagination-nav__link {
  flex-direction: row-reverse;
}

[dir='rtl'] .table-of-contents {
  padding-right: 0;
  padding-left: var(--ifm-toc-padding-horizontal);
}

[dir='rtl'] .markdown > h2,
[dir='rtl'] .markdown > h3,
[dir='rtl'] .markdown > h4 {
  text-align: right;
}

[dir='rtl'] .markdown ul,
[dir='rtl'] .markdown ol {
  padding-right: 2rem;
  padding-left: 0;
}

/* ====================== Language-Specific Fonts ====================== */

/* Urdu Typography */
html[lang='ur-PK'],
[lang='ur'] {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
}

html[lang='ur-PK'] .main-heading,
[lang='ur'] .main-heading {
  font-family: 'Noto Nastaliq Urdu', serif;
  font-size: 2.5rem;
  line-height: 1.8;
}

html[lang='ur-PK'] .second-heading,
[lang='ur'] .second-heading {
  font-family: 'Noto Nastaliq Urdu', serif;
  font-size: 2rem;
  line-height: 1.8;
}

html[lang='ur-PK'] .third-heading,
[lang='ur'] .third-heading {
  font-family: 'Noto Nastaliq Urdu', serif;
  font-size: 1.5rem;
  line-height: 1.8;
}

html[lang='ur-PK'] code,
[lang='ur'] code {
  font-family: 'Courier New', monospace;
  direction: ltr;
  text-align: left;
}

/* Hindi Typography */
html[lang='hi-IN'],
[lang='hi'] {
  font-family: 'Noto Sans Devanagari', 'Mangal', sans-serif;
}

html[lang='hi-IN'] .main-heading,
[lang='hi'] .main-heading {
  font-family: 'Noto Sans Devanagari', sans-serif;
  font-size: 2.5rem;
  line-height: 1.6;
}

html[lang='hi-IN'] .second-heading,
[lang='hi'] .second-heading {
  font-family: 'Noto Sans Devanagari', sans-serif;
  font-size: 2rem;
  line-height: 1.6;
}

html[lang='hi-IN'] .third-heading,
[lang='hi'] .third-heading {
  font-family: 'Noto Sans Devanagari', sans-serif;
  font-size: 1.5rem;
  line-height: 1.6;
}

/* ====================== Language Dropdown Styling ====================== */

.language-dropdown-custom {
  margin-right: 20px;
}

.navbar__link--active .dropdown__menu {
  background: linear-gradient(135deg, #fff5f0 0%, #ffffff 100%);
  border: 2px solid #ff6b35;
}

.dropdown__link {
  padding: 0.5rem 1rem;
  transition: all 0.2s ease;
}

.dropdown__link:hover {
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  color: white;
}

.dropdown__link--active {
  background: #ff6b35;
  color: white;
  font-weight: 600;
}

/* Flag Icons in Dropdown */
.dropdown__link::before {
  content: '';
  display: inline-block;
  width: 20px;
  margin-right: 8px;
}

/* ====================== Preserve Existing Styles ====================== */

/* Main Heading */
.main-heading {
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  font-weight: 700;
  margin-bottom: 1rem;
}

/* Second Heading */
.second-heading {
  color: #ff6b35;
  font-weight: 600;
  margin-top: 2rem;
  margin-bottom: 1rem;
}

/* Third Heading */
.third-heading {
  color: #ff8c42;
  font-weight: 600;
  margin-top: 1.5rem;
  margin-bottom: 0.75rem;
}

/* Underline Class */
.underline-class {
  width: 60px;
  height: 4px;
  background: linear-gradient(90deg, #ff6b35 0%, #ff8c42 100%);
  margin: 0.5rem 0 1.5rem 0;
  border-radius: 2px;
}

[dir='rtl'] .underline-class {
  margin-left: auto;
  margin-right: 0;
}

/* Border Line */
.border-line {
  border-bottom: 1px solid rgba(255, 107, 53, 0.2);
  margin: 2rem 0;
}

/* Full Content */
.full-content {
  display: block;
}

/* Summary Content */
.summary-content {
  display: none;
  background: linear-gradient(135deg, #fff5f0 0%, #ffffff 100%);
  border-left: 4px solid #ff6b35;
  padding: 1.5rem;
  margin-top: 2rem;
  border-radius: 8px;
}

@media (max-width: 996px) {
  .full-content {
    display: none;
  }

  .summary-content {
    display: block;
  }
}

/* ====================== Code Block Adjustments ====================== */

[dir='rtl'] .prism-code {
  direction: ltr;
  text-align: left;
}

[dir='rtl'] pre {
  direction: ltr;
  text-align: left;
}

/* ====================== Admonition Support ====================== */

[dir='rtl'] .admonition {
  padding-right: 1rem;
  padding-left: 0;
}

[dir='rtl'] .admonition-icon {
  margin-right: 0;
  margin-left: 0.5rem;
}

/* ====================== Responsive Adjustments ====================== */

@media (max-width: 996px) {
  .language-dropdown-custom {
    margin-right: 0;
    margin-bottom: 0.5rem;
  }

  [dir='rtl'] .navbar__items {
    flex-direction: column;
  }
}

/* ====================== Print Styles ====================== */

@media print {
  .translateButtonContainer {
    display: none;
  }

  .language-dropdown-custom {
    display: none;
  }
}
```

### 3.4 Translation File Specifications

#### 3.4.1 Urdu Category JSON Example

```json
// File: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/_category_.json

{
  "label": "تعارف",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کے بارے میں سیکھنے کا آغاز کریں"
  }
}
```

#### 3.4.2 Urdu Welcome Chapter Example

```markdown
<!-- File: i18n/ur/docusaurus-plugin-content-docs/current/00-introduction/01-welcome.md -->

---
sidebar_position: 1
title: "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس میں خوش آمدید"
description: "فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کی نصابی کتاب کا تعارف، بنیادی تصورات اور سیکھنے کا طریقہ"
---

import ReadingTime from '@site/src/components/ReadingTime';
import PersonalizeButton from "@site/src/components/PersonalizeButton"
import TranslateButton from "@site/src/components/TranslateButton"

<PersonalizeButton/>
<TranslateButton/>

<ReadingTime minutes={8} />

# <h1 className="main-heading">فزیکل اے آئی اور ہیومنائیڈ روبوٹکس میں خوش آمدید</h1>
<div className="underline-class"></div>

<div className="full-content">

**ماڈیول**: 00 - تعارف
**سیکھنے کے مقاصد**:
- • فزیکل اے آئی کے بنی-docs/current
mkdir -p i18n/ur/docusaurus-theme-classic
mkdir -p i18n/hi/docusaurus-theme-classic

# Step 3: Initialize translation files
npm run write-translations -- --locale ur
npm run write-translations -- --locale hi

# Step 4: Download and install fonts
mkdir -p static/fonts
# Download Noto Nastaliq Urdu and Noto Sans Devanagari
```

### 4.2 Phase 2: Configuration (Day 3)

```bash
# Step 1: Update docusaurus.config.mjs
# Add i18n configuration as specified

# Step 2: Create TranslateButton component
# Create src/components/TranslateButton.tsx
# Create src/components/TranslateButton.module.css

# Step 3: Add translation.css
# Create src/css/translation.css

# Step 4: Test basic setup
npm run start
npm run start:ur
npm run start:hi
```

### 4.3 Phase 3: Content Translation (Days 4-10)

```bash
# Priority order:
# 1. 00-introduction/ (7 files) - Days 4-5
# 2. 01-ros2/ (8 files) - Days 6-7
# 3. 02-simulation/ - Days 8-9
# 4. 03-isaac/ - Day 10
# 5. 04-vla/ - Day 11
# 6. 05-capstone/ - Day 12

# Translation workflow for each file:
# 1. Copy English file to target locale
# 2. Translate content (preserve all structure)
# 3. Add TranslateButton import and component
# 4. Test rendering
# 5. Verify styling
# 6. Check navigation
```

### 4.4 Phase 4: Testing & QA (Days 13-14)

```bash
# Test checklist:
- [ ] Build succeeds for all locales
- [ ] All pages render correctly
- [ ] Language switching works
- [ ] RTL support for Urdu
- [ ] Fonts load correctly
- [ ] Styling preserved
- [ ] Navigation works
- [ ] Auth unaffected
- [ ] Chatbot unaffected
- [ ] Mobile responsive
```

---

## 5. Quality Assurance

### 5.1 Translation Quality Standards

```yaml
Content Accuracy:
  - Technical terms correctly translated
  - Context preserved
  - No meaning loss
  - Cultural appropriateness

Structural Integrity:
  - All frontmatter preserved
  - All imports maintained
  - All components functional
  - All links working

Visual Consistency:
  - Styling matches English
  - Colors identical
  - Layout preserved
  - Responsive design works
```

### 5.2 Testing Matrix

```markdown
| Feature | EN | UR | HI | Status |
|---------|----|----|----|-  ------|
| Homepage | ✓ | ? | ? | Pending |
| Navbar | ✓ | ? | ? | Pending |
| Language Dropdown | N/A | ? | ? | Pending |
| TranslateButton | N/A | ? | ? | Pending |
| Chapter 00-01 | ✓ | ? | ? | Pending |
| Chapter 00-02 | ✓ | ? | ? | Pending |
| ... | | | | |
| RTL Support | N/A | ? | N/A | Pending |
| Font Loading | ✓ | ? | ? | Pending |
| Authentication | ✓ | ? | ? | Pending |
| Chatbot | ✓ | ? | ? | Pending |
```

---

## 6. Deployment Specifications

### 6.1 Build Configuration

```bash
# Build all locales
npm run build

# Build specific locale
npm run build -- --locale ur
npm run build -- --locale hi

# Serve production build
npm run serve
```

### 6.2 Environment Variables

```env
# .env file additions
DOCUSAURUS_LOCALE=en
DOCUSAURUS_FALLBACK_LOCALE=en
ENABLE_TRANSLATIONS=true
```

---

## 7. Maintenance & Scalability

### 7.1 Adding New Languages

```bash
# 1. Add locale to docusaurus.config.mjs
# 2. Create directory structure
mkdir -p i18n/{locale}/docusaurus-plugin-content-docs/current

# 3. Initialize translations
npm run write-translations -- --locale {locale}

# 4. Translate content
# 5. Test and deploy
```

### 7.2 Content Update Workflow

```markdown
When updating English content:
1. Update docs/{path}/file.md
2. Flag for translation in i18n/{locale}
3. Translate updated content
4. Test all affected locales
5. Deploy changes
```

---

## 8. Performance Optimization

### 8.1 Build Optimization

```javascript
// webpack config in docusaurus.config.mjs
webpack: {
  jsLoader: (isServer) => ({
    loader: require.resolve('swc-loader'),
    options: {
      jsc: {
        parser: {
          syntax: 'typescript',
          tsx: true,
        },
        target: 'es2017',
        minify: {
          compress: true,
          mangle: true,
        },
      },
    },
  }),
}
```

### 8.2 Font Loading Strategy

```css
/* Preload critical fonts */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/NotoNastaliqUrdu-Regular.woff2') format('woff2');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}

@font-face {
  font-family: 'Noto Sans Devanagari';
  src: url('/fonts/NotoSansDevanagari-Regular.woff2') format('woff2');
  font-weight: normal;
  font-style: normal;
  font-display: swap;
}
```

---

## 9. Success Metrics

### 9.1 Technical Metrics

```yaml
Build Performance:
  - Build time < 5 minutes per locale
  - Bundle size increase < 30%
  - Page load time < 3 seconds

Functionality:
  - 100% feature parity across locales
  - 0 broken links
  - 0 missing translations
  - 100% style consistency

User Experience:
  - Language switch < 1 second
  - RTL layout perfect
  - Font rendering smooth
  - Mobile responsiveness 100%
```

### 9.2 Hackathon Bonus Points

```markdown
Requirements for 50 bonus points:
✓ Logged user can translate content
✓ Urdu translation complete
✓ Button at start of each chapter
✓ Professional UI/UX

Completion Criteria:
- All chapters translated to Urdu
- All styling preserved
- All functionality working
- TranslateButton in every chapter
- Language dropdown in navbar
- RTL support implemented
- Build successful
- Zero regressions
```

---

## 10. Risk Mitigation

### 10.1 Potential Risks

```yaml
Risk 1: Translation Quality
  Impact: High
  Probability: Medium
  Mitigation: Use professional translation tools, native speaker review

Risk 2: RTL Layout Issues
  Impact: Medium
  Probability: High
  Mitigation: Extensive testing, CSS fallbacks

Risk 3: Build Failures
  Impact: High
  Probability: Low
  Mitigation: Incremental testing, version control

Risk 4: Performance Degradation
  Impact: Medium
  Probability: Medium
  Mitigation: Code splitting, lazy loading, font optimization

Risk 5: Breaking Existing Features
  Impact: Critical
  Probability: Low
  Mitigation: Protected file policy, comprehensive testing
```

---

## 11. Documentation

### 11.1 User Documentation

Create user guide covering:
- How to switch languages
- How to use translation button
- Language-specific features
- Troubleshooting common issues

### 11.2 Developer Documentation

Create developer guide covering:
- How to add new languages
- Translation workflow
- Testing procedures
- Deployment process
- Maintenance guidelines

---

## 12. Appendices

### Appendix A: File Checklist

```markdown
Configuration Files:
- [ ] docusaurus.config.mjs updated
- [ ] package.json updated
- [ ] .env configured

New Components:
- [ ] TranslateButton.tsx created
- [ ] TranslateButton.module.css created

New Styles:
- [ ] translation.css created
- [ ] RTL support added

Translation Files (per locale):
- [ ] 00-introduction/ (7 files)
- [ ] 01-ros2/ (8 files)
- [ ] 02-simulation/ (N files)
- [ ] 03-isaac/ (N files)
- [ ] 04-vla/ (N files)
- [ ] 05-capstone/ (N files)
- [ ] _category_.json files
- [ ] index.md files
- [ ] Theme translations

Fonts:
- [ ] Noto Nastaliq Urdu downloaded
- [ ] Noto Sans Devanagari downloaded
- [ ] Font files placed in static/fonts/
```

### Appendix B: Command Reference

```bash
# Development
npm run start              # Start English (default)
npm run start:ur          # Start Urdu
npm run start:hi          # Start Hindi

# Building
npm run build             # Build all locales
npm run build -- --locale ur  # Build Urdu only
npm run build -- --locale hi  # Build Hindi only

# Translation
npm run write-translations -- --locale ur
npm run write-translations -- --locale hi

# Testing
npm run serve             # Serve production build
npm run clear             # Clear cache
```

### Appendix C: Color Palette

```css
/* Orange & White Theme */
:root {
  --primary-orange: #FF6B35;
  --secondary-orange: #FF8C42;
  --white: #FFFFFF;
  --light-orange: #FFF5F0;

  /* Gradients */
  --gradient-primary: linear-gradient(135deg, #FF6B35 0%, #FF8C42 100%);
  --gradient-light: linear-gradient(135deg, #FFF5F0 0%, #FFFFFF 100%);

  /* Dark mode */
  --dark-bg: #1A1A1A;
  --dark-bg-secondary: #2A2A2A;
}
```

---

## Document Approval

**Prepared by**: System Architect   be careful do not remove and delete any other specification file or folder  just create a new
  one folder named shoudl be like translation-specification  and create this specification in it
```

# Response

```
I have successfully created the multi-language translation system specification as requested. The specification has been created in the following location:

- File: specs/translation-specification/spec.md

The specification includes all the required components:
- Executive summary with business objectives
- Detailed scope definition
- User scenarios for language switching
- Comprehensive functional requirements
- Technical architecture with configuration details
- Implementation plan with phases
- Quality assurance measures
- Deployment specifications
- Risk mitigation strategies
- Appendices with checklists and references

The specification covers both Urdu and Hindi language support with RTL layout for Urdu, preserves all existing functionality, and maintains the orange/white styling theme. It includes all the technical details for Docusaurus i18n configuration, component specifications for the TranslateButton, and CSS specifications for RTL support.

The specification is ready for the next phase of the project.
```