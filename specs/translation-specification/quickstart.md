# Quickstart Guide: Multi-Language Translation System

## Overview
This guide provides a rapid introduction to implementing the multi-language translation system for the Physical AI & Humanoid Robotics textbook.

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project setup
- Basic understanding of React and TypeScript
- Access to project repository

## Step 1: Configure Docusaurus i18n Support

### Update docusaurus.config.mjs
```javascript
// File: docusaurus.config.mjs
export default {
  // ... existing configuration

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
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
      hi: {
        label: 'हिंदी',
        direction: 'ltr',
        htmlLang: 'hi-IN',
      },
    },
  },

  themeConfig: {
    // ... existing theme config
    navbar: {
      items: [
        // ... existing items
        {
          type: 'localeDropdown',
          position: 'right',
          className: 'language-dropdown-custom',
        },
      ],
    },
  },
};
```

## Step 2: Create Directory Structure

```bash
# Create translation directories
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
mkdir -p i18n/hi/docusaurus-plugin-content-docs/current

# Create component directories
mkdir -p src/components
mkdir -p src/css
```

## Step 3: Add Translation Components

### Create TranslateButton Component
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

## Step 4: Add CSS for RTL Support

### Create Translation CSS
```css
/* File: src/css/translation.css */

/* Urdu RTL Support */
[dir='rtl'] {
  direction: rtl;
  text-align: right;
}

[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

[dir='rtl'] .markdown > h2,
[dir='rtl'] .markdown > h3,
[dir='rtl'] .markdown > h4 {
  text-align: right;
}

/* Urdu Typography */
html[lang='ur-PK'],
[lang='ur'] {
  font-family: 'Noto Nastaliq Urdu', serif;
}

/* Hindi Typography */
html[lang='hi-IN'],
[lang='hi'] {
  font-family: 'Noto Sans Devanagari', sans-serif;
}

/* Language Dropdown Styling */
.language-dropdown-custom {
  margin-right: 20px;
}
```

## Step 5: Initialize Translation Files

```bash
# Generate initial translation files
npm run write-translations -- --locale ur
npm run write-translations -- --locale hi
```

## Step 6: Update Package Scripts

```json
{
  "scripts": {
    // ... existing scripts
    "start:ur": "docusaurus start --locale ur",
    "start:hi": "docusaurus start --locale hi",
    "build:ur": "docusaurus build --locale ur",
    "build:hi": "docusaurus build --locale hi",
    "write-translations": "docusaurus write-translations"
  }
}
```

## Step 7: Add Fonts

### Create fonts directory and add font files:
```bash
mkdir -p static/fonts
# Add Noto Nastaliq Urdu and Noto Sans Devanagari font files
```

### Add font-face declarations to custom.css:
```css
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

## Step 8: Add TranslateButton to Content

Add the TranslateButton component to each chapter:

```markdown
---
sidebar_position: 1
title: "Welcome to Physical AI & Humanoid Robotics"
---

import TranslateButton from "@site/src/components/TranslateButton"

<TranslateButton/>

# Welcome Content Here...
```

## Testing Your Setup

### Local Development
```bash
# Test English locale
npm run start

# Test Urdu locale
npm run start:ur

# Test Hindi locale
npm run start:hi
```

### Build and Serve
```bash
# Build all locales
npm run build

# Serve production build
npm run serve
```

## Key Configuration Points

1. **Locale Detection**: The system detects locale from URL path first, then user preference, then default
2. **RTL Support**: Urdu automatically gets RTL layout via the [dir='rtl'] attribute
3. **Font Loading**: Custom fonts load automatically based on language attribute
4. **Fallback**: If translation is missing, system falls back to English content

## Troubleshooting

### Common Issues:
- **Missing translations**: Check that translation files exist in correct i18n/{locale} directories
- **RTL not working**: Verify that ur locale has direction: 'rtl' in config
- **Fonts not loading**: Ensure font files are in static/fonts/ and CSS font-face declarations are correct
- **Language switching not working**: Check that localeDropdown is properly configured in navbar

### Quick Checks:
- Verify docusaurus.config.mjs has proper i18n configuration
- Confirm directory structure matches the specification
- Test that both locale switching mechanisms work (navbar and TranslateButton)
- Validate that styling is preserved across all locales