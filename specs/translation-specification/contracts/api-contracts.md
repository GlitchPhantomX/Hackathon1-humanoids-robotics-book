# API Contracts: Multi-Language Translation System

## Document Information
- **Feature**: Multi-Language Translation System
- **Project**: Physical AI & Humanoid Robotics Textbook
- **Version**: 1.0.0
- **Status**: Design Complete
- **Created**: 2025-12-30
- **Last Updated**: 2025-12-30

## Overview
This document defines the API contracts for the multi-language translation system, specifying the interfaces for language switching, content retrieval, and preference management.

## 1. Docusaurus i18n Configuration API

### 1.1 Locale Configuration Schema
```yaml
endpoint: docusaurus.config.mjs i18n configuration
method: N/A (Static configuration)
description: Defines supported locales and their properties
request:
  type: static configuration object
  schema:
    i18n:
      defaultLocale: string
      locales: array[string]
      localeConfigs:
        [locale]:
          label: string
          direction: "ltr" | "rtl"
          htmlLang: string
          calendar: string
          path: string
response:
  type: static configuration object
  schema:
    i18n:
      defaultLocale: string
      locales: array[string]
      localeConfigs:
        [locale]:
          label: string
          direction: "ltr" | "rtl"
          htmlLang: string
          calendar: string
          path: string
```

### 1.2 Navigation API
```yaml
endpoint: /api/navigation/{locale}
method: GET
description: Retrieve navigation structure for specific locale
request:
  params:
    locale: string (required) - locale identifier (en, ur, hi)
  headers:
    Accept: application/json
response:
  status: 200
  body:
    type: object
    properties:
      sidebar: array[object]
      navigation: array[object]
      locale: string
      direction: string
```

## 2. Language Switching API

### 2.1 Language Preference Storage
```yaml
endpoint: localStorage
method: N/A (Client-side storage)
description: Store user language preference
request:
  key: "preferredLocale"
  value: string (locale identifier)
response:
  type: string
  value: locale identifier
```

### 2.2 Locale Redirect API
```yaml
endpoint: /api/locale/redirect
method: POST
description: Generate locale-specific redirect URL
request:
  body:
    type: object
    properties:
      currentPath: string (required)
      targetLocale: string (required)
      sourceLocale: string (optional)
  headers:
    Content-Type: application/json
response:
  status: 200
  body:
    type: object
    properties:
      redirectUrl: string
      targetLocale: string
      sourceLocale: string
```

## 3. Content Retrieval API

### 3.1 Translated Content API
```yaml
endpoint: /api/content/{locale}/{path}
method: GET
description: Retrieve content in specific locale
request:
  params:
    locale: string (required) - locale identifier (en, ur, hi)
    path: string (required) - content path
  headers:
    Accept: text/html
response:
  status: 200
  body:
    type: string (rendered HTML content)
  headers:
    Content-Language: string (locale)
    Content-Type: text/html
errors:
  - status: 404
    description: Content not found in specified locale
    body:
      type: object
      properties:
        error: "CONTENT_NOT_FOUND"
        locale: string
        path: string
        fallbackLocale: string
```

### 3.2 Fallback Content API
```yaml
endpoint: /api/content/fallback
method: GET
description: Retrieve fallback content when translation is missing
request:
  query:
    locale: string (required) - requested locale
    path: string (required) - content path
    fallbackChain: array[string] (optional) - ordered fallback locales
response:
  status: 200
  body:
    type: object
    properties:
      content: string
      locale: string
      fallbackUsed: boolean
      fallbackLocale: string
```

## 4. Component APIs

### 4.1 TranslateButton Component API
```typescript
interface TranslateButtonProps {
  compact?: boolean;
  showAllLanguages?: boolean;
}

interface TranslateButtonState {
  currentLocale: string;
  availableLocales: Array<{
    code: string;
    label: string;
    nativeLabel: string;
    flag: string;
  }>;
}

function switchLanguage(targetLocale: string): void {
  // Implementation handles URL redirection and preference storage
}
```

### 4.2 LanguageDropdown Component API
```typescript
interface LanguageDropdownProps {
  position?: 'left' | 'right';
  className?: string;
}

interface LanguageDropdownState {
  currentLocale: string;
  supportedLocales: Array<{
    code: string;
    label: string;
    nativeLabel: string;
    flag: string;
    direction: 'ltr' | 'rtl';
  }>;
}
```

## 5. Font Loading API

### 5.1 Font Preloading
```yaml
endpoint: /fonts/{font-family}/{font-file}
method: GET
description: Serve font files for custom typography
request:
  params:
    fontFamily: string (required) - font family name
    fontFile: string (required) - font file with extension
response:
  status: 200
  headers:
    Content-Type: font/woff2 | font/woff | font/ttf
    Cache-Control: public, max-age=31536000
```

### 5.2 CSS Font Face Declaration
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

## 6. RTL CSS API

### 6.1 Direction Attribute API
```css
/* Base RTL support */
[dir='rtl'] {
  direction: rtl;
  text-align: right;
}

/* Component-specific RTL overrides */
[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

[dir='rtl'] .menu__link {
  padding-right: var(--ifm-menu-link-padding-horizontal);
  padding-left: 0;
}

[dir='rtl'] .pagination-nav__link {
  flex-direction: row-reverse;
}
```

## 7. Error Handling API

### 7.1 Translation Loading Error Handler
```typescript
interface TranslationError {
  type: 'TRANSLATION_LOAD_ERROR' | 'FALLBACK_SUCCESS' | 'FALLBACK_FAILED';
  originalLocale: string;
  requestedLocale: string;
  fallbackLocale: string;
  fallbackContentAvailable: boolean;
  errorMessage: string;
}

function handleTranslationError(error: TranslationError): void {
  // Implementation handles error display and fallback logic
}
```

## 8. Validation Contracts

### 8.1 Request Validation
All API requests must:
- Include proper content-type headers where applicable
- Validate locale parameters against supported locales
- Sanitize path parameters to prevent directory traversal

### 8.2 Response Validation
All API responses must:
- Include appropriate content-language headers
- Return valid JSON for JSON endpoints
- Handle errors gracefully with appropriate status codes

## 9. Performance Contracts

### 9.1 Response Time SLAs
- Locale switching: < 1000ms
- Content loading: < 3000ms
- Font loading: < 500ms (after initial load)

### 9.2 Caching Contracts
- Static content: Cache for 1 hour
- Font files: Cache for 1 year
- Navigation data: Cache for 30 minutes

## 10. Security Contracts

### 10.1 Input Validation
- All locale parameters must be whitelisted values
- Path parameters must be sanitized against directory traversal
- Content must be validated for proper MDX structure

### 10.2 Content Security
- All user preferences stored client-side only
- No sensitive data transmitted in URL parameters
- Proper CSP headers for font loading

## 11. Compatibility Contracts

### 11.1 Browser Support
- Modern browsers supporting ES6 modules
- CSS Grid and Flexbox support
- localStorage availability

### 11.2 Device Support
- Desktop and mobile responsive layouts
- Touch and mouse interaction support
- Screen reader accessibility

## 12. Future Extension Points

### 12.1 New Language Addition
API contracts should support:
- Easy addition of new locale configurations
- Minimal code changes for new language support
- Consistent user experience across all locales

### 12.2 Translation Management
Future extensions might include:
- Translation progress tracking API
- Community contribution workflows
- Automated translation quality scoring