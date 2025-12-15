# Translation System Documentation

This project implements a hybrid translation system that combines Docusaurus' built-in i18n capabilities with custom JSON translation files.

## Overview

The translation system consists of:

1. **Docusaurus i18n**: Built-in internationalization for UI elements like navigation, footer, etc.
2. **Custom JSON translations**: For content translation stored in `src/translations/`
3. **Language Context**: Manages language state and URL synchronization
4. **Translation Hook**: `useTranslation` hook to load and use translation files

## Directory Structure

```
docusaurus/
├── i18n/                           # Docusaurus i18n files
│   ├── ur/                         # Urdu translations
│   │   ├── docusaurus-theme-classic/
│   │   └── docusaurus-plugin-content-docs/
│   └── ar/                         # Arabic translations
│       ├── docusaurus-theme-classic/
│       └── docusaurus-plugin-content-docs/
├── src/
│   ├── translations/               # Custom translation files
│   │   ├── en/                     # English translations
│   │   │   ├── 00-introduction/
│   │   │   ├── 01-ros2/
│   │   │   └── ...
│   │   ├── ur/                     # Urdu translations
│   │   │   ├── 00-introduction/
│   │   │   ├── 01-ros2/
│   │   │   └── ...
│   │   └── ar/                     # Arabic translations
│   │       ├── 00-introduction/
│   │       ├── 01-ros2/
│   │       └── ...
│   ├── context/
│   │   └── LanguageContext.tsx     # Language management
│   ├── hooks/
│   │   └── useTranslation.ts       # Translation hook
│   └── components/
│       └── LanguageToggle/         # Language toggle component
```

## How It Works

### 1. URL-based Language Switching

- URLs include locale: `/en/docs/intro`, `/ur/docs/intro`, `/ar/docs/intro`
- Language context automatically syncs with URL
- Language selection updates URL path

### 2. Using Translations in Components

```typescript
import { useTranslation } from '../hooks/useTranslation';

const MyComponent = () => {
  const t = useTranslation('00-introduction/welcome');

  return (
    <div>
      <h1>{t.title}</h1>
      <p>{t.description}</p>
      {t.content && (
        <div>
          <h2>{t.content.heading}</h2>
          <p>{t.content.paragraph}</p>
        </div>
      )}
    </div>
  );
};
```

### 3. Using Translations in MDX

```mdx
import { useTranslation } from '@site/src/hooks/useTranslation';

const TranslatedContent = () => {
  const t = useTranslation('00-introduction/welcome');

  return (
    <div>
      <h1>{t.title}</h1>
      <p>{t.description}</p>
    </div>
  );
};

<TranslatedContent />
```

## Configuration

### docusaurus.config.mjs

The configuration includes:

- i18n setup with English, Urdu, and Arabic
- RTL support for Urdu and Arabic
- Locale dropdown in navbar
- Custom webpack optimization for translations

### Language Context

- Manages language state
- Syncs with URL
- Handles authentication
- Provides keyboard shortcuts (Ctrl+1/2/3 for language switching)

## Scripts

- `npm run translate:structure`: Generate translation structure from English files
- `npm run write-translations`: Generate Docusaurus translation files

## Adding New Translations

1. Add new translation files to `src/translations/{lang}/{chapter}/{file}.json`
2. Use the `useTranslation` hook in your components
3. The system will automatically load the appropriate translation based on the current language

## RTL Support

Urdu and Arabic languages have RTL (right-to-left) support configured in:
- docusaurus.config.mjs
- Individual translation components
- CSS styles

## Migration Script

The migration script (`scripts/generate-translation-structure.js`) helps:
- Copy English translation structure to other languages
- Create placeholder files for missing translations
- Maintain consistent directory structure