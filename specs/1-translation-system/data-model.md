# Data Model: Multi-Language Translation System

## Overview
This document defines the data structures and models for the multi-language translation system, including translation files, language context, and API contracts.

## Translation File Structure

### Translation JSON Schema
```typescript
interface TranslationFile {
  meta: {
    title: string;              // Translated page title
    description: string;        // Translated description
    language: 'ur' | 'ar';      // Target language
    chapter: string;            // Chapter identifier (e.g., "00-introduction/01-welcome")
    sourceFile: string;         // Original .md file path
    lastUpdated: string;        // ISO date string
    translator: string;         // "OpenAI GPT-4" or manual
    reviewedBy?: string;        // Optional reviewer
  };

  content: {
    headings: Record<string, string>;     // ID → Translated heading
    paragraphs: Record<string, string>;   // ID → Translated paragraph
    lists: Record<string, string[]>;      // ID → Translated list items
    tables?: Record<string, TableData>;   // ID → Translated table
    codeComments?: Record<string, string>; // ID → Translated comments
  };

  html: string;  // Complete translated HTML (main content)
}

interface TableData {
  headers: string[];
  rows: string[][];
}
```

### Example Translation File
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
      "prerequisites": "ضروری شرائط",
      "module-overview": "ماڈیول کا جائزہ"
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

## Language Context Model

### Language Context State
```typescript
interface LanguageContextType {
  language: 'en' | 'ur' | 'ar';
  setLanguage: (lang: 'en' | 'ur' | 'ar') => void;
  isAuthenticated: boolean;
  loading: boolean;
  error: string | null;
}
```

### Language Configuration
```typescript
interface LanguageConfig {
  code: string;           // 'en', 'ur', 'ar'
  name: string;           // 'English', 'Urdu', 'Arabic'
  nativeName: string;     // 'English', 'اردو', 'العربية'
  flag: string;           // '🇬🇧', '🇵🇰', '🇸🇦'
  direction: 'ltr' | 'rtl'; // Text direction
  fontFamily?: string;    // Custom font family for language
}
```

### Language Configuration Data
```typescript
const languages: Record<string, LanguageConfig> = {
  en: {
    code: 'en',
    name: 'English',
    nativeName: 'English',
    flag: '🇬🇧',
    direction: 'ltr',
  },
  ur: {
    code: 'ur',
    name: 'Urdu',
    nativeName: 'اردو',
    flag: '🇵🇰',
    direction: 'rtl',
    fontFamily: 'Noto Nastaliq Urdu',
  },
  ar: {
    code: 'ar',
    name: 'Arabic',
    nativeName: 'العربية',
    flag: '🇸🇦',
    direction: 'rtl',
    fontFamily: 'Noto Naskh Arabic',
  },
};
```

## API Models

### Translation Request Model
```typescript
interface TranslationRequest {
  content: string;        // Original content to translate
  source_lang: string;    // Source language code (default: "en")
  target_lang: string;    // Target language code
  chapter_id: string;     // Chapter identifier for context
}
```

### Translation Response Model
```typescript
interface TranslationResponse {
  success: boolean;
  translated_content: string;  // Translated HTML content
  target_lang: string;         // Target language code
  chapter_id: string;          // Chapter identifier
  processing_time: number;     // Time taken for translation (ms)
}
```

### Available Languages Response Model
```typescript
interface AvailableLanguagesResponse {
  languages: Array<{
    code: string;
    name: string;
    nativeName: string;
  }>;
}
```

### User Preferences Model
```typescript
interface UserPreferences {
  language: 'en' | 'ur' | 'ar';  // User's preferred language
  theme: 'light' | 'dark';       // UI theme preference
  notifications: boolean;        // Notification settings
  lastUpdated: string;           // ISO date string
}
```

## Translation Preservation Rules

### Content Preservation Model
```typescript
interface ContentPreservationRules {
  htmlTags: boolean;        // Preserve all HTML tags
  cssClasses: boolean;      // Preserve all CSS classes
  ids: boolean;             // Preserve all IDs
  codeBlocks: boolean;      // Keep code blocks in English
  mermaidDiagrams: boolean; // Keep diagrams in English
  urls: boolean;            // Preserve all URLs
  filePaths: boolean;       // Preserve all file paths
  technicalTerms: string;   // Transliterate with original in parentheses
  uiLabels: boolean;        // Translate UI labels
  textContent: boolean;     // Translate main text content
}
```

### Preservation Rules Data
```typescript
const preservationRules: ContentPreservationRules = {
  htmlTags: true,
  cssClasses: true,
  ids: true,
  codeBlocks: true,
  mermaidDiagrams: true,
  urls: true,
  filePaths: true,
  technicalTerms: 'transliterate-with-original',
  uiLabels: true,
  textContent: true,
};
```

## File Organization Model

### Translation Directory Structure
```
src/translations/
├── en/                          # English (original references)
│   ├── 00-introduction/
│   │   ├── index.json
│   │   ├── 01-welcome.json
│   │   ├── 02-prerequisites.json
│   │   ├── 03-hardware-requirements.json
│   │   ├── 04-how-to-use.json
│   │   └── 05-syllabus.json
│   ├── 01-ros2/
│   │   ├── index.json
│   │   ├── 01-architecture.json
│   │   ├── 02-nodes-topics.json
│   │   ├── 03-services-actions.json
│   │   ├── 04-python-packages.json
│   │   ├── 05-urdf-humanoids.json
│   │   └── 06-launch-files.json
│   ├── 02-simulation/
│   │   ├── index.json
│   │   ├── 01-gazebo-intro.json
│   │   ├── 02-urdf-sdf.json
│   │   ├── 03-sensors-plugins.json
│   │   ├── 04-world-building.json
│   │   ├── 05-ros2-integration.json
│   │   └── 06-advanced-simulation.json
│   ├── 03-isaac/
│   │   ├── index.json
│   │   ├── 01-isaac-sim.json
│   │   ├── 02-isaac-ros.json
│   │   ├── 03-vslam-navigation.json
│   │   ├── 04-perception.json
│   │   └── 05-sim-to-real.json
│   ├── 04-vla/
│   │   ├── index.json
│   │   ├── 01-voice-to-action.json
│   │   ├── 02-llm-planning.json
│   │   ├── 03-natural-language.json
│   │   └── 04-multimodal.json
│   └── 05-capstone/
│       ├── index.json
│       ├── 01-project-overview.json
│       ├── 02-architecture.json
│       ├── 03-voice-system.json
│       ├── 04-navigation.json
│       ├── 05-manipulation.json
│       └── 06-integration.json
├── ur/                          # Urdu translations (same structure)
│   ├── 00-introduction/
│   ├── 01-ros2/
│   ├── 02-simulation/
│   ├── 03-isaac/
│   ├── 04-vla/
│   └── 05-capstone/
└── ar/                          # Arabic translations (same structure)
    ├── 00-introduction/
    ├── 01-ros2/
    ├── 02-simulation/
    ├── 03-isaac/
    ├── 04-vla/
    └── 05-capstone/
```

## Validation Models

### Translation Validation Schema
```typescript
interface TranslationValidationResult {
  valid: boolean;
  checks: {
    htmlTagsPreserved: boolean;
    classNamesPreserved: boolean;
    idsPreserved: boolean;
    codeBlocksIntact: boolean;
    noEmptyContent: boolean;
    validJSON: boolean;
    contentTranslated: boolean;
  };
  errors: string[];
  warnings: string[];
}
```

### Font Loading Model
```typescript
interface FontConfig {
  fontFamily: string;
  src: string;
  fontWeight: number;
  fontStyle: string;
  fontDisplay: 'auto' | 'block' | 'swap' | 'fallback' | 'optional';
  size: string;  // Approximate file size
}
```

### Font Configuration Data
```typescript
const fontConfigs: Record<string, FontConfig> = {
  'Noto Nastaliq Urdu': {
    fontFamily: 'Noto Nastaliq Urdu',
    src: '/fonts/urdu/NotoNastaliqUrdu-Regular.ttf',
    fontWeight: 400,
    fontStyle: 'normal',
    fontDisplay: 'swap',
    size: '~500KB',
  },
  'Noto Naskh Arabic': {
    fontFamily: 'Noto Naskh Arabic',
    src: 'https://fonts.googleapis.com/css2?family=Noto+Naskh+Arabic:wght@400;500;600;700&display=swap',
    fontWeight: 400,
    fontStyle: 'normal',
    fontDisplay: 'swap',
    size: '~200KB',
  },
};
```