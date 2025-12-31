# Data Model: Multi-Language Translation System

## Document Information
- **Feature**: Multi-Language Translation System
- **Project**: Physical AI & Humanoid Robotics Textbook
- **Version**: 1.0.0
- **Status**: Design Complete
- **Created**: 2025-12-30
- **Last Updated**: 2025-12-30

## Overview
This document describes the data structures and relationships for the multi-language translation system, focusing on how content is organized and accessed across different locales.

## Entity Definitions

### 1. Locale Configuration
- **Entity**: LocaleConfig
- **Fields**:
  - id: string (locale identifier, e.g. "en", "ur", "hi")
  - label: string (display name for locale)
  - direction: string ("ltr" or "rtl")
  - htmlLang: string (HTML language code)
  - path: string (URL path prefix)
- **Relationships**: One-to-many with ContentDocument
- **Validation**:
  - id must be unique
  - direction must be "ltr" or "rtl"
  - htmlLang must follow RFC 5646 standard

### 2. Content Document
- **Entity**: ContentDocument
- **Fields**:
  - id: string (unique identifier)
  - locale: string (foreign key to LocaleConfig)
  - filePath: string (relative path from docs root)
  - sidebar_position: number (ordering in sidebar)
  - title: string (document title)
  - description: string (meta description)
  - content: string (translated content in MDX format)
  - createdAt: datetime
  - updatedAt: datetime
- **Relationships**: Many-to-one with LocaleConfig
- **Validation**:
  - filePath must be unique within locale
  - sidebar_position must be positive number
  - content must be valid MDX

### 3. User Language Preference
- **Entity**: UserLanguagePreference
- **Fields**:
  - userId: string (reference to authenticated user)
  - preferredLocale: string (selected locale)
  - lastUpdated: datetime
  - fallbackLocales: array[string] (ordered list of fallback locales)
- **Relationships**: One-to-one with User (via userId)
- **Validation**:
  - preferredLocale must exist in LocaleConfig
  - fallbackLocales must be subset of configured locales

### 4. Translation Status
- **Entity**: TranslationStatus
- **Fields**:
  - documentId: string (reference to ContentDocument)
  - sourceLocale: string (original locale)
  - targetLocale: string (translation locale)
  - status: enum["pending", "in-progress", "review", "completed"]
  - translator: string (who translated the content)
  - reviewedBy: string (who reviewed the translation)
  - lastTranslatedAt: datetime
- **Relationships**: Many-to-one with ContentDocument
- **Validation**:
  - sourceLocale and targetLocale must be different
  - status must be one of allowed values

## State Transitions

### Translation Status Transitions
- "pending" → "in-progress" (when translator starts work)
- "in-progress" → "review" (when translator marks as ready for review)
- "review" → "completed" (when reviewer approves)
- "review" → "in-progress" (when changes requested)
- "completed" → "in-progress" (when updates needed)

### User Preference Transitions
- Default: User preference follows system default locale
- Changed: User selects different locale via UI
- Expired: Preference cleared after period of inactivity

## Relationships

### Content Hierarchy
```
LocaleConfig (1) → (Many) ContentDocument
ContentDocument (1) → (Many) TranslationStatus (via targetLocale)
UserLanguagePreference (1) → (1) LocaleConfig
```

### Content Mapping
For each English document at `docs/{section}/{file}.md`, there should be:
- Urdu translation at `i18n/ur/docusaurus-plugin-content-docs/current/{section}/{file}.md`
- Hindi translation at `i18n/hi/docusaurus-plugin-content-docs/current/{section}/{file}.md`

## Validation Rules

### Content Validation
- All translated documents must preserve original frontmatter structure
- Import statements must be preserved in translations
- Component usage (e.g., `<PersonalizeButton/>`) must be preserved
- CSS class names must be preserved in translations
- Links and navigation must be updated to reflect locale-specific paths

### Locale Validation
- Each locale must have corresponding directory structure
- Locale configurations must be consistent across all environments
- RTL locales must have appropriate CSS direction settings

## Data Flow

### Reading Flow
1. User visits page with or without locale in URL
2. System determines effective locale (URL param → user preference → default)
3. Content is loaded from appropriate locale directory
4. If content doesn't exist in preferred locale, fallback mechanism activates

### Writing Flow
1. Translator accesses translation interface
2. System presents source content and target locale
3. Translator creates translation preserving structure
4. Translation status is updated and reviewed
5. Content is published when approved

## Constraints

### Structural Constraints
- Source English content in `docs/` directory is immutable
- Translation directories follow exact same structure as source
- All CSS classes and component imports must be preserved
- Sidebar positions should be maintained across translations

### Functional Constraints
- User preferences persist across sessions
- Fallback to English occurs gracefully when translations are missing
- Navigation links adapt to current locale
- Search functionality works independently for each locale

## Future Extensions

### Additional Language Support
- New locales can be added by extending LocaleConfig
- Translation status tracking scales to new languages
- Font loading configuration is extensible

### Content Management
- Translation memory system could be integrated
- Community contribution workflow could be implemented
- Automated quality checking could be added