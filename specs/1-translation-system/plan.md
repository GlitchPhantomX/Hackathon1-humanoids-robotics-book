# Implementation Plan: Multi-Language Translation System

**Branch**: `1-translation-system` | **Date**: 2025-12-15 | **Spec**: [specs/1-translation-system/spec.md](C:\new - Copy\specs\1-translation-system\spec.md)
**Input**: Feature specification from `C:\new - Copy\specs\1-translation-system\spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a professional, production-ready multi-language translation system for the Docusaurus textbook that allows logged-in users to seamlessly translate all chapter content into Urdu and Arabic while perfectly preserving all existing styling, layout, and functionality. The system includes a navbar language toggle, authentication-protected features, RTL support, and pre-translated JSON content.

## Technical Context

**Language/Version**: TypeScript (React), JavaScript (Docusaurus), Python (FastAPI backend)
**Primary Dependencies**: React, Docusaurus, FastAPI, OpenAI API, Noto fonts
**Storage**: JSON files for translations, localStorage for preferences
**Testing**: Jest, React Testing Library, axe-core for accessibility
**Target Platform**: Web application (Docusaurus documentation site)
**Project Type**: Web
**Performance Goals**: Translation load < 500ms, language switch < 300ms, 90%+ test coverage
**Constraints**: Auth-protected, preserve all existing styling, RTL support for Urdu/Arabic, no modifications to original chapter files
**Scale/Scope**: 6 chapters, ~78 translation files, 3 languages (en, ur, ar)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Preserves All Styling**: Translation system must maintain exact CSS classes, HTML structure, and layout of original content
2. **Authentication Required**: Language toggle must be disabled for non-authenticated users with appropriate messaging
3. **RTL Support**: Urdu and Arabic translations must render with proper right-to-left text direction and layout
4. **No Original File Modifications**: Translation system must work without modifying existing chapter files
5. **Professional UI/UX**: Language toggle must be seamlessly integrated into navbar with polished dropdown and smooth interactions
6. **Zero Console Errors**: Implementation must be error-free with proper error handling and fallbacks

## Project Structure

### Documentation (this feature)

```text
specs/1-translation-system/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
physical-ai-robotics-textbook/
├── docusaurus/
│   ├── src/
│   │   ├── components/
│   │   │   ├── LanguageToggle/           # Language toggle dropdown
│   │   │   │   ├── index.tsx
│   │   │   │   └── styles.module.css
│   │   │   └── TranslatedContent/        # Content wrapper with translation
│   │   │       ├── index.tsx
│   │   │       └── styles.module.css
│   │   ├── context/
│   │   │   └── LanguageContext.tsx       # Global language state management
│   │   ├── translations/                 # Translation JSON files
│   │   │   ├── en/                       # English reference translations
│   │   │   ├── ur/                       # Urdu translations
│   │   │   └── ar/                       # Arabic translations
│   │   └── css/
│   │       └── custom.css                # Font imports and RTL styles
│   ├── static/
│   │   └── fonts/                        # Urdu/Arabic font files
│   │       └── urdu/
│   │           └── NotoNastaliqUrdu-Regular.ttf
│   ├── src/theme/
│   │   ├── Root.js                       # Root wrapper for LanguageProvider
│   │   └── Navbar/                       # Swizzled navbar to add language toggle
│   │       └── index.js
│   ├── docusaurus.config.mjs             # Updated config with i18n settings
│   └── package.json                      # Updated dependencies
└── backend/
    └── routers/
        └── translation.py                # Translation API endpoints
```

**Structure Decision**: Web application with frontend (Docusaurus) and backend (FastAPI) components. Frontend handles UI, state management, and translation loading while backend provides translation API and authentication integration.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple component files | Required for maintainable code separation | Single file would be too complex to maintain |
| JSON translation files | Required for pre-translated content without modifying original files | Dynamic translation would be too slow and expensive |
| Backend API | Required for authentication checks and translation generation | Frontend-only would be insecure and lack auth integration |