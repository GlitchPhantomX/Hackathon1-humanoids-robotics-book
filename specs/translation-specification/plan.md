# Implementation Plan: Multi-Language Translation System

**Branch**: `translation-system` | **Date**: 2025-12-30 | **Spec**: [link]
**Input**: Feature specification from `/specs/translation-specification/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive multi-language translation system for the Physical AI & Humanoid Robotics educational platform. The system will support English (existing), Urdu, and Hindi languages with seamless switching capabilities while maintaining exact styling, functionality, and user experience. This includes a TranslateButton component at the start of each chapter, language dropdown in navbar, RTL support for Urdu, and persistent language preference storage.

## Technical Context

**Language/Version**: TypeScript/JavaScript, Docusaurus v3.0+
**Primary Dependencies**: @docusaurus/core, @docusaurus/preset-classic, @docusaurus/module-type-aliases, @docusaurus/types
**Storage**: Browser localStorage for language preferences, file-based for translations
**Testing**: Manual testing across locales, build verification
**Target Platform**: Web application, responsive design for mobile/desktop
**Project Type**: Web/Docusaurus documentation site
**Performance Goals**: Language switch < 1 second, page load time < 3 seconds
**Constraints**: 0 breaking changes to existing functionality, 100% feature parity across locales, <30% bundle size increase
**Scale/Scope**: 6 chapters, 3 languages, RTL/LTR support

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ No existing file will be deleted
- ✅ No existing logic will be removed
- ✅ No schema fields will be removed
- ✅ No authentication flow will be altered
- ✅ No existing components will be modified
- ✅ No existing styling will be changed
- ✅ All changes must be **additive only**
- ✅ Protected files (docs/**, auth-backend/**, rag-chatbot/**) must not be modified
- ✅ All existing functionality must remain intact

## Project Structure

### Documentation (this feature)

```text
specs/translation-specification/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docusaurus/
├── docusaurus.config.mjs     # Modified: Add i18n configuration
├── package.json              # Modified: Add locale scripts
├── i18n/                     # NEW: Translation root directory
│   ├── ur/                   # NEW: Urdu locale
│   │   ├── code.json         # NEW: UI translations
│   │   ├── docusaurus-theme-classic/
│   │   │   ├── navbar.json
│   │   │   └── footer.json
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/
│   │           └── [chapter structure]
│   └── hi/                   # NEW: Hindi locale
│       ├── code.json
│       ├── docusaurus-theme-classic/
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               └── [chapter structure]
├── src/
│   └── components/
│       ├── TranslateButton.tsx    # NEW: Translation toggle component
│       └── TranslateButton.module.css    # NEW: Component styling
├── src/css/
│   └── translation.css       # NEW: Translation-specific styles
└── static/
    └── fonts/                # NEW: Custom fonts for Urdu/Hindi
        ├── NotoNastaliqUrdu/
        └── NotoSansDevanagari/
```

**Structure Decision**: Web application with Docusaurus i18n integration. The structure follows Docusaurus' standard i18n directory layout with additional components for translation UI. All new files are additive and do not modify existing functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |