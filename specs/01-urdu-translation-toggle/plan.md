# Implementation Plan: Urdu Translation Toggle

**Branch**: `01-urdu-translation-toggle` | **Date**: 2025-12-19 | **Spec**: [specs/01-urdu-translation-toggle/spec.md](../specs/01-urdu-translation-toggle/spec.md)
**Input**: Feature specification from `/specs/01-urdu-translation-toggle/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable authenticated users to translate entire chapter content from English to Urdu by pressing a single toggle button at the start of each chapter page. The system preserves markdown structure, applies RTL layout automatically, caches translations, and integrates with existing authentication and AI infrastructure. The implementation includes a frontend toggle component and backend translation API with caching.

## Technical Context

**Language/Version**: TypeScript (frontend), Python 3.11 (backend)
**Primary Dependencies**: Docusaurus (frontend), FastAPI (backend), React (frontend), LLM translation service
**Storage**: File-based caching, in-memory or Redis for translation cache
**Testing**: Jest (frontend), pytest (backend)
**Target Platform**: Web application (Docusaurus documentation site with FastAPI backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <4s first translation, <200ms cached responses, 95% markdown structure preservation
**Constraints**: <200ms cached responses, authenticated user only, preserves markdown structure
**Scale/Scope**: Individual chapter translations, per-user caching, multilingual support for future expansion

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file, the following requirements must be met:
- ✅ Only authenticated users may trigger translation
- ✅ Translation API must validate user session and chapter ownership
- ✅ No original markdown files are overwritten
- ✅ Translations are derived views, not source edits
- ✅ Cache key uses user_id + chapter_id + language
- ✅ Frontend components must be in docusaurus/src/components/translation/
- ✅ Backend components must be in physical-ai/translation/
- ✅ API endpoint must be POST /api/translate
- ✅ RTL rendering when Urdu is active
- ✅ Code blocks remain LTR even in RTL mode

## Project Structure

### Documentation (this feature)

```text
specs/01-urdu-translation-toggle/
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
└── src/
    └── components/
        └── translation/
            ├── LanguageToggle.tsx
            ├── TranslationButton.tsx
            └── rtlStyles.css

physical-ai/
└── translation/
    ├── router.py
    ├── service.py
    ├── cache.py
    ├── schemas.py
    └── prompts/
        └── en_to_ur.txt
```

**Structure Decision**: Web application with Docusaurus frontend for documentation and FastAPI backend for translation services. Frontend components follow the constitution-specified location in docusaurus/src/components/translation/. Backend translation services follow the constitution-specified location in physical-ai/translation/.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
