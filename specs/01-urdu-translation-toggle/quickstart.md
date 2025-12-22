# Quickstart: Urdu Translation Toggle

## Prerequisites
- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Access to LLM translation service

## Setup

### 1. Frontend (Docusaurus)
```bash
cd docusaurus
npm install
```

### 2. Backend (FastAPI)
```bash
cd physical-ai
pip install fastapi uvicorn
```

## Development

### 1. Start Backend Server
```bash
cd physical-ai
uvicorn translation.router:app --reload --port 8000
```

### 2. Start Frontend
```bash
cd docusaurus
npm start
```

## Key Components

### Frontend Components
- `docusaurus/src/components/translation/LanguageToggle.tsx` - Main toggle logic
- `docusaurus/src/components/translation/TranslationButton.tsx` - UI button component
- `docusaurus/src/components/translation/rtlStyles.css` - RTL styling

### Backend Components
- `physical-ai/translation/router.py` - API endpoint definition
- `physical-ai/translation/service.py` - Translation business logic
- `physical-ai/translation/cache.py` - Caching implementation
- `physical-ai/translation/schemas.py` - Data validation schemas

## Testing
```bash
# Frontend tests
cd docusaurus
npm test

# Backend tests
cd physical-ai
pytest
```