# Physical AI & Robotics Textbook RAG Chatbot & Multi-Language Translation System

An intelligent chatbot that allows users to ask questions about the Physical AI & Robotics textbook using Retrieval-Augmented Generation (RAG), with multi-language translation support for Urdu, Hindi, and Arabic.

## Overview

This project implements a RAG-based chatbot that enables users to have conversations with the Physical AI & Robotics textbook. The system:

- Ingests textbook content into a vector database
- Performs semantic search to find relevant passages
- Uses AI to generate contextual, accurate responses
- Maintains conversation history
- Provides source citations for all answers
- Supports multi-language translation (English, Urdu, Hindi, Arabic) with RTL layout support
- Requires authentication for translation features

## Features

- **Intelligent Search**: Finds relevant content using vector similarity search
- **Context-Aware Responses**: Understands conversation context and follows up appropriately
- **Source Citations**: All answers include citations to specific textbook sections
- **Text Selection**: Users can select text on any page and ask questions about it
- **Persistent Conversations**: Conversation history maintained across sessions
- **Responsive Design**: Works on desktop and mobile devices
- **Multi-Language Support**: Translate content to Urdu, Hindi, and Arabic with RTL layout
- **Authentication Required**: Translation features require user authentication
- **Font Support**: Proper font rendering for Urdu (Noto Nastaliq), Hindi (Noto Sans Devanagari), and Arabic (Noto Naskh)
- **Content Preservation**: Maintains all original styling, layout, and functionality during translation
- **Language Preference Persistence**: User preferences are saved across sessions using localStorage and cookies
- **Automatic Language Detection**: System detects user's preferred language from browser settings

## Architecture

The system consists of two main components:

### Backend (FastAPI)
- Handles document ingestion and vectorization
- Manages conversation state in PostgreSQL
- Performs semantic search in Qdrant
- Generates responses using Google Generative AI
- Provides REST API endpoints
- Manages authentication and user sessions
- Handles translation API requests
- Manages user language preferences in database

### Frontend (Docusaurus + React)
- Integrated chat widget on all documentation pages
- Text selection popup for asking questions about selected text
- Conversation history persistence via sessionStorage
- Responsive design for desktop and mobile
- Multi-language translation system with RTL support
- Language toggle integrated in navbar
- Content preservation during translation
- TranslateButton component for chapter-specific language switching
- Automatic language detection and preference persistence

## Translation Feature

The multi-language translation system provides seamless language switching for the textbook content:

### Supported Languages
- **English** (default, LTR)
- **Urdu** (RTL, with Noto Nastaliq font)
- **Hindi** (LTR, with Noto Sans Devanagari font)
- **Arabic** (RTL, with Noto Naskh font)

### Key Features
- **Authentication Required**: Language toggle is disabled for non-authenticated users
- **RTL Support**: Proper right-to-left text rendering for Urdu and Arabic
- **Font Management**: Self-hosted fonts for Urdu and Hindi, Google Fonts for Arabic
- **Content Preservation**: All HTML structure, CSS classes, and formatting maintained
- **Code Block Preservation**: Code remains in English regardless of language setting
- **Caching**: Translation content cached for performance
- **Smooth Transitions**: Language switching with loading states and error handling
- **Language Preference Persistence**: Saves user preferences to localStorage and cookies
- **Automatic Detection**: Detects user's preferred language from browser settings
- **Fallback Mechanism**: Gracefully falls back to English when translations are missing
- **User Profile Integration**: Saves language preferences to user account when logged in

### How to Use
1. Log in to access translation features
2. Click the language toggle in the top-right navbar
3. Select your preferred language from the dropdown
4. Content will automatically translate while preserving all styling
5. Alternatively, use the TranslateButton component at the beginning of each chapter

## Build Commands for Locales

- `npm run start` - Start English (default)
- `npm run start:ur` - Start Urdu locale
- `npm run start:hi` - Start Hindi locale
- `npm run build` - Build all locales
- `npm run build -- --locale ur` - Build Urdu only
- `npm run build -- --locale hi` - Build Hindi only

## Contribution Guidelines

We welcome contributions to expand language support and improve translations:

1. **Adding New Languages**: Follow the developer guide in `TRANSLATION_GUIDE.md`
2. **Improving Translations**: Submit pull requests with corrections
3. **Bug Reports**: Create issues for any translation-related problems
4. **Feature Requests**: Suggest improvements to the translation system

## Setup & Installation

### Backend
1. Navigate to the `backend` directory
2. Follow the setup instructions in [backend/README.md](backend/README.md)
3. Ensure authentication endpoints are available at `/api/auth/session`

### Frontend (Docusaurus)
1. Navigate to the `physical-ai-robotics-textbook/docusaurus` directory
2. Install dependencies: `npm install`
3. Start development server: `npm run start`
4. The translation system is already integrated with the Docusaurus documentation site
5. Translation JSON files are located in `i18n/` directory
6. Font files are located in `static/fonts/` directory

### Dependencies
- Node.js 18+
- Docusaurus 3.x
- React 18+
- TypeScript
- FastAPI (backend)
- OpenAI API key (for translation generation)
- Noto Nastaliq Urdu font (self-hosted)
- Noto Sans Devanagari Hindi font (self-hosted)
- Noto Naskh Arabic font (Google Fonts)

## Usage

1. **Document Ingestion**: Run the ingestion script to load textbook content into the vector database
2. **Start the Backend**: Run the FastAPI server
3. **Browse Documentation**: Visit any documentation page
4. **Ask Questions**: Use the floating chat button or select text to ask questions about the content

## Technologies Used

- **Backend**: Python, FastAPI, PostgreSQL, Qdrant, Google Generative AI
- **Frontend**: React, Docusaurus, JavaScript/TypeScript
- **Vector Storage**: Qdrant Cloud
- **Database**: Neon Postgres
- **AI Model**: Google Gemini Pro

## Development

See individual README files for development setup:
- [backend/README.md](backend/README.md) for backend development setup
- [TRANSLATION_GUIDE.md](TRANSLATION_GUIDE.md) for translation system development

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

[Specify license here]