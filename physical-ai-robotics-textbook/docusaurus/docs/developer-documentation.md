# Developer Documentation: Multi-Language Translation System

## Architecture Overview

The multi-language translation system is built as a React/Docusaurus application with a FastAPI backend. The architecture follows a client-server model with authentication integration and pre-translated content delivery.

### System Components

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Frontend      │    │   Backend        │    │  Translation   │
│   (Docusaurus)  │◄──►│   (FastAPI)      │◄──►│   (JSON Files) │
│                 │    │                  │    │                 │
│ • Language      │    │ • Auth Session   │    │ • Pre-translated│
│   Context       │    │   API            │    │   JSON files    │
│ • Language      │    │ • Translation    │    │ • Urdu & Arabic │
│   Toggle        │    │   API            │    │   content       │
│ • Translation   │    │ • User Pref.     │    │                 │
│   Loading       │    │   API            │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Technology Stack

- **Frontend**: React 18, TypeScript, Docusaurus 3.x
- **Backend**: Python, FastAPI
- **Authentication**: Session-based authentication
- **Fonts**: Noto Nastaliq Urdu (self-hosted), Noto Naskh Arabic (Google Fonts)
- **Testing**: Jest, React Testing Library, axe-core
- **Translation Format**: JSON with preserved HTML structure

## Component Documentation

### LanguageContext

The LanguageContext provides global state management for language selection across the application.

#### Interface
```typescript
interface LanguageContextType {
  language: 'en' | 'ur' | 'ar';
  setLanguage: (lang: 'en' | 'ur' | 'ar') => void;
  isAuthenticated: boolean;
  loading: boolean;
  error: string | null;
}
```

#### Functionality
- Manages current language state
- Checks authentication status on mount
- Loads saved language preference from localStorage
- Handles authentication-protected language switching
- Provides loading and error states

#### Usage
```typescript
import { useLanguage } from '@site/src/context/LanguageContext';

const MyComponent = () => {
  const { language, setLanguage, isAuthenticated } = useLanguage();

  return (
    <div>
      Current language: {language}
      {isAuthenticated && <button onClick={() => setLanguage('ur')}>Switch to Urdu</button>}
    </div>
  );
};
```

### LanguageToggle Component

The LanguageToggle provides a dropdown interface for language selection in the navbar.

#### Props
- None required (uses LanguageContext)

#### Features
- Shows current language with flag emoji
- Dropdown with all supported languages
- Authentication-aware (disabled when not logged in)
- ARIA-compliant accessibility attributes
- Keyboard navigation support
- RTL-aware positioning

#### Styling
- Uses CSS modules for scoped styles
- Responsive design for all screen sizes
- Smooth transitions and hover effects
- Proper RTL support for dropdown positioning

### TranslatedContent Component

The TranslatedContent component handles loading and displaying translated content.

#### Props
```typescript
interface TranslatedContentProps {
  children: React.ReactNode;
  chapterId: string;  // e.g., "00-introduction/index"
}
```

#### Functionality
- Dynamically loads translation JSON based on current language
- Handles loading and error states
- Preserves original HTML structure and CSS classes
- Implements caching for performance
- Falls back to English content if translation fails

#### Lifecycle
1. Checks if current language is English (no translation needed)
2. Attempts to load translation JSON file
3. Displays loading state during fetch
4. Updates content with translated HTML
5. Handles errors with fallback to English

## API Documentation

### Authentication API

#### GET /api/auth/session
Check if user is authenticated.

**Request:**
```
GET /api/auth/session HTTP/1.1
Host: localhost:8001
Cookie: session=xxx
```

**Response (200 OK):**
```json
{
  "isAuthenticated": true,
  "user": {
    "id": "user_abc123",
    "email": "user@example.com",
    "name": "John Doe",
    "createdAt": "2025-12-01T00:00:00Z"
  }
}
```

**Response (401 Unauthorized):**
```json
{
  "isAuthenticated": false
}
```

### Translation API

#### POST /api/translation/translate
Translate content using OpenAI GPT-4.

**Request:**
```
POST /api/translation/translate HTTP/1.1
Host: localhost:8001
Content-Type: application/json
Cookie: session=xxx

{
  "content": "<h1>Original content</h1><p>Text to translate</p>",
  "source_lang": "en",
  "target_lang": "ur",
  "chapter_id": "00-introduction/index"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "translated_content": "<h1>مترجم مواد</h1><p>ترجمہ شدہ متن</p>",
  "target_lang": "ur",
  "chapter_id": "00-introduction/index",
  "processing_time": 1250
}
```

#### GET /api/translation/available-languages
Get list of available translation languages.

**Response:**
```json
{
  "languages": [
    {
      "code": "en",
      "name": "English",
      "nativeName": "English",
      "direction": "ltr"
    },
    {
      "code": "ur",
      "name": "Urdu",
      "nativeName": "اردو",
      "direction": "rtl"
    },
    {
      "code": "ar",
      "name": "Arabic",
      "nativeName": "العربية",
      "direction": "rtl"
    }
  ]
}
```

### User Preferences API

#### GET /api/user/preferences
Get user's saved language preference.

**Response:**
```json
{
  "language": "ur",
  "theme": "dark",
  "notifications": true,
  "lastUpdated": "2025-12-15T10:30:00Z"
}
```

#### POST /api/user/preferences
Save user's language preference.

**Request:**
```json
{
  "language": "ur"
}
```

**Response:**
```json
{
  "success": true,
  "preferences": {
    "language": "ur",
    "theme": "dark",
    "notifications": true,
    "lastUpdated": "2025-12-15T10:30:00Z"
  }
}
```

## Translation File Structure

### JSON Schema
```typescript
interface TranslationFile {
  meta: {
    title: string;              // Translated page title
    description: string;        // Translated description
    language: 'ur' | 'ar';      // Target language
    chapter: string;            // Chapter identifier
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
      "learning-objectives": "سیکھنے کے مقاصد"
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

## RTL Implementation

### CSS Strategy
The system implements RTL support using CSS direction attributes and specific RTL styles:

```css
/* Base RTL styles */
[dir='rtl'] {
  text-align: right;
}

/* Preserve LTR for code and diagrams */
[dir='rtl'] pre,
[dir='rtl'] code,
[dir='rtl'] .mermaid,
[dir='rtl'] .codeBlockContainer {
  direction: ltr !important;
  text-align: left !important;
}
```

### Font Loading
- Urdu: Noto Nastaliq Urdu (self-hosted with @font-face)
- Arabic: Noto Naskh Arabic (Google Fonts with font-display: swap)
- Both use font-display: swap to prevent invisible text during loading

## Performance Optimization

### Caching Strategy
- Translation JSON files cached in browser
- Language context state cached with localStorage
- Auth status cached for 5 minutes
- Translation files cached with appropriate HTTP headers

### Code Splitting
- Translation components loaded on-demand
- Language-specific resources loaded only when needed
- Bundle size optimized with tree-shaking

## Contributing Guide

### Adding New Chapters

1. **Create Original Content**: Add your new chapter in English to the appropriate location in the docs directory.

2. **Create Translation Directories**: For each new chapter, create corresponding directories in all language folders:
   ```
   src/translations/en/new-chapter/
   src/translations/ur/new-chapter/
   src/translations/ar/new-chapter/
   ```

3. **Generate Translation Files**: Create JSON translation files following the schema structure. You can use the translation script or generate manually.

4. **Test Integration**: Wrap your new chapter with the TranslatedContent component if it's a main content page.

### Adding New Languages

1. **Update Language Configuration**: Add the new language to the language configuration in both frontend and backend.

2. **Add Font Support**: Include appropriate font files or CDN links for the new language.

3. **Update RTL Support**: Ensure proper RTL/LTR handling for the new language.

4. **Create Translation Files**: Generate translation files for all existing content in the new language.

### Development Workflow

1. **Setup Environment**:
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm install
   ```

2. **Start Development Server**:
   ```bash
   npm run start
   ```

3. **Backend API**: Ensure the backend server is running at `http://localhost:8001`

4. **Testing**: Run tests before committing changes:
   ```bash
   npm test
   ```

### Code Standards

- Use TypeScript for type safety
- Follow React best practices
- Maintain accessibility compliance (WCAG 2.1 AA)
- Write comprehensive tests for new features
- Document public APIs and interfaces
- Use CSS modules for component styling
- Preserve original HTML structure and CSS classes in translations

### Security Considerations

- All translation API endpoints require authentication
- Input validation on all user-provided content
- Rate limiting on API endpoints
- Proper sanitization of translation content
- Secure session management

## Environment Variables

Create a `.env` file in the docusaurus directory:
```bash
OPENAI_API_KEY=your_openai_api_key_here
AUTH_API_URL=http://localhost:8001
```

## Testing Strategy

### Unit Tests
- LanguageContext functionality
- LanguageToggle component interactions
- TranslatedContent loading logic
- Utility functions

### Integration Tests
- Auth flow integration
- Translation loading
- API communication
- State management

### End-to-End Tests
- Complete user workflows
- Language switching
- Content rendering
- Accessibility features

## Deployment Considerations

- Translation JSON files should be served statically
- Font files should be properly hosted
- Backend API should be accessible
- Authentication system should be operational
- Caching headers should be configured appropriately

---

*Last updated: December 2025*