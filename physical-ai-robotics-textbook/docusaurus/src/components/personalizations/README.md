# Urdu Translation Toggle Components

React components for the Urdu translation toggle feature that enables users to switch between English and Urdu content.

## Components

### LanguageToggle.tsx

The main component that orchestrates the translation toggle functionality.

**Props**:
- `chapterId` (string): Unique identifier for the current chapter
- `initialContent` (string): Original English content to translate

**Features**:
- Authentication state detection
- Conditional rendering based on user authentication
- Error handling and display
- RTL layout management

### TranslationButton.tsx

The toggle button UI component with loading states and accessibility features.

**Props**:
- `onToggle` (function): Callback function when user clicks the button
- `currentLanguage` (string): Current language ('en' or 'ur')
- `isTranslating` (boolean): Loading state indicator
- `error` (string, optional): Error message to display

**Features**:
- Loading and disabled states
- ARIA labels for accessibility
- Keyboard navigation support
- Visual feedback for current state

### useTranslation.ts

Custom React hook that manages translation state and API communication.

**Returns**:
- `translatedContent` (string): The translated content
- `isTranslating` (boolean): Whether translation is in progress
- `error` (string): Error message if translation failed
- `toggleLanguage` (function): Function to toggle between languages
- `currentLanguage` (string): Current display language ('en' or 'ur')

### rtlStyles.css

CSS file containing RTL (right-to-left) styling for Urdu content.

**Features**:
- Proper RTL text direction
- Code blocks maintaining LTR direction within RTL content
- Responsive design for toggle button
- Loading animations and states

## Usage

```tsx
import LanguageToggle from './LanguageToggle';

function MyPage() {
  return (
    <div>
      <h1>Chapter Title</h1>
      <LanguageToggle
        chapterId="my-chapter-id"
        initialContent="# Content\n\n..."
      />
    </div>
  );
}
```

## Accessibility

- ARIA labels for screen readers
- Keyboard navigation support
- Proper focus management
- Semantic HTML structure
- High contrast colors for readability

## Internationalization

- English to Urdu translation support
- Right-to-left (RTL) layout for Urdu content
- Left-to-right (LTR) code blocks maintained within RTL content
- Proper Arabic script rendering

## Performance

- Client-side caching of translation state
- Loading states to provide user feedback
- Efficient rendering to prevent unnecessary re-renders
- Lazy loading for better initial page load