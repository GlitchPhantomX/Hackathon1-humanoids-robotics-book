# Translation System Guide

## Table of Contents
1. [Overview](#overview)
2. [Language Switching](#language-switching)
3. [TranslateButton Component](#translatebutton-component)
4. [Language Preferences](#language-preferences)
5. [Troubleshooting](#troubleshooting)
6. [Developer Guide](#developer-guide)
7. [Adding New Languages](#adding-new-languages)

## Overview

The Physical AI & Humanoid Robotics Textbook now supports multiple languages with seamless switching capabilities. The system currently supports:
- English (default)
- Urdu (RTL support)
- Hindi (LTR support)

All content is professionally translated while maintaining the original styling, functionality, and user experience.

## Language Switching

### Using the Navbar Dropdown
1. Look for the language dropdown in the top-right corner of the navigation bar
2. Click the dropdown to see available languages
3. Select your preferred language from the list
4. The page will automatically reload with the new language

### Using the TranslateButton
1. Each chapter contains a TranslateButton at the beginning
2. Click the button to see language options
3. Select your preferred language
4. The content will update to the selected language

## TranslateButton Component

The TranslateButton component provides an alternative way to switch languages directly from within a chapter. It includes:
- Current language indicator
- Language selection options
- Professional styling that matches the orange/white theme
- Responsive design for all device sizes

### Features
- **Current Language Display**: Shows what language you're currently reading
- **Language Options**: Provides links to all available languages
- **Responsive Design**: Adapts to mobile, tablet, and desktop screens
- **RTL Support**: Properly displays for right-to-left languages like Urdu

## Language Preferences

### Persistence
Your language preference is automatically saved and persists across:
- Browser sessions
- Page refreshes
- Different chapters and sections
- Multiple visits to the site

### How Preferences Are Stored
1. **localStorage**: Primary storage for language preference
2. **Cookies**: Fallback for server-side detection
3. **User Profile**: For authenticated users, preferences are saved to their account

### Automatic Detection
If you visit the site for the first time, the system will:
1. Detect your browser's preferred language
2. Map it to supported languages (Urdu, Hindi, or default to English)
3. Save this preference for future visits

## Troubleshooting

### Common Issues

#### Language Not Changing
**Problem**: Clicking language options doesn't change the content
**Solution**:
- Clear your browser cache and try again
- Ensure JavaScript is enabled
- Try refreshing the page

#### Layout Issues in Urdu
**Problem**: Text alignment or layout problems in Urdu
**Solution**:
- Ensure your browser supports RTL languages
- Check that custom fonts (Noto Nastaliq Urdu) are loading properly
- Try disabling browser extensions that might interfere

#### Missing Translations
**Problem**: Some content appears in English when it should be translated
**Solution**:
- The system automatically falls back to English for missing translations
- This is intentional to ensure content is always available
- Contact us if you find significant missing translations

#### Font Loading Issues
**Problem**: Urdu or Hindi text appears as boxes or incorrect characters
**Solution**:
- Ensure you have a stable internet connection (fonts are loaded from CDN)
- Clear browser cache to reload fonts
- Check that your browser supports the required font formats

### Browser Compatibility
The translation system works with:
- Chrome (latest versions)
- Firefox (latest versions)
- Safari (latest versions)
- Edge (latest versions)
- Mobile browsers (iOS Safari, Android Chrome)

## Developer Guide

### File Structure
```
docusaurus/
├── i18n/                           # Translation files
│   ├── ur/                         # Urdu translations
│   │   ├── code.json               # UI translations
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/            # Content translations
│   └── hi/                         # Hindi translations
│       ├── code.json
│       └── docusaurus-plugin-content-docs/
│           └── current/
├── src/
│   ├── components/
│   │   └── TranslateButton.tsx     # Language switching component
│   └── utils/
│       ├── i18n.ts                 # Internationalization utilities
│       └── translationFallback.ts  # Translation fallback mechanisms
└── static/
    └── fonts/                      # Custom language fonts
        ├── NotoNastaliqUrdu/
        └── NotoSansDevanagari/
```

### Adding New Content
When adding new chapters or content:
1. Add the English content to `docs/` directory
2. Create corresponding translation files in `i18n/{locale}/docusaurus-plugin-content-docs/current/`
3. Include the TranslateButton component in new content: `import TranslateButton from '@site/src/components/TranslateButton';`
4. Test all languages to ensure proper rendering

## Adding New Languages

### Prerequisites
- Understanding of Docusaurus i18n system
- Translated content for all existing chapters
- Appropriate fonts for the language (if needed)

### Steps to Add a New Language

1. **Update Configuration**
   Add the new locale to `docusaurus.config.mjs`:
   ```javascript
   i18n: {
     defaultLocale: 'en',
     locales: ['en', 'ur', 'hi', 'new-locale'],
     localeConfigs: {
       'new-locale': {
         label: 'New Language',
         direction: 'ltr', // or 'rtl' for right-to-left
         htmlLang: 'new-locale-code',
         calendar: 'gregory',
         path: 'new-locale',
       },
     },
   },
   ```

2. **Create Directory Structure**
   ```bash
   mkdir -p i18n/new-locale/docusaurus-plugin-content-docs/current
   mkdir -p i18n/new-locale/docusaurus-theme-classic
   ```

3. **Generate Translation Files**
   ```bash
   npm run write-translations -- --locale new-locale
   ```

4. **Translate Content**
   - Translate all content files in the new locale directory
   - Maintain all HTML structure, class names, and component imports
   - Keep code blocks in English with optional comments in the new language

5. **Update Components**
   - Add the new language to the TranslateButton component
   - Update language detection utilities
   - Add appropriate fonts if needed

6. **Test Thoroughly**
   - Test language switching functionality
   - Verify layout and styling
   - Check all interactive components
   - Validate accessibility features

## Support

For issues with the translation system:
- Check the troubleshooting section above
- Ensure you're using a supported browser
- Contact the development team if problems persist
- Report missing translations or errors

---

*This guide is maintained as part of the Physical AI & Humanoid Robotics Textbook project.*