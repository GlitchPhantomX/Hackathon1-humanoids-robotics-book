# Developer Guide: Adding New Languages

## Overview

This guide explains how to add new languages to the Physical AI & Humanoid Robotics Textbook. The system uses Docusaurus' built-in i18n capabilities combined with custom translation components.

## Prerequisites

Before adding a new language, ensure you have:
- Translated content for all existing chapters (or a plan to translate them)
- Understanding of Docusaurus i18n system
- Access to appropriate fonts for the language (if needed)
- Knowledge of RTL vs LTR layout requirements (if applicable)

## Step-by-Step Process

### Step 1: Update Docusaurus Configuration

First, add the new locale to `docusaurus.config.mjs`:

```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur', 'hi', 'new-locale'], // Add your new locale here
  localeConfigs: {
    'en': {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
      calendar: 'gregory',
      path: 'en',
    },
    'ur': {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur-PK',
      calendar: 'gregory',
      path: 'ur',
    },
    'hi': {
      label: 'हिंदी',
      direction: 'ltr',
      htmlLang: 'hi-IN',
      calendar: 'gregory',
      path: 'hi',
    },
    'new-locale': {  // Add configuration for your new language
      label: 'New Language Name',
      direction: 'ltr', // Use 'rtl' for right-to-left languages
      htmlLang: 'new-locale-code', // e.g., 'fr-FR' for French
      calendar: 'gregory', // or appropriate calendar system
      path: 'new-locale', // URL path for this language
    },
  },
},
```

### Step 2: Create Directory Structure

Create the necessary directory structure for your new language:

```bash
mkdir -p i18n/new-locale/docusaurus-plugin-content-docs/current
mkdir -p i18n/new-locale/docusaurus-theme-classic
```

### Step 3: Generate Initial Translation Files

Use Docusaurus' built-in command to generate the initial translation files:

```bash
npm run write-translations -- --locale new-locale
```

This command will create:
- `i18n/new-locale/code.json` - UI string translations
- `i18n/new-locale/docusaurus-theme-classic/` - Theme component translations

### Step 4: Update Package.json Scripts (Optional)

Add locale-specific build scripts to `package.json` for convenience:

```json
{
  "scripts": {
    "start:new-locale": "docusaurus start --locale new-locale",
    "build:new-locale": "docusaurus build --locale new-locale"
  }
}
```

### Step 5: Translate Content Files

Now you need to translate all content files. The structure should mirror the English version:

```
i18n/new-locale/docusaurus-plugin-content-docs/current/
├── 00-introduction/
│   ├── 01-welcome.md
│   ├── 02-prerequisites.md
│   ├── 03-hardware-requirements.md
│   ├── 04-how-to-use.md
│   ├── 05-syllabus.md
│   ├── index.md
│   └── _category_.json
├── 01-ros2/
│   ├── 01-architecture.md
│   ├── 02-nodes-topics.md
│   └── [other files...]
└── [other chapters...]
```

### Step 6: Add TranslateButton to New Content

For each translated content file, ensure you include the TranslateButton component:

```markdown
---
sidebar_position: 1
title: "Your Translated Title"
description: "Your translated description"
---

import TranslateButton from '@site/src/components/TranslateButton';

<TranslateButton />

# Your Translated Content Here

[Rest of your translated content...]
```

### Step 7: Update the TranslateButton Component

Add your new language to the TranslateButton component in `src/components/TranslateButton.tsx`:

```typescript
const languages = [
  { code: 'en', label: 'English', nativeLabel: 'English', flag: '🇬🇧' },
  { code: 'ur', label: 'Urdu', nativeLabel: 'اردو', flag: '🇵🇰' },
  { code: 'hi', label: 'Hindi', nativeLabel: 'हिंदी', flag: '🇮🇳' },
  { code: 'new-locale', label: 'New Lang', nativeLabel: 'Native Name', flag: ' countryCode' }, // Add your language
];
```

### Step 8: Add Font Support (If Needed)

If your language requires special fonts, add them:

1. **Download Fonts**: Get appropriate fonts (e.g., from Google Fonts or other sources)
2. **Place in Static Directory**:
   ```bash
   mkdir -p static/fonts/NewLanguageFont
   # Place your font files there
   ```
3. **Add Font Face in CSS** (`src/css/translation.css`):
   ```css
   @font-face {
     font-family: 'New Language Font';
     src: url('/fonts/NewLanguageFont/font.woff2') format('woff2');
     font-weight: normal;
     font-style: normal;
     font-display: swap;
   }
   ```
4. **Apply Font Styling**:
   ```css
   html[lang='new-locale-code'],
   [lang='new-locale-code'] {
     font-family: 'New Language Font', 'fallback-font', sans-serif;
   }
   ```

### Step 9: Update CSS for RTL Support (If Applicable)

If your language is right-to-left, ensure proper CSS support:

```css
/* Add to src/css/translation.css */
[dir='rtl'] {
  /* Additional RTL-specific styles if needed */
}

/* Example for specific elements */
html[lang='new-locale-code'] .main-heading,
[lang='new-locale'] .main-heading {
  font-family: 'New Language Font', serif;
  font-size: 2.5rem;
  line-height: 1.8;
  /* Add other specific styling */
}
```

## Translation Workflow

### Content Translation Process

1. **Copy English Content**: Start with the English version of each file
2. **Preserve Structure**: Maintain all HTML tags, class names, and component imports
3. **Translate Text**: Translate only the actual content, keeping code blocks in English
4. **Update Frontmatter**: Translate title and description in the YAML frontmatter
5. **Test Rendering**: Ensure the page renders correctly with your translations

### Best Practices for Translations

- **Preserve Code Blocks**: Keep all code examples in English
- **Maintain Class Names**: Never change HTML class names or component imports
- **Preserve Links**: Keep all internal and external links intact
- **Maintain Structure**: Keep all HTML structure, headings, and formatting
- **Add Comments**: For code blocks, consider adding comments in the new language

### File Structure for Translations

```
i18n/
└── new-locale/
    ├── code.json                    # UI translations
    ├── docusaurus-theme-classic/    # Theme component translations
    │   ├── footer.json
    │   └── navbar.json
    └── docusaurus-plugin-content-docs/
        └── current/                 # Content translations
            ├── 00-introduction/
            ├── 01-ros2/
            ├── 02-simulation/
            ├── 03-isaac/
            ├── 04-vla/
            └── 05-capstone/
```

## Testing Your New Language

### Local Development Testing

1. **Start Development Server**:
   ```bash
   npm run start -- --locale new-locale
   ```

2. **Test All Pages**: Navigate through all translated pages
3. **Verify Layout**: Check for proper text direction and layout
4. **Test Components**: Ensure all interactive components work
5. **Check Links**: Verify all internal and external links work

### Build Testing

1. **Build for New Locale**:
   ```bash
   npm run build -- --locale new-locale
   ```

2. **Serve Build Locally**:
   ```bash
   npm run serve
   ```

3. **Test Functionality**: Verify all features work in the built version

## Common Issues and Solutions

### RTL Layout Issues
**Problem**: Right-to-left languages not displaying correctly
**Solution**: Ensure proper CSS RTL support and test thoroughly

### Font Loading Issues
**Problem**: Custom fonts not loading for the new language
**Solution**: Verify font paths, formats, and CORS settings

### Missing Translations
**Problem**: Some UI elements remain in English
**Solution**: Check `i18n/new-locale/code.json` for missing translations

### Build Failures
**Problem**: Build fails when including the new locale
**Solution**: Check for syntax errors in translation files

## Maintenance Considerations

### New Content Updates
When new English content is added:
1. Copy the new English files to the new locale directory
2. Translate the content
3. Add the TranslateButton component
4. Test the new content

### Regular Updates
- Monitor for missing translations
- Keep font files updated
- Test with new Docusaurus versions
- Update documentation as needed

## Performance Considerations

### Bundle Size
- New languages add to the overall bundle size
- Monitor build size to ensure it stays within limits
- Consider code splitting if needed for many languages

### Font Loading
- Self-host fonts when possible for better performance
- Use appropriate font-display strategies
- Consider font subsetting for large character sets

## Deployment Considerations

### Build Process
- Ensure your CI/CD system builds all locales
- Test deployment with the new locale
- Verify that URLs work correctly

### CDN and Caching
- Configure appropriate caching for translation assets
- Ensure font files are properly cached
- Consider regional CDN distribution for better performance

## Example: Adding French

Here's a complete example of adding French (fr) to the system:

### 1. Configuration
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur', 'hi', 'fr'],
  localeConfigs: {
    'fr': {
      label: 'Français',
      direction: 'ltr',
      htmlLang: 'fr-FR',
      calendar: 'gregory',
      path: 'fr',
    },
  },
},
```

### 2. Directory Structure
```bash
mkdir -p i18n/fr/docusaurus-plugin-content-docs/current
mkdir -p i18n/fr/docusaurus-theme-classic
```

### 3. Update TranslateButton
```typescript
{ code: 'fr', label: 'French', nativeLabel: 'Français', flag: '🇫🇷' },
```

## Conclusion

Adding new languages to the Physical AI & Humanoid Robotics Textbook is straightforward with Docusaurus' i18n system. The key is maintaining consistency in structure while providing accurate translations. Always test thoroughly and consider the user experience in the new language.

Remember to update the documentation when adding new languages, and ensure all team members understand the translation workflow for maintaining quality across all supported languages.