# Language Switching User Guide

## Overview

The Physical AI & Humanoid Robotics Textbook supports multiple languages with seamless switching. This guide explains how to use the language switching features to access content in your preferred language.

## Available Languages

Currently, the textbook is available in:
- **English** - The default language
- **اردو (Urdu)** - Right-to-left (RTL) layout support
- **हिंदी (Hindi)** - Left-to-right (LTR) layout

All content is professionally translated while maintaining the original styling and functionality.

## Methods to Switch Languages

### Method 1: Navigation Bar Dropdown

The primary way to switch languages is through the language dropdown in the navigation bar:

1. **Locate the Language Dropdown**
   - Look in the top-right corner of the page
   - You'll see a language indicator (e.g., "English", "اردو", "हिंदी")

2. **Open the Dropdown**
   - Click on the language indicator
   - A dropdown menu will appear with available options

3. **Select Your Language**
   - Click on your preferred language from the list
   - The page will automatically reload with the new language

4. **Your Preference is Saved**
   - Your language choice is remembered across sessions
   - No need to select again on future visits

### Method 2: TranslateButton Component

Each chapter also contains a TranslateButton for convenient language switching:

1. **Find the TranslateButton**
   - Look for a prominent button near the beginning of each chapter
   - It typically says "Reading in [language]" with language options

2. **View Available Languages**
   - The button displays your current language
   - Other available languages are shown as options

3. **Switch Languages**
   - Click on the language you want to switch to
   - The content will update immediately

## Language Preference Persistence

### What Gets Saved
Your language preference is automatically saved for:
- Current and future browsing sessions
- All pages and chapters within the textbook
- Multiple visits to the site

### Where Preferences Are Stored
- **Browser Storage**: Primary storage in localStorage
- **Cookies**: Backup storage for server-side detection
- **User Profile**: For logged-in users, preferences are saved to your account

## Special Features for Different Languages

### Urdu (RTL Support)
When reading in Urdu, you'll notice:
- **Right-to-left text direction** for proper Urdu reading
- **RTL layout adjustments** for navigation and content
- **Noto Nastaliq Urdu font** for beautiful text rendering
- **Preserved code blocks** in English with appropriate styling

### Hindi (LTR Support)
When reading in Hindi:
- **Left-to-right text direction** (standard for Hindi)
- **Noto Sans Devanagari font** for clear rendering
- **Consistent styling** with the orange/white theme
- **English code blocks** with Hindi comments when needed

## Troubleshooting Common Issues

### Language Not Changing
**Symptoms**: Clicking language options doesn't update the content
**Solutions**:
1. Refresh the page (Ctrl+F5 or Cmd+Shift+R)
2. Clear your browser cache
3. Ensure JavaScript is enabled in your browser
4. Try using a different browser

### Layout Issues in Urdu
**Symptoms**: Text alignment problems or layout issues when reading Urdu
**Solutions**:
1. Ensure your browser supports RTL languages
2. Check that custom fonts are loading properly
3. Disable browser extensions that might interfere with layout
4. Try a hard refresh to reload CSS

### Missing Translations
**Symptoms**: Some content appears in English when it should be in your selected language
**Explanation**: The system gracefully falls back to English when translations are missing to ensure content is always available
**Note**: This is a feature, not a bug, ensuring you always have access to content

### Font Loading Issues
**Symptoms**: Text appears as boxes or incorrect characters
**Solutions**:
1. Check your internet connection (fonts are loaded from CDN)
2. Clear browser cache to force font reload
3. Verify browser supports required font formats (WOFF2)

## Accessibility Features

### Screen Reader Support
- All languages include proper ARIA labels
- Language-specific pronunciation support
- Clear navigation structure maintained across languages

### Keyboard Navigation
- Full keyboard accessibility in all languages
- Proper focus management when switching languages
- Standard navigation shortcuts work in all locales

### High Contrast and Zoom
- Maintains readability at 200% zoom
- Color contrast maintained across all languages
- Responsive layout adapts to different accessibility settings

## Browser Compatibility

The language switching system works with:
- **Chrome**: Version 90 and above
- **Firefox**: Version 90 and above
- **Safari**: Version 14 and above
- **Edge**: Version 90 and above
- **Mobile Browsers**: iOS Safari and Android Chrome

## Tips for Best Experience

1. **Use Modern Browsers**: For best font rendering and RTL support
2. **Stable Connection**: Ensures custom fonts load properly
3. **Updated Browser**: Gets latest language and accessibility features
4. **No Aggressive Ad Blockers**: Some may interfere with font loading

## Support

If you encounter issues with language switching:
1. Try the troubleshooting steps above
2. Verify you're using a supported browser
3. Contact support with details about your browser and the issue
4. Report any missing translations or errors

---

*This guide helps you make the most of the multi-language features in the Physical AI & Humanoid Robotics Textbook.*