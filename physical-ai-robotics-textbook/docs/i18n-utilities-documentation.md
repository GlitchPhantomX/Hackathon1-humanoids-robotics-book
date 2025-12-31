# i18n Utilities Documentation

## Overview

This document provides comprehensive documentation for the internationalization (i18n) utility functions used in the Physical AI & Humanoid Robotics Textbook. These utilities handle language preference management, locale detection, and translation fallback mechanisms.

## File Locations

- **Primary Utilities**: `src/utils/i18n.ts`
- **Translation Fallback**: `src/utils/translationFallback.ts`
- **Language Detection**: `src/utils/languageDetection.ts`

## Core i18n Utilities

### getCurrentLocale()

**Purpose**: Gets the current locale from various sources in priority order.

**Return Type**: `string` (locale code: 'en', 'ur', 'hi')

**Logic**:
1. Checks URL path for locale
2. Falls back to localStorage preference
3. Falls back to cookie preference
4. Falls back to browser settings
5. Defaults to 'en'

**Usage**:
```typescript
const currentLocale = getCurrentLocale();
console.log(`Current locale: ${currentLocale}`);
```

### switchLocale(targetLocale: string)

**Purpose**: Switches to a different locale and updates persistence.

**Parameters**:
- `targetLocale` (string): The locale to switch to ('en', 'ur', 'hi')

**Behavior**:
- Saves preference to localStorage
- Sets cookie for server-side detection
- Updates URL to reflect new locale
- Reloads the page with new locale

**Usage**:
```typescript
switchLocale('ur'); // Switch to Urdu
switchLocale('hi'); // Switch to Hindi
```

### getPreferredLocale()

**Purpose**: Gets the user's preferred locale from localStorage.

**Return Type**: `string | null`

**Usage**:
```typescript
const preferredLocale = getPreferredLocale();
if (preferredLocale) {
  console.log(`User prefers: ${preferredLocale}`);
}
```

### savePreferredLocale(locale: string)

**Purpose**: Saves the user's preferred locale to localStorage.

**Parameters**:
- `locale` (string): The locale to save

**Usage**:
```typescript
savePreferredLocale('hi');
```

### getAvailableLocales()

**Purpose**: Returns information about all available locales.

**Return Type**: `Array<{code: string, name: string, direction: string}>`

**Usage**:
```typescript
const locales = getAvailableLocales();
locales.forEach(locale => {
  console.log(`${locale.name} (${locale.code}) - ${locale.direction}`);
});
// Output:
// English (en) - ltr
// اردو (ur) - rtl
// हिंदी (hi) - ltr
```

### getLocaleDirection(locale: string)

**Purpose**: Gets the text direction for a given locale.

**Parameters**:
- `locale` (string): The locale code

**Return Type**: `string` ('ltr' or 'rtl')

**Usage**:
```typescript
const direction = getLocaleDirection('ur');
console.log(`Urdu direction: ${direction}`); // 'rtl'
```

### isRTL(locale: string)

**Purpose**: Checks if a locale has right-to-left direction.

**Parameters**:
- `locale` (string): The locale code

**Return Type**: `boolean`

**Usage**:
```typescript
if (isRTL('ur')) {
  console.log('Urdu uses RTL layout');
}
```

## Translation Fallback Utilities

### getTranslationWithFallback()

**Purpose**: Gets a translation with fallback mechanism.

**Parameters**:
- `targetLocale` (string): The target locale
- `translationKey` (string): The key to look up
- `translations` (Record<string, any>): Available translations
- `englishTranslations` (Record<string, any>): English fallback translations

**Return Type**: `any` (the translation or fallback)

**Logic**:
1. Tries to get translation in target locale
2. Falls back to English if not found
3. Returns the key itself as ultimate fallback

### loadTranslationWithFallback()

**Purpose**: Loads translation with fallback mechanism using dynamic imports.

**Parameters**:
- `locale` (string): The locale to load
- `translationPath` (string): Path to translation file
- `fallbackPath` (string): Path for fallback (default: 'en')

**Return Type**: `Promise<any>`

**Logic**:
1. Tries to load target locale translation
2. Falls back to English if first attempt fails
3. Returns empty object if both fail

### logMissingTranslation()

**Purpose**: Logs missing translation keys for tracking and analytics.

**Parameters**:
- `locale` (string): The locale where translation is missing
- `key` (string): The missing translation key
- `context` (string, optional): Additional context

**Behavior**:
- Logs to console in development
- Can send to analytics service in production

### createTranslationGetter()

**Purpose**: Creates a translation function with fallback capabilities.

**Parameters**:
- `currentLocale` (string): The current locale
- `translations` (Record<string, any>): Available translations
- `fallbackTranslations` (Record<string, any>): Fallback translations

**Return Type**: `(key: string, defaultValue?: string) => string`

**Usage**:
```typescript
const t = createTranslationGetter('ur', urTranslations, enTranslations);
const greeting = t('hello', 'Hello'); // Gets Urdu translation or English fallback
```

## Language Detection Utilities

### detectUserLanguage()

**Purpose**: Detects the user's preferred language using multiple strategies.

**Return Type**: `string` (locale code)

**Logic**:
1. Checks localStorage for existing preference
2. Checks user profile for preference (simulated)
3. Detects from browser language settings
4. Defaults to 'en'

### initializeLanguagePreference()

**Purpose**: Initializes language preference detection on first visit.

**Return Type**: `Promise<string>` (the detected/saved language)

**Logic**:
1. Checks if user has visited before
2. If first visit, detects language and saves preference
3. If returning user, uses existing preference
4. Returns appropriate locale

### saveLanguagePreferenceToProfile(language: string)

**Purpose**: Saves language preference to user profile (simulated API call).

**Parameters**:
- `language` (string): The language to save

**Return Type**: `Promise<boolean>` (success status)

### trackLanguageUsage(language: string, page?: string)

**Purpose**: Tracks language usage for analytics.

**Parameters**:
- `language` (string): The language being used
- `page` (string, optional): The current page

**Behavior**:
- Logs usage for debugging
- Can send to analytics service in production

## Usage Examples

### Basic Language Switching
```typescript
import { getCurrentLocale, switchLocale, getAvailableLocales } from './utils/i18n';

// Get current locale
const current = getCurrentLocale();
console.log(`Currently using: ${current}`);

// Switch to Urdu
switchLocale('ur');

// Get all available locales
const locales = getAvailableLocales();
console.log('Available locales:', locales);
```

### Translation with Fallback
```typescript
import { getTranslationWithFallback, createTranslationGetter } from './utils/translationFallback';

// Direct fallback usage
const text = getTranslationWithFallback(
  'ur',
  'welcome_message',
  { ur: { welcome_message: 'خوش آمدید' } },
  { welcome_message: 'Welcome' }
);

// Using translation getter
const t = createTranslationGetter('ur', urTranslations, enTranslations);
const greeting = t('greeting');
```

### Language Detection
```typescript
import { detectUserLanguage, initializeLanguagePreference } from './utils/languageDetection';

// Detect user's language
const detectedLang = detectUserLanguage();
console.log(`Detected language: ${detectedLang}`);

// Initialize preference on first visit
initializeLanguagePreference().then(preferredLang => {
  console.log(`Using language: ${preferredLang}`);
});
```

## Error Handling

### Fallback Mechanism
The system implements a robust fallback mechanism:
1. **Primary**: Target locale translation
2. **Secondary**: English translation
3. **Tertiary**: Translation key as fallback
4. **Ultimate**: Empty object or default value

### Logging
Missing translations are logged with context to help identify areas needing translation updates.

## Performance Considerations

### Caching
- Language preferences are cached in localStorage
- Cookie fallback provides server-side detection
- Translation files are loaded dynamically as needed

### Loading
- Only required translations are loaded
- Fallback mechanisms prevent broken experiences
- Asynchronous loading for better performance

## Security Considerations

### Input Validation
- Locale codes are validated against known values
- Prevents injection of invalid locale codes
- URL manipulation is handled safely

### Storage
- Preferences are stored securely in browser storage
- Cookies use appropriate security settings
- No sensitive information stored in plain text

## Testing

### Unit Tests
Each utility function should have corresponding unit tests covering:
- Normal operation scenarios
- Fallback scenarios
- Error conditions
- Edge cases

### Integration Tests
- Language switching functionality
- Persistence mechanisms
- Fallback behavior
- Cross-browser compatibility

## Maintenance

### Adding New Languages
When adding new languages:
1. Update validation arrays in utility functions
2. Add locale to available locales list
3. Test fallback mechanisms
4. Update documentation

### Monitoring
Monitor for:
- Frequent fallback usage (indicates missing translations)
- Performance issues with translation loading
- User feedback on language preferences
- Browser compatibility issues

## Troubleshooting

### Common Issues

#### Language Not Persisting
- Check localStorage permissions
- Verify cookie settings
- Ensure user profile updates work

#### Fallbacks Not Working
- Verify fallback translation objects
- Check for typos in locale codes
- Ensure proper error handling

#### RTL Issues
- Verify CSS RTL support
- Check text direction settings
- Test layout in RTL languages

## Conclusion

These i18n utilities provide a robust foundation for multi-language support in the Physical AI & Humanoid Robotics Textbook. They handle language preference persistence, automatic detection, fallback mechanisms, and performance optimization to ensure a seamless experience across all supported languages.