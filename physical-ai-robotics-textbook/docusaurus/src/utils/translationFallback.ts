/**
 * Translation fallback mechanism utilities
 */

/**
 * Fallback strategy: locale → default locale → English
 */
export function getTranslationWithFallback(
  targetLocale: string,
  translationKey: string,
  translations: Record<string, any>,
  englishTranslations: Record<string, any>
): any {
  // First, try to get translation in target locale
  if (translations && translations[targetLocale] && translations[targetLocale][translationKey]) {
    return translations[targetLocale][translationKey];
  }

  // If not found, try default locale (English)
  if (englishTranslations && englishTranslations[translationKey]) {
    console.warn(`Translation missing for key: ${translationKey} in locale: ${targetLocale}, falling back to English`);
    return englishTranslations[translationKey];
  }

  // If still not found, return the key itself as fallback
  console.error(`Translation completely missing for key: ${translationKey}`);
  return translationKey;
}

/**
 * Check if a translation exists for a given locale and key
 */
export function hasTranslation(locale: string, translationKey: string, translations: Record<string, any>): boolean {
  return translations &&
         translations[locale] &&
         translations[locale][translationKey] !== undefined;
}

/**
 * Load translation with fallback mechanism
 */
export async function loadTranslationWithFallback(
  locale: string,
  translationPath: string,
  fallbackPath: string = 'en'
): Promise<any> {
  try {
    // Try to load the target locale translation
    const translationModule = await import(`../i18n/${locale}/docusaurus-plugin-content-docs/current/${translationPath}`);
    return translationModule;
  } catch (error) {
    console.warn(`Failed to load translation for locale: ${locale}, path: ${translationPath}, error:`, error);

    // Fallback to English
    try {
      console.log(`Falling back to English translation for path: ${translationPath}`);
      const fallbackModule = await import(`../i18n/${fallbackPath}/docusaurus-plugin-content-docs/current/${translationPath}`);
      return fallbackModule;
    } catch (fallbackError) {
      console.error(`Fallback to English also failed for path: ${translationPath}, error:`, fallbackError);
      // Return an empty object as ultimate fallback
      return {};
    }
  }
}

/**
 * Log missing translation keys for tracking
 */
export function logMissingTranslation(locale: string, key: string, context?: string): void {
  const message = `Missing translation: locale=${locale}, key=${key}${context ? `, context=${context}` : ''}`;
  console.warn(message);

  // In a production environment, you might want to send this to an analytics service
  // For now, we'll just log it to the console
  if (typeof window !== 'undefined' && window.location.hostname !== 'localhost') {
    // In production, you could send this to an error tracking service
    // Example: send to a logging endpoint
    // fetch('/api/log-missing-translation', {
    //   method: 'POST',
    //   headers: { 'Content-Type': 'application/json' },
    //   body: JSON.stringify({ locale, key, context, url: window.location.href })
    // });
  }
}

/**
 * Create a translation getter with fallback
 */
export function createTranslationGetter(
  currentLocale: string,
  translations: Record<string, any>,
  fallbackTranslations: Record<string, any>
) {
  return (key: string, defaultValue?: string): string => {
    // Try current locale first
    if (translations && translations[key]) {
      return translations[key];
    }

    // Try fallback locale (English)
    if (fallbackTranslations && fallbackTranslations[key]) {
      logMissingTranslation(currentLocale, key, 'using fallback');
      return fallbackTranslations[key];
    }

    // Return default value or the key itself
    const result = defaultValue || key;
    logMissingTranslation(currentLocale, key, 'using default');
    return result;
  };
}