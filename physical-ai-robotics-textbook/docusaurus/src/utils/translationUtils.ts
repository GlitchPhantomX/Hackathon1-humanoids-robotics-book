// Translation utility functions

// Type definition for locale information
export interface LocaleInfo {
  code: string;
  label: string;
  direction: 'ltr' | 'rtl';
  htmlLang: string;
}

// Supported locales
export const SUPPORTED_LOCALES: LocaleInfo[] = [
  {
    code: 'en',
    label: 'English',
    direction: 'ltr',
    htmlLang: 'en-US',
  },
  {
    code: 'ur',
    label: 'اردو',
    direction: 'rtl',
    htmlLang: 'ur-PK',
  },
  {
    code: 'hi',
    label: 'हिन्दी',
    direction: 'ltr',
    htmlLang: 'hi-IN',
  },
];

// Get locale information by code
export const getLocaleInfo = (localeCode: string): LocaleInfo | undefined => {
  return SUPPORTED_LOCALES.find(locale => locale.code === localeCode);
};

// Check if locale is supported
export const isLocaleSupported = (localeCode: string): boolean => {
  return SUPPORTED_LOCALES.some(locale => locale.code === localeCode);
};

// Get default locale
export const getDefaultLocale = (): string => {
  return 'en';
};

// Get current locale from URL or browser
export const getCurrentLocale = (): string => {
  if (typeof window !== 'undefined') {
    const pathParts = window.location.pathname.split('/');
    const potentialLocale = pathParts[1];

    // Check if the potential locale is supported
    if (potentialLocale && isLocaleSupported(potentialLocale)) {
      return potentialLocale;
    }

    // Fallback to browser language
    const browserLang = navigator.language.substring(0, 2);
    if (isLocaleSupported(browserLang)) {
      return browserLang;
    }
  }

  // Default to English
  return getDefaultLocale();
};

// Get the direction (ltr/rtl) for a locale
export const getLocaleDirection = (localeCode: string): 'ltr' | 'rtl' => {
  const localeInfo = getLocaleInfo(localeCode);
  return localeInfo ? localeInfo.direction : 'ltr';
};

// Get the HTML lang attribute for a locale
export const getHtmlLang = (localeCode: string): string => {
  const localeInfo = getLocaleInfo(localeCode);
  return localeInfo ? localeInfo.htmlLang : 'en-US';
};

// Check if locale is RTL
export const isRtlLocale = (localeCode: string): boolean => {
  return getLocaleDirection(localeCode) === 'rtl';
};

// Get fallback locale for a given locale
export const getFallbackLocale = (localeCode: string): string => {
  // For now, default to English as fallback
  return 'en';
};

// Format locale code for display
export const formatLocaleDisplay = (localeCode: string): string => {
  const localeInfo = getLocaleInfo(localeCode);
  return localeInfo ? localeInfo.label : localeCode.toUpperCase();
};

// Get all locale codes
export const getAllLocaleCodes = (): string[] => {
  return SUPPORTED_LOCALES.map(locale => locale.code);
};

// Get all locale labels
export const getAllLocaleLabels = (): string[] => {
  return SUPPORTED_LOCALES.map(locale => locale.label);
};

// Function to store user's preferred locale in localStorage
export const setPreferredLocale = (localeCode: string): void => {
  if (typeof window !== 'undefined') {
    localStorage.setItem('preferredLocale', localeCode);
  }
};

// Function to get user's preferred locale from localStorage
export const getPreferredLocale = (): string | null => {
  if (typeof window !== 'undefined') {
    return localStorage.getItem('preferredLocale');
  }
  return null;
};

// Function to update HTML attributes for locale
export const updateHtmlAttributes = (localeCode: string): void => {
  if (typeof window !== 'undefined' && typeof document !== 'undefined') {
    const html = document.documentElement;
    html.setAttribute('lang', getHtmlLang(localeCode));
    html.setAttribute('dir', getLocaleDirection(localeCode));
  }
};

// Function to load locale-specific resources
export const loadLocaleResources = async (localeCode: string): Promise<void> => {
  // In a real implementation, this would load locale-specific resources
  // such as translation files, fonts, etc.
  console.log(`Loading resources for locale: ${localeCode}`);

  // Update HTML attributes
  updateHtmlAttributes(localeCode);
};

// Function to switch locale
export const switchLocale = (newLocaleCode: string): void => {
  if (!isLocaleSupported(newLocaleCode)) {
    console.warn(`Locale ${newLocaleCode} is not supported`);
    return;
  }

  // Store the preferred locale
  setPreferredLocale(newLocaleCode);

  // Update HTML attributes
  updateHtmlAttributes(newLocaleCode);

  // Reload the page with the new locale
  if (typeof window !== 'undefined') {
    const currentPath = window.location.pathname;
    const pathParts = currentPath.split('/');

    // If we're on the default locale, we might not have a locale prefix
    if (pathParts[1] === '' || isLocaleSupported(pathParts[1])) {
      // Replace the current locale with the new one
      if (isLocaleSupported(pathParts[1])) {
        pathParts[1] = newLocaleCode;
      } else {
        // If no locale in path, add the new locale
        window.location.href = `/${newLocaleCode}${currentPath}`;
      }
    } else {
      // Add locale to the path
      window.location.href = `/${newLocaleCode}${currentPath}`;
    }
  }
};

// Function to get the base URL for translation API
export const getTranslationApiBaseUrl = (): string => {
  return process.env.TRANSLATION_API_URL || '/api/translations';
};

// Function to fetch translation for a document
export const fetchDocumentTranslation = async (
  documentPath: string,
  targetLocale: string,
  fallback: boolean = true
): Promise<any> => {
  try {
    const response = await fetch(
      `${getTranslationApiBaseUrl()}/${targetLocale}/${documentPath}?fallback=${fallback}`
    );

    if (!response.ok) {
      throw new Error(`Failed to fetch translation: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching translation:', error);
    throw error;
  }
};