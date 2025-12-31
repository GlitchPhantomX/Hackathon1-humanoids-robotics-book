/**
 * i18n utility functions for language preference persistence and detection
 */

/**
 * Get current locale from URL, localStorage, or browser settings
 */
export function getCurrentLocale(): string {
  // First, try to get from URL
  const pathParts = window.location.pathname.split('/');
  const localeFromPath = pathParts[1] || null;

  if (localeFromPath && ['en', 'ur', 'hi'].includes(localeFromPath)) {
    return localeFromPath;
  }

  // Then try localStorage
  const storedLocale = localStorage.getItem('preferredLocale');
  if (storedLocale && ['en', 'ur', 'hi'].includes(storedLocale)) {
    return storedLocale;
  }

  // Then try cookie
  const cookieLocale = getLocaleFromCookie();
  if (cookieLocale) {
    return cookieLocale;
  }

  // Then try browser settings
  const browserLocale = getBrowserLocale();
  if (browserLocale) {
    return browserLocale;
  }

  // Default to English
  return 'en';
}

/**
 * Switch to a different locale
 */
export function switchLocale(targetLocale: string): void {
  if (!['en', 'ur', 'hi'].includes(targetLocale)) {
    console.warn(`Invalid locale: ${targetLocale}`);
    return;
  }

  // Save preference to localStorage
  localStorage.setItem('preferredLocale', targetLocale);

  // Also save to cookie for server-side detection
  setLocaleCookie(targetLocale);

  // Get current path without locale prefix
  const pathParts = window.location.pathname.split('/');
  const localeFromPath = pathParts[1];
  let pathWithoutLocale = window.location.pathname;

  if (['en', 'ur', 'hi'].includes(localeFromPath)) {
    pathWithoutLocale = window.location.pathname.substring(localeFromPath.length + 1);
  }

  // Construct new path with target locale
  const newPath = targetLocale === 'en'
    ? pathWithoutLocale
    : `/${targetLocale}${pathWithoutLocale}`;

  // Navigate to new path
  window.location.href = newPath;
}

/**
 * Get user's preferred locale from localStorage
 */
export function getPreferredLocale(): string | null {
  return localStorage.getItem('preferredLocale');
}

/**
 * Save user's preferred locale to localStorage
 */
export function savePreferredLocale(locale: string): void {
  localStorage.setItem('preferredLocale', locale);
}

/**
 * Get locale from cookie
 */
function getLocaleFromCookie(): string | null {
  const name = 'preferredLocale=';
  const decodedCookie = decodeURIComponent(document.cookie);
  const cookieArray = decodedCookie.split(';');

  for (let i = 0; i < cookieArray.length; i++) {
    let cookie = cookieArray[i];
    while (cookie.charAt(0) === ' ') {
      cookie = cookie.substring(1);
    }
    if (cookie.indexOf(name) === 0) {
      return cookie.substring(name.length, cookie.length);
    }
  }
  return null;
}

/**
 * Set locale in cookie
 */
function setLocaleCookie(locale: string): void {
  const expirationDate = new Date();
  expirationDate.setTime(expirationDate.getTime() + (365 * 24 * 60 * 60 * 1000)); // 1 year
  const expires = `expires=${expirationDate.toUTCString()}`;
  document.cookie = `preferredLocale=${locale};${expires};path=/;SameSite=Lax`;
}

/**
 * Get browser's preferred language
 */
function getBrowserLocale(): string | null {
  const browserLang = navigator.language || (navigator as any).userLanguage;

  if (browserLang.startsWith('ur')) {
    return 'ur';
  } else if (browserLang.startsWith('hi') || browserLang.startsWith('bn')) {
    return 'hi'; // Treat Bengali as Hindi for now since we don't have Bengali support
  } else {
    return 'en'; // Default to English for other languages
  }
}

/**
 * Get all available locales
 */
export function getAvailableLocales(): Array<{code: string, name: string, direction: string}> {
  return [
    { code: 'en', name: 'English', direction: 'ltr' },
    { code: 'ur', name: 'اردو', direction: 'rtl' },
    { code: 'hi', name: 'हिंदी', direction: 'ltr' }
  ];
}

/**
 * Get direction (ltr/rtl) for a locale
 */
export function getLocaleDirection(locale: string): string {
  const localeInfo = getAvailableLocales().find(l => l.code === locale);
  return localeInfo ? localeInfo.direction : 'ltr';
}

/**
 * Check if a locale has RTL direction
 */
export function isRTL(locale: string): boolean {
  return getLocaleDirection(locale) === 'rtl';
}