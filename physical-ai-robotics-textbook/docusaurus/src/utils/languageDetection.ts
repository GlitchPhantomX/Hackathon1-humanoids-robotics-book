/**
 * Language detection utilities for automatic language selection
 */

/**
 * Detect user's preferred language from browser settings
 */
export function detectUserLanguage(): string {
  // Check for existing preference in localStorage
  const storedPreference = localStorage.getItem('preferredLocale');
  if (storedPreference && ['en', 'ur', 'hi'].includes(storedPreference)) {
    return storedPreference;
  }

  // Check for preference from user profile (if available)
  const profilePreference = getUserLanguageFromProfile();
  if (profilePreference && ['en', 'ur', 'hi'].includes(profilePreference)) {
    return profilePreference;
  }

  // Detect from browser language
  const browserLang = getBrowserLanguage();
  if (browserLang) {
    return browserLang;
  }

  // Default to English
  return 'en';
}

/**
 * Get user's language preference from their profile (simulated API call)
 */
function getUserLanguageFromProfile(): string | null {
  // In a real implementation, this would make an API call to get the user's language preference
  // For now, we'll simulate it by checking a global variable or session
  if (typeof window !== 'undefined' && window.localStorage) {
    // Check if user is authenticated and has a language preference in profile
    const userLang = localStorage.getItem('userLanguagePreference');
    if (userLang && ['en', 'ur', 'hi'].includes(userLang)) {
      return userLang;
    }
  }
  return null;
}

/**
 * Get browser language and map it to supported languages
 */
function getBrowserLanguage(): string | null {
  if (typeof navigator !== 'undefined') {
    const browserLang = navigator.language || (navigator as any).userLanguage;

    // Map common language codes to our supported languages
    if (browserLang.startsWith('ur') || browserLang.includes('urdu')) {
      return 'ur';
    } else if (browserLang.startsWith('hi') || browserLang.includes('hindi')) {
      return 'hi';
    } else if (browserLang.startsWith('en')) {
      return 'en';
    } else {
      // For other languages, we could potentially detect region
      // For example, if user is in India, they might prefer Hindi
      const userRegion = getUserRegion();
      if (userRegion === 'IN') {
        return 'hi'; // Default to Hindi for India
      } else if (userRegion === 'PK' || userRegion === 'IN') {
        return 'ur'; // Default to Urdu for Pakistan and some parts of India
      }
    }
  }
  return null;
}

/**
 * Get user's region based on IP or other methods
 */
function getUserRegion(): string | null {
  // In a real implementation, this would use a geolocation service
  // For now, we'll return null to indicate we can't determine the region
  return null;
}

/**
 * Initialize language preference detection on first visit
 */
export async function initializeLanguagePreference(): Promise<string> {
  // Check if this is the user's first visit (or if we don't have a stored preference)
  const hasVisitedBefore = localStorage.getItem('hasVisitedBefore');
  const currentPreference = localStorage.getItem('preferredLocale');

  if (!hasVisitedBefore && !currentPreference) {
    // This is the first visit, so we'll auto-detect the language
    const detectedLanguage = detectUserLanguage();

    // Save that the user has visited before
    localStorage.setItem('hasVisitedBefore', 'true');

    // Save the detected language as the user's preference
    localStorage.setItem('preferredLocale', detectedLanguage);

    // In a real implementation, we might also want to save this to the user's profile
    // await saveLanguagePreferenceToProfile(detectedLanguage);

    return detectedLanguage;
  } else if (currentPreference) {
    // User has a preference, return it
    return currentPreference;
  } else {
    // User has visited before but doesn't have a preference, detect it
    const detectedLanguage = detectUserLanguage();
    localStorage.setItem('preferredLocale', detectedLanguage);
    return detectedLanguage;
  }
}

/**
 * Save language preference to user profile (simulated API call)
 */
export async function saveLanguagePreferenceToProfile(language: string): Promise<boolean> {
  // In a real implementation, this would make an API call to save the language preference
  try {
    // Example API call:
    // const response = await fetch('/api/user/preferences', {
    //   method: 'POST',
    //   headers: {
    //     'Content-Type': 'application/json',
    //   },
    //   body: JSON.stringify({ language }),
    // });
    //
    // return response.ok;

    // For now, we'll just update localStorage and return true
    localStorage.setItem('userLanguagePreference', language);
    return true;
  } catch (error) {
    console.error('Failed to save language preference to profile:', error);
    return false;
  }
}

/**
 * Track language usage for analytics
 */
export function trackLanguageUsage(language: string, page?: string): void {
  // In a real implementation, this would send analytics data to a tracking service
  // For example, Google Analytics, Mixpanel, etc.
  if (typeof window !== 'undefined' && (window as any).gtag) {
    // Example Google Analytics event:
    // (window as any).gtag('event', 'language_change', {
    //   language: language,
    //   page: page || window.location.pathname
    // });
  }

  // Log for debugging
  console.log(`Language usage tracked: ${language}`, { page: page || window.location.pathname });
}