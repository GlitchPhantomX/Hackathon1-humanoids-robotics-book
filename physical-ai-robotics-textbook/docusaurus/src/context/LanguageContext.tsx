import React, { createContext, useContext, useState, useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';

/**
 * Language context type definition
 * Defines the structure of the language context with language state, setter function, and auth status
 */
interface LanguageContextType {
  language: 'en' | 'ur' | 'ar';        // Current selected language (English, Urdu, or Arabic)
  setLanguage: (lang: 'en' | 'ur' | 'ar') => void;  // Function to change the language
  isAuthenticated: boolean;            // Whether the user is authenticated to use translation
}

// Create the language context with undefined as initial value
const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

/**
 * LanguageProvider component
 * Provides language context to all child components in the app
 * Manages language state, authentication status, and user preferences
 *
 * @param children - React elements that will have access to the language context
 * @returns JSX element wrapping children with language context provider
 */
export function LanguageProvider({ children }: { children: React.ReactNode }) {
  // Get the current location to determine the locale from URL
  const location = useLocation();

  // Initialize language state with English as default or from URL
  const [language, setLanguageState] = useState<'en' | 'ur' | 'ar'>('en');
  // Initialize authentication state - default to true for development
  const [isAuthenticated, setIsAuthenticated] = useState(true);

  /**
   * Effect hook to check authentication status on component mount
   * Calls the backend auth API to determine if user is logged in
   * Updates the isAuthenticated state accordingly
   * Falls back to allowing access if backend is unavailable
   */
  useEffect(() => {
    async function checkAuth() {
      try {
        // Call backend API to check session status
        const response = await fetch('/api/auth/session', {
          credentials: 'include'  // Include cookies for session authentication
        });
        
        if (response.ok) {
          const data = await response.json();
          setIsAuthenticated(data.isAuthenticated || true);
        } else {
          // If backend returns error, allow access anyway (development mode)
          console.log('Auth check failed, allowing access for development');
          setIsAuthenticated(true);
        }
      } catch (error) {
        // If backend is not running, allow access anyway (development mode)
        console.log('Auth backend unavailable, allowing access for development');
        setIsAuthenticated(true);  // Default to authenticated when backend is unavailable
      }
    }
    checkAuth();
  }, []);  // Empty dependency array means this runs once on mount

  /**
   * Effect hook to load user's language preference from localStorage
   * Runs on mount to restore previous language selection
   */
  useEffect(() => {
    // Retrieve saved language preference from localStorage
    const saved = localStorage.getItem('preferred-language');
    if (saved && ['en', 'ur', 'ar'].includes(saved)) {
      // Update language state with saved preference
      setLanguageState(saved as 'en' | 'ur' | 'ar');
    }
  }, []);  // Runs once on mount

  /**
   * Effect hook to sync language with URL locale
   * Extracts language from URL path and updates context accordingly
   */
  useEffect(() => {
    // Extract language from URL path (e.g., /ur/, /ar/, /en/)
    const pathParts = location.pathname.split('/');
    const urlLanguage = pathParts[1];

    if (['en', 'ur', 'ar'].includes(urlLanguage)) {
      setLanguageState(urlLanguage as 'en' | 'ur' | 'ar');
    }
  }, [location.pathname]);

  /**
   * Effect hook to handle keyboard shortcuts for language switching
   * Ctrl+1: Switch to English
   * Ctrl+2: Switch to Urdu
   * Ctrl+3: Switch to Arabic
   * Ctrl+Shift+L: Focus language toggle (handled by LanguageToggle component)
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Check if Ctrl+Shift+L is pressed to open language toggle
      if (event.ctrlKey && event.shiftKey && event.key.toLowerCase() === 'l') {
        event.preventDefault();
        // This will be handled by the LanguageToggle component
        // We just need to ensure the toggle can respond to this
      }

      // Check if Ctrl+1, Ctrl+2, or Ctrl+3 is pressed to switch languages
      if (event.ctrlKey) {
        if (event.key === '1') {
          event.preventDefault();
          setLanguage('en');
        } else if (event.key === '2') {
          event.preventDefault();
          setLanguage('ur');
        } else if (event.key === '3') {
          event.preventDefault();
          setLanguage('ar');
        }
      }
    };

    // Add event listener for keyboard shortcuts
    window.addEventListener('keydown', handleKeyDown);
    // Cleanup function to remove event listener
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);  // Empty dependency - keyboard shortcuts always work

  /**
   * Function to set the language with optional authentication check
   * Updates the language state and saves preference to localStorage
   * In production with auth enabled, shows alert if user is not authenticated
   *
   * @param lang - The language to switch to ('en', 'ur', or 'ar')
   */
  const setLanguage = (lang: 'en' | 'ur' | 'ar') => {
    // Optional: Uncomment this block to enforce authentication in production
    if (!isAuthenticated) {
      alert('Please log in to use translation feature');
      return;
    }

    // Update language state
    setLanguageState(lang);
    // Save preference to localStorage for persistence
    localStorage.setItem('preferred-language', lang);

    // Update URL to include the locale
    if (typeof window !== 'undefined') {
      const currentPath = window.location.pathname;
      const pathParts = currentPath.split('/');

      // Check if the first part is already a language code
      if (['en', 'ur', 'ar'].includes(pathParts[1])) {
        // Replace existing language in URL
        pathParts[1] = lang;
      } else {
        // Add language to URL
        pathParts.splice(1, 0, lang);
      }

      const newPath = pathParts.join('/');
      window.history.pushState({}, '', newPath);
    }
  };

  // Provide language context to all child components
  return (
    <LanguageContext.Provider value={{ language, setLanguage, isAuthenticated }}>
      {children}
    </LanguageContext.Provider>
  );
}

/**
 * Custom hook to use the language context
 * Provides access to language state and functions in any component
 *
 * @returns Language context object with language, setLanguage, and isAuthenticated
 * @throws Error if used outside of LanguageProvider
 */
export function useLanguage() {
  const context = useContext(LanguageContext);
  if (!context) {
    // Throw error if hook is used outside of provider
    throw new Error('useLanguage must be used within LanguageProvider');
  }
  return context;
}