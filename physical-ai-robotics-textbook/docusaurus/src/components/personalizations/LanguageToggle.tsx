import React, { useState, useEffect } from 'react';
import TranslationButton from './TranslationButton';
import { useTranslation } from './useTranslation';

interface LanguageToggleProps {
  chapterId: string;
  initialContent: string;
}

const LanguageToggle: React.FC<LanguageToggleProps> = ({ chapterId, initialContent }) => {
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);

  const {
    translatedContent,
    isTranslating,
    error,
    toggleLanguage,
    currentLanguage
  } = useTranslation(chapterId, initialContent);

  // Check authentication status on component mount
  useEffect(() => {
    const checkAuthStatus = () => {
      // In a real implementation, this would check for a valid auth token
      // For example, by making an API call to verify the user's session
      const token = localStorage.getItem('authToken') || sessionStorage.getItem('authToken');
      setIsAuthenticated(!!token); // Convert to boolean
    };

    checkAuthStatus();

    // Add event listener to handle auth state changes
    const handleStorageChange = () => {
      checkAuthStatus();
    };

    window.addEventListener('storage', handleStorageChange);
    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  // Determine which content to display
  const displayContent = currentLanguage === 'ur' ? translatedContent || initialContent : initialContent;

  return (
    <div className="language-toggle-container" role="region" aria-label="Language translation controls">
      {/* Only show the toggle button if user is authenticated */}
      {isAuthenticated ? (
        <TranslationButton
          onToggle={toggleLanguage}
          currentLanguage={currentLanguage}
          isTranslating={isTranslating}
          error={error}
        />
      ) : (
        <div className="auth-required-notice" role="alert" aria-live="polite">
          Please log in to access the translation feature.
        </div>
      )}

      {error && (
        <div className="translation-error" role="alert" aria-live="assertive">
          {error}
        </div>
      )}

      <div
        className={`translation-content ${currentLanguage === 'ur' ? 'rtl' : 'ltr'}`}
        dir={currentLanguage === 'ur' ? 'rtl' : 'ltr'}
        tabIndex={0}
        aria-label={`Content in ${currentLanguage === 'ur' ? 'Urdu (RTL)' : 'English (LTR)'}`}
      >
        <div
          dangerouslySetInnerHTML={{ __html: displayContent }}
        />
      </div>
    </div>
  );
};

export default LanguageToggle;