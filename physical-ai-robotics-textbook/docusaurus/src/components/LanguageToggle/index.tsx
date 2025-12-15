import React, { useState, useEffect, useRef, memo , useCallback } from 'react';
import { useLanguage } from '../../context/LanguageContext';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

/**
 * LanguageToggle Component
 * A dropdown component that allows users to switch between different languages (English, Urdu, Arabic)
 * Features authentication check, keyboard navigation, touch support, and accessibility attributes
 *
 * @returns JSX element for the language toggle dropdown
 */
const LanguageToggle = memo(() => {
  // Get language context values (current language, setter function, auth status)
  const { language, setLanguage, isAuthenticated } = useLanguage();
  const history = useHistory();

  // State to track if the dropdown is open/closed
  const [isOpen, setIsOpen] = useState(false);
  // Ref to the dropdown container for click-outside detection
  const dropdownRef = useRef<HTMLDivElement>(null);

  /**
   * Effect hook to close dropdown when clicking or touching outside the component
   * Adds event listeners for mousedown and touchstart events
   * Cleans up event listeners on component unmount
   */
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent | TouchEvent) => {
      // Check if the click/touch occurred outside the dropdown container
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    // Add event listeners for outside clicks/touches
    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('touchstart', handleClickOutside);
    // Cleanup function to remove event listeners
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('touchstart', handleClickOutside);
    };
  }, []);

  /**
   * Effect hook to handle keyboard navigation
   * - Escape key: Close the dropdown
   * - Ctrl+Shift+L: Toggle the language dropdown (if authenticated)
   * Adds and removes keyboard event listeners
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Close dropdown when Escape key is pressed
      if (event.key === 'Escape') {
        setIsOpen(false);
      }
      // Handle Ctrl+Shift+L to toggle the language dropdown
      if (event.ctrlKey && event.shiftKey && event.key.toLowerCase() === 'l') {
        event.preventDefault();
        if (isAuthenticated) {
          setIsOpen(!isOpen);
        }
      }
    };

    // Add keyboard event listener
    document.addEventListener('keydown', handleKeyDown);
    // Cleanup function to remove keyboard event listener
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen, isAuthenticated]);

  /**
   * Callback function to toggle the dropdown open/closed state
   * Shows alert if user is not authenticated
   * Uses useCallback for performance optimization
   */
  const handleToggle = useCallback(() => {
    if (!isAuthenticated) {
      // Show alert if user is not logged in
      alert('Please log in to use translation feature');
      return;
    }
    // Toggle the dropdown state
    setIsOpen(!isOpen);
  }, [isAuthenticated, isOpen]);

  /**
   * Callback function to handle touch events for mobile devices
   * Prevents default behavior to avoid scrolling while touching
   * Calls the toggle function to open/close the dropdown
   */
  const handleTouchStart = useCallback((e: React.TouchEvent) => {
    e.preventDefault(); // Prevent default to avoid scrolling while touching
    handleToggle();
  }, [handleToggle]);

  /**
   * Callback function to handle language selection
   * Updates the language context and closes the dropdown
   * Shows alert if user is not authenticated
   * Uses useCallback for performance optimization
   *
   * @param lang - The language code to switch to ('en', 'ur', or 'ar')
   */
  const handleSelect = useCallback((lang: 'en' | 'ur' | 'ar') => {
    if (!isAuthenticated) {
      // Show alert if user is not logged in
      alert('Please log in to use translation feature');
      return;
    }
    // Set the new language in the context
    setLanguage(lang);
    // Close the dropdown after selection
    setIsOpen(false);
  }, [isAuthenticated, setLanguage]);

  // Language options with labels and text direction settings
  const languages = [
    { code: 'en', label: 'English', dir: 'ltr' },    // Left-to-right for English
    { code: 'ur', label: 'اردو', dir: 'rtl' },       // Right-to-left for Urdu
    { code: 'ar', label: 'العربية', dir: 'rtl' }    // Right-to-left for Arabic
  ];

  // Get current language label for display
  const currentLang = languages.find(lang => lang.code === language)?.label || 'English';

  // Determine if component should be disabled based on authentication status
  const isDisabled = !isAuthenticated;

  return (
    <div className={styles.languageToggleContainer} ref={dropdownRef}>
      <button
        className={`${styles.languageToggleBtn} ${isDisabled ? styles.disabled : ''}`}
        onClick={handleToggle}
        onTouchStart={handleTouchStart}
        disabled={isDisabled}
        aria-label="Select Language"
        aria-haspopup="true"
        aria-expanded={isOpen}
        aria-disabled={isDisabled}
        title={isDisabled ? "Please log in to use translation feature" : "Select language"}
      >
        <span className={styles.currentLang}>{currentLang}</span>
        <span className={`${styles.arrow} ${isOpen ? styles.arrowUp : styles.arrowDown}`}>
          ▼
        </span>
      </button>

      {/* Render dropdown menu when open and user is authenticated */}
      {isOpen && isAuthenticated && (
        <div className={styles.dropdown}>
          <ul className={styles.dropdownList}>
            {languages.map((lang) => (
              <li key={lang.code} className={styles.dropdownItem}>
                <button
                  className={`${styles.dropdownItemBtn} ${
                    language === lang.code ? styles.active : ''
                  }`}
                  onClick={() => handleSelect(lang.code as 'en' | 'ur' | 'ar')}
                  onTouchStart={() => handleSelect(lang.code as 'en' | 'ur' | 'ar')}
                  aria-label={`Select ${lang.label}`}
                  dir={lang.dir}
                >
                  <span className={styles.langLabel}>{lang.label}</span>
                  {language === lang.code && (
                    <span className={styles.checkmark}>✓</span>
                  )}
                </button>
              </li>
            ))}
          </ul>
        </div>
      )}

      {/* Show authentication prompt if dropdown is open but user is not authenticated */}
      {!isAuthenticated && isOpen && (
        <div className={styles.authPrompt}>
          Please log in to access translation features
        </div>
      )}
    </div>
  );
});

export default LanguageToggle;