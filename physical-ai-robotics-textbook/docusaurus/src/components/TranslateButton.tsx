import React, { useState, useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  compact?: boolean;
  showAllLanguages?: boolean;
}

export default function TranslateButton({
  compact = false,
  showAllLanguages = true
}: TranslateButtonProps): JSX.Element {
  const { i18n } = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();
  const [currentLocale, setCurrentLocale] = useState(i18n.currentLocale);

  useEffect(() => {
    setCurrentLocale(i18n.currentLocale);
  }, [i18n.currentLocale]);

  const switchLanguage = (targetLocale: string) => {
    if (targetLocale === currentLocale) return;

    // Get current path without locale prefix
    const pathWithoutLocale = location.pathname.replace(
      /^\/(en|ur|hi)\//,
      '/'
    );

    // Construct new path with target locale
    const newPath = targetLocale === 'en'
      ? pathWithoutLocale
      : `/${targetLocale}${pathWithoutLocale}`;

    // Save preference to localStorage
    localStorage.setItem('preferredLocale', targetLocale);

    // Navigate to new path
    history.push(newPath);
  };

  const languages = [
    { code: 'en', label: 'English', nativeLabel: 'English', flag: '🇬🇧' },
    { code: 'ur', label: 'Urdu', nativeLabel: 'اردو', flag: '🇵🇰' },
    { code: 'hi', label: 'Hindi', nativeLabel: 'हिंदी', flag: '🇮🇳' },
  ];

  const otherLanguages = languages.filter(lang => lang.code !== currentLocale);

  if (compact) {
    return (
      <div className={styles.translateButtonCompact}>
        {otherLanguages.map((lang) => (
          <button
            key={lang.code}
            className={styles.compactButton}
            onClick={() => switchLanguage(lang.code)}
            aria-label={`Switch to ${lang.label}`}
          >
            {lang.flag} {lang.nativeLabel}
          </button>
        ))}
      </div>
    );
  }

  return (
    <div className={styles.translateButtonContainer}>
      <div className={styles.translateButtonWrapper}>
        <div className={styles.currentLanguage}>
          <span className={styles.languageIcon}>🌐</span>
          <span className={styles.languageText}>
            {currentLocale === 'en' && 'Reading in English'}
            {currentLocale === 'ur' && 'اردو میں پڑھ رہے ہیں'}
            {currentLocale === 'hi' && 'हिंदी में पढ़ रहे हैं'}
          </span>
        </div>

        {showAllLanguages && (
          <div className={styles.languageButtons}>
            {otherLanguages.map((lang) => (
              <button
                key={lang.code}
                className={styles.languageButton}
                onClick={() => switchLanguage(lang.code)}
                aria-label={`Switch to ${lang.label}`}
              >
                <span className={styles.buttonFlag}>{lang.flag}</span>
                <span className={styles.buttonText}>
                  {lang.code === 'ur' && 'اردو میں پڑھیں'}
                  {lang.code === 'hi' && 'हिंदी में पढ़ें'}
                  {lang.code === 'en' && 'Read in English'}
                </span>
              </button>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}