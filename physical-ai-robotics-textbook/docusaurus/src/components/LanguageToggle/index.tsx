import React from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import './styles.module.css';

export default function LanguageToggle() {
  const [language, setLanguage] = React.useState('en');

  const toggleLanguage = () => {
    const newLang = language === 'en' ? 'ur' : 'en';
    setLanguage(newLang);
    
    // Language change event dispatch karo
    document.documentElement.setAttribute('data-lang', newLang);
    localStorage.setItem('language', newLang);
    
    // Custom event trigger
    window.dispatchEvent(new CustomEvent('languageChanged', { 
      detail: { language: newLang } 
    }));
  };

  return (
    <button
      className={styles.languageToggle}
      onClick={toggleLanguage}
      aria-label="Toggle language"
      title={language === 'en' ? 'Switch to Urdu' : 'Switch to English'}
    >
      {language === 'en' ? '🇵🇰 اردو' : '🇬🇧 EN'}
    </button>
  );
}