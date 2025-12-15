import React from 'react';
import { useTranslation } from "../../hooks/useTranslation"

/**
 * Example component showing how to use translations in React components
 * This component loads translations from src/translations/{locale}/00-introduction/welcome.json
 */
const WelcomeComponent = () => {
  // Load translation data for the welcome section
  const t = useTranslation('00-introduction/welcome');

  return (
    <div>
      <h1>{t.title || 'Welcome'}</h1>
      <p>{t.description || 'Welcome to our application'}</p>
      {t.content && (
        <div>
          <h2>{t.content.heading || 'Getting Started'}</h2>
          <p>{t.content.paragraph || 'This is a sample paragraph'}</p>
        </div>
      )}
    </div>
  );
};

export default WelcomeComponent;