import React from 'react';
import './rtlStyles.css';

interface TranslationButtonProps {
  onToggle: () => void;
  currentLanguage: string;
  isTranslating: boolean;
  error?: string;
}

const TranslationButton: React.FC<TranslationButtonProps> = ({
  onToggle,
  currentLanguage,
  isTranslating,
  error
}) => {
  const buttonText = currentLanguage === 'ur' ? 'اردو → EN' : 'EN → اردو';
  const buttonClass = `translation-toggle-btn ${currentLanguage === 'ur' ? 'urdu-active' : 'english-active'} ${isTranslating ? 'translating' : ''}`;
  const isLoading = isTranslating;

  return (
    <div className="translation-button-container">
      <button
        className={buttonClass}
        onClick={onToggle}
        disabled={isLoading}
        aria-busy={isLoading}
        aria-label={
          isLoading
            ? 'Translating content, please wait'
            : currentLanguage === 'ur'
              ? 'Switch to English'
              : 'Switch to Urdu'
        }
      >
        {isLoading ? (
          <>
            <span className="loading-spinner" aria-hidden="true">🔄</span>
            <span className="loading-text">Translating...</span>
          </>
        ) : (
          buttonText
        )}
      </button>
      {isLoading && (
        <span className="loading-indicator" aria-label="Loading indicator">
          🔄
        </span>
      )}
    </div>
  );
};

export default TranslationButton;