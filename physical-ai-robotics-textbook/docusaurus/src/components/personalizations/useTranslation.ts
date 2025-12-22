import { useState, useEffect } from 'react';

interface TranslationState {
  translatedContent: string | null;
  isTranslating: boolean;
  error: string | null;
  currentLanguage: 'en' | 'ur';
}

export const useTranslation = (chapterId: string, initialContent: string) => {
  const [state, setState] = useState<TranslationState>({
    translatedContent: null,
    isTranslating: false,
    error: null,
    currentLanguage: 'en' as 'en' | 'ur',
  });

  // Check if user is authenticated (simplified check)
  const isAuthenticated = (): boolean => {
    // In a real implementation, this would check for a valid auth token
    // For now, we'll assume the user is authenticated if they can access this component
    return true;
  };

  const translateContent = async () => {
    if (!isAuthenticated()) {
      setState(prev => ({
        ...prev,
        error: 'User not authenticated',
        isTranslating: false,
      }));
      return;
    }

    setState(prev => ({
      ...prev,
      isTranslating: true,
      error: null,
    }));

    try {
      // Call the backend translation API
      const response = await fetch('/api/translation/urdu', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('authToken') || sessionStorage.getItem('authToken') || 'dummy-token'}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          source_language: 'en',
          target_language: 'ur',
          content: initialContent,
          user_id: localStorage.getItem('userId') || 'anonymous', // In real app, this would come from auth
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Handle error response from backend
      if (data.error) {
        setState(prev => ({
          ...prev,
          error: data.error,
          isTranslating: false,
          currentLanguage: 'en', // Stay in English if there's an error
        }));
        return; // Don't update translated content if there was an error
      }

      setState(prev => ({
        ...prev,
        translatedContent: data.translated_content,
        isTranslating: false,
        currentLanguage: 'ur',
      }));
    } catch (error) {
      console.error('Translation error:', error);
      setState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'An error occurred during translation',
        isTranslating: false,
      }));
    }
  };

  const toggleLanguage = () => {
    if (state.currentLanguage === 'en') {
      // Switch to Urdu - translate the content
      translateContent();
    } else {
      // Switch back to English - show original content
      setState(prev => ({
        ...prev,
        translatedContent: null, // This will cause the effect to show initial content
        currentLanguage: 'en',
        error: null,
      }));
    }
  };

  // Effect to handle language switching and content display
  useEffect(() => {
    if (state.currentLanguage === 'en') {
      // When language is English, we always show the initial content
      setState(prev => ({
        ...prev,
        translatedContent: null,
      }));
    }
  }, [state.currentLanguage, initialContent]);

  return {
    translatedContent: state.translatedContent || initialContent,
    isTranslating: state.isTranslating,
    error: state.error,
    currentLanguage: state.currentLanguage,
    toggleLanguage,
  };
};