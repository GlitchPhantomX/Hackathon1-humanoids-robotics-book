import React, { useState, useEffect, useCallback } from 'react';
import { useLanguage } from "../../context/LanguageContext"
import styles from './styles.module.css';

/**
 * Translation data structure
 * Defines the format of translation JSON files
 */
interface TranslationData {
  meta: {
    title: string;        // Translated title of the page
    language: string;     // Language code ('ur' or 'ar')
    chapter: string;      // Chapter identifier
  };
  html: string;           // Complete translated HTML content
}

/**
 * Props for the TranslatedContent component
 */
interface TranslatedContentProps {
  chapterId: string;      // Identifier for the current chapter (e.g., '00-introduction')
  children: React.ReactNode; // Original English content to display as fallback
}

// Translation cache to improve performance by storing loaded translations in memory
const translationCache = new Map<string, TranslationData>();

/**
 * Function to preload the next chapter's translation
 * Improves user experience by loading likely next content in advance
 *
 * @param currentChapterId - The ID of the current chapter
 * @param language - The target language for preloading
 */
const preloadNextChapter = async (currentChapterId: string, language: string) => {
  if (language === 'en') return; // No need to preload for English (original content)

  try {
    // Define chapter sequence for preloading next chapter in order
    const chapterSequence = [
      '00-introduction',
      '01-ros2',
      '02-simulation',
      '03-isaac',
      '04-vla',
      '05-capstone'
    ];

    const currentIndex = chapterSequence.indexOf(currentChapterId);
    if (currentIndex !== -1 && currentIndex < chapterSequence.length - 1) {
      const nextChapterId = chapterSequence[currentIndex + 1];
      const key = `${language}-${nextChapterId}`;

      // Only preload if not already in cache to avoid unnecessary work
      if (!translationCache.has(key)) {
        const translation = await import(
          `@site/src/translations/${language}/${nextChapterId}.json`
        );
        translationCache.set(key, translation.default);
      }
    }
  } catch (error) {
    console.warn(`Preload failed for next chapter after ${currentChapterId}:`, error);
  }
};

/**
 * TranslatedContent Component
 * Loads and displays translated content based on the current language setting
 * Features caching, loading states, error handling, and RTL support
 *
 * @param props - Component props including chapterId and original content
 * @returns JSX element with translated or original content based on language setting
 */
const TranslatedContent: React.FC<TranslatedContentProps> = ({ chapterId, children }) => {
  // Get current language from context
  const { language } = useLanguage();
  // State for storing translated data
  const [translatedData, setTranslatedData] = useState<TranslationData | null>(null);
  // Loading state for UI feedback
  const [loading, setLoading] = useState(false);
  // Error state for error handling
  const [error, setError] = useState<Error | null>(null);
  // Success state for successful translation load feedback
  const [success, setSuccess] = useState(false);
  // Progress state for loading indicator
  const [progress, setProgress] = useState(0);

  /**
   * Function to load translation with caching
   * Uses dynamic imports to load translation JSON files
   * Implements caching to avoid repeated network requests
   *
   * @param lang - The target language code ('en', 'ur', or 'ar')
   * @param chapId - The chapter ID to load translation for
   * @returns Translation data or null if not found
   */
  const loadTranslation = useCallback(async (lang: string, chapId: string) => {
    const key = `${lang}-${chapId}`;

    // Check cache first to avoid unnecessary network requests
    if (translationCache.has(key)) {
      return translationCache.get(key);
    }

    try {
      // Dynamically import the translation JSON file
      const translation = await import(
        `@site/src/translations/${lang}/${chapId}.json`
      );

      // Cache the result to improve subsequent loads
      translationCache.set(key, translation.default);
      return translation.default;
    } catch (err) {
      console.error(`Translation load failed for ${lang}/${chapId}:`, err);
      return null;
    }
  }, []);

  /**
   * Effect hook to load translation when language or chapterId changes
   * Handles loading states, progress indication, error handling, and retry logic
   * Implements preloading for better performance
   */
  useEffect(() => {
    // Reset progress when starting to load
    setProgress(0);

    const loadTranslationData = async () => {
      // If language is English, use original content (no translation needed)
      if (language === 'en') {
        setTranslatedData(null);
        setError(null);
        setProgress(100);
        setTimeout(() => setProgress(0), 300); // Reset after brief display
        return;
      }

      setLoading(true);
      setError(null);

      // Simulate progress for better UX during loading
      const progressInterval = setInterval(() => {
        setProgress(prev => {
          if (prev >= 90) {
            clearInterval(progressInterval);
            return 90; // Don't reach 100 until actual load is complete
          }
          return prev + 10;
        });
      }, 200);

      try {
        const data = await loadTranslation(language, chapterId);
        clearInterval(progressInterval);

        if (data) {
          setProgress(100); // Complete progress
          setTranslatedData(data);
          setSuccess(true);
          // Clear success message after 2 seconds for clean UI
          setTimeout(() => setSuccess(false), 2000);

          // Preload next chapter after successful load for better UX
          setTimeout(() => {
            preloadNextChapter(chapterId, language);
          }, 100); // Small delay to not impact main content loading

          // Reset progress after a short delay
          setTimeout(() => setProgress(0), 300);
        } else {
          console.warn(`Translation not found for ${language}/${chapterId}, showing original content`);
          // Instead of setting error, fallback to original content
          setProgress(0);
        }
      } catch (err) {
        clearInterval(progressInterval);
        console.error(`Translation load failed for ${language}/${chapterId}:`, err);

        // For slow network handling: implement retry logic with exponential backoff
        const retryLoad = async (retries = 3, delay = 1000) => {
          for (let i = 0; i < retries; i++) {
            try {
              setProgress(20 + (i * 20)); // Show retry progress
              await new Promise(resolve => setTimeout(resolve, delay * (i + 1))); // Exponential backoff
              const retryData = await loadTranslation(language, chapterId);
              if (retryData) {
                setProgress(100);
                setTranslatedData(retryData);
                setSuccess(true);
                setTimeout(() => setSuccess(false), 2000);
                setTimeout(() => setProgress(0), 300);
                return; // Success, exit retry loop
              }
            } catch (retryErr) {
              console.warn(`Retry ${i + 1} failed:`, retryErr);
              if (i === retries - 1) {
                // Final attempt failed, fallback to original content
                console.info('Translation unavailable, showing original content');
              }
            }
          }
        };

        // Start retry process for slow networks
        await retryLoad();

        setProgress(0);
      } finally {
        setLoading(false);
      }
    };

    loadTranslationData();
  }, [language, chapterId, loadTranslation]);  // Run when language, chapterId, or loadTranslation function changes

  // Render loading state with progress indicator
  if (loading) {
    return (
      <div className={styles.translatedContent} data-language={language}>
        <div className={styles.loadingState}>
          <div className={styles.progressBarContainer}>
            <div
              className={styles.progressBar}
              style={{ width: `${progress}%` }}
            ></div>
          </div>
          <div className={styles.spinner}></div>
          <p>Loading translation... {progress}%</p>
        </div>
      </div>
    );
  }

  // Render error state with fallback to original content
  if (error && language !== 'en') {
    console.info('Translation failed, showing original content as fallback');
    return (
      <div className={styles.translatedContent} data-language={language}>
        <div className={styles.errorState}>
          <p>Translation not available. Showing original content.</p>
        </div>
        <div className={styles.originalContent}>
          {children}
        </div>
      </div>
    );
  }

  // Render translated content with RTL support
  if (translatedData && language !== 'en') {
    // Apply language-specific attributes for RTL support
    const isRTL = language === 'ur' || language === 'ar';

    return (
      <div
        className={styles.translatedContent}
        data-language={language}
        dir={isRTL ? 'rtl' : 'ltr'}  // Set text direction based on language
      >
        {success && (
          <div className={styles.successState}>
            <span className={styles.successIcon}>✓</span>
            <span>Translation loaded successfully</span>
          </div>
        )}
        <div
          className={styles.translatedHtml}
          dangerouslySetInnerHTML={{ __html: translatedData.html }}
        />
      </div>
    );
  }

  // Default to original content if no translation or English selected
  return (
    <div className={styles.translatedContent} data-language="en">
      <div className={styles.originalContent}>
        {children}
      </div>
    </div>
  );
};

export default TranslatedContent;