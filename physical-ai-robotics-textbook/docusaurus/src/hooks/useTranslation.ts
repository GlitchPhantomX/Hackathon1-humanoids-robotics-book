import { useState, useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';

/**
 * Type definition for translation data structure
 * Matches the expected structure of translation JSON files
 */
interface TranslationData {
  title?: string;
  description?: string;
  content?: {
    [key: string]: string;
  };
  [key: string]: any; // Allow additional properties
}

/**
 * Custom hook for loading and using translations
 * Loads JSON files from src/translations/{locale}/{path}.json
 *
 * @param path - The path to the translation file (e.g., '00-introduction/01-welcome')
 * @returns Translation data object with all translation keys
 *
 * Example usage:
 * const t = useTranslation('00-introduction/01-welcome');
 * return <h1>{t.title}</h1>
 */
export const useTranslation = (path: string): TranslationData => {
  const { language } = useLanguage();
  const [translation, setTranslation] = useState<TranslationData>({});

  useEffect(() => {
    const loadTranslation = async () => {
      try {
        // Construct the import path based on language and provided path
        // Using dynamic import to load translation JSON files
        const translationPath = `../translations/${language}/${path}`;

        // Import the translation file
        const translationModule = await import(
          /* webpackMode: "lazy" */
          /* webpackChunkName: "translation-[request]" */
          `${translationPath}.json`
        );

        setTranslation(translationModule.default || translationModule);
      } catch (error) {
        console.warn(`Translation file not found for path: ${path} in language: ${language}`, error);
        // Return empty object if translation file doesn't exist
        setTranslation({});
      }
    };

    loadTranslation();
  }, [path, language]);

  return translation;
};

/**
 * Type-safe version of useTranslation with generic typing
 *
 * @param path - The path to the translation file
 * @param fallback - Optional fallback translation object
 * @returns Translation data with type safety
 */
export const useTypedTranslation = <T extends TranslationData>(
  path: string,
  fallback: T = {} as T
): T => {
  const { language } = useLanguage();
  const [translation, setTranslation] = useState<T>(fallback as T);

  useEffect(() => {
    const loadTranslation = async () => {
      try {
        const translationPath = `../translations/${language}/${path}`;
        const translationModule = await import(
          /* webpackMode: "lazy" */
          /* webpackChunkName: "translation-[request]" */
          `${translationPath}.json`
        );

        setTranslation(translationModule.default || translationModule || fallback);
      } catch (error) {
        console.warn(`Translation file not found for path: ${path} in language: ${language}`, error);
        setTranslation(fallback);
      }
    };

    loadTranslation();
  }, [path, language, fallback]);

  return translation;
};