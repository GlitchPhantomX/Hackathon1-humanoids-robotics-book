// useTranslation.d.ts
export interface UseTranslationReturn {
  language: 'en' | 'ur';
  loading: boolean;
  toggleLanguage: () => Promise<void>;
}

export default function useTranslation(chapterId: string): UseTranslationReturn;