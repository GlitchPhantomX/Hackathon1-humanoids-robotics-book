import React, { useEffect, useRef } from "react";
import TranslationButton from "./TranslationButton";
// @ts-ignore
import useTranslation from "./useTranslation";

export default function LanguageToggle({ chapterId }: { chapterId: string }) {
  const articleRef = useRef<HTMLElement | null>(null);

  useEffect(() => {
    articleRef.current = document.querySelector("article");
  }, []);

  if (!articleRef.current) return null;

  const originalHtml = articleRef.current.innerHTML;

  const { language, translatedHtml, loading, toggleLanguage } =
    useTranslation(chapterId);

  useEffect(() => {
    if (translatedHtml && articleRef.current) {
      articleRef.current.innerHTML = translatedHtml;
    } else if (language === "en" && articleRef.current) {
      articleRef.current.innerHTML = originalHtml;
    }
  }, [translatedHtml, language]);

  return (
    <div style={{ marginBottom: "1rem" }}>
      <TranslationButton
        language={language}
        loading={loading}
        onClick={() => toggleLanguage(originalHtml)}
      />
    </div>
  );
}
