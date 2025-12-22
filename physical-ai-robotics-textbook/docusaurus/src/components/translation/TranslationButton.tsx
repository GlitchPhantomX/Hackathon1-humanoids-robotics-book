import React from "react";
import "../../css/translation.css"

interface Props {
  language: "en" | "ur";
  loading: boolean;
  onClick: () => void;
}

export default function TranslationButton({ language, loading, onClick }: Props) {
  return (
    <button
      className={`translation-btn ${language === "ur" ? "active" : ""}`}
      onClick={onClick}
      disabled={loading}
      aria-label="Toggle Urdu Translation"
    >
      {loading ? "Translating..." : language === "ur" ? "EN" : "اردو"}
    </button>
  );
}
