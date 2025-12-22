import React from "react";
import { useLocation } from "@docusaurus/router";
import useTranslation from "./useTranslation";

export default function NavbarLanguageToggle() {
  const location = useLocation();

  if (!location.pathname.startsWith("/docs")) {
    return null;
  }

  const chapterId = location.pathname.replace("/docs/", "");
  const { language, loading, toggleLanguage } = useTranslation(chapterId);

  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault(); // ✅ Prevent default behavior
    e.stopPropagation(); // ✅ Stop event bubbling
    console.log('🖱️ Button clicked!');
    toggleLanguage();
  };

  return (
    <button
      type="button" // ✅ Explicitly set type
      onClick={handleClick}
      disabled={loading}
      style={{
        display: 'inline-block', // ✅ Add this
        background: language === 'ur' ? '#007bff' : '#fff',
        border: '2px solid #007bff',
        borderRadius: '6px',
        padding: '8px 16px',
        cursor: loading ? 'not-allowed' : 'pointer',
        fontWeight: 'bold',
        fontSize: '14px',
        marginLeft: '10px',
        color: language === 'ur' ? '#fff' : '#007bff',
        opacity: loading ? 0.6 : 1,
        transition: 'all 0.2s ease',
        textDecoration: 'none', // ✅ Remove link styling
        pointerEvents: loading ? 'none' : 'auto', // ✅ Disable when loading
      }}
      onMouseEnter={(e) => {
        if (!loading) {
          e.currentTarget.style.background = language === 'ur' ? '#0056b3' : '#f0f0f0';
        }
      }}
      onMouseLeave={(e) => {
        e.currentTarget.style.background = language === 'ur' ? '#007bff' : '#fff';
      }}
      title={language === 'en' ? 'Translate to Urdu' : 'Switch to English'}
    >
      {loading ? '⏳ Loading...' : language === 'en' ? '🇵🇰 اردو' : '🇬🇧 EN'}
    </button>
  );
}