import React from "react";
import { useLocation } from "@docusaurus/router";
// @ts-ignore
import useTranslation from "./useTranslation";
import "./translation.css";

export default function NavbarItem() {
  const location = useLocation();

  // sirf docs pages par
  if (!location.pathname.startsWith("/docs")) {
    return null;
  }

  const chapterId = location.pathname.replace("/docs/", "");

  const article = document.querySelector("article");
  if (!article) return null;

  const { language, loading, toggleLanguage } =
    useTranslation(chapterId);

  return (
    <button
      className={`translation-btn navbar-btn ${
        language === "ur" ? "active" : ""
      }`}
      onClick={() => toggleLanguage(article.innerHTML)}
      disabled={loading}
    >
      {language === "ur" ? "EN" : "اردو"}
    </button>
  );
}
