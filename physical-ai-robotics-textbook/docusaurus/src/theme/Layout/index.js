import React, { useEffect } from "react";
import Layout from "@theme-original/Layout";
import { useColorMode } from "@docusaurus/theme-common";
import { useLocation } from "@docusaurus/router";
import NewChatbot from "../../components/NewChatbot";
import PersonalizeButton from "../../components/PersonalizeButton";

export default function LayoutWrapper(props) {
  const location = useLocation();
  
  try {
    const { colorMode } = useColorMode();

    useEffect(() => {
      const root = document.documentElement;
      if (colorMode === "dark") {
        root.classList.add("dark");
      } else {
        root.classList.remove("dark");
      }
    }, [colorMode]);
  } catch (e) {
    // Provider not available yet
  }

  // ✅ Only show PersonalizeButton on docs pages
  const isDocsPage = location.pathname.startsWith('/docs');
  const chapterId = isDocsPage ? location.pathname.replace('/docs/', '') : '';

  return (
    <>
      <Layout {...props} />
      <NewChatbot />
      {isDocsPage && <PersonalizeButton chapterId={chapterId} />}
    </>
  );
}