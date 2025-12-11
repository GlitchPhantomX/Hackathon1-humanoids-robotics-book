import React, { useEffect } from "react";
import Layout from "@theme-original/Layout";
import { useColorMode } from "@docusaurus/theme-common";

export default function LayoutWrapper(props) {
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

  return <Layout {...props} />;
}