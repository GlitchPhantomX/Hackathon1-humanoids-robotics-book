import React from "react";
import OriginalNavbar from "@theme-original/Navbar";
import NavbarLanguageToggle from "@site/src/components/translation/NavbarLanguageToggle";
import Banner from "../../components/Banner";

export default function Navbar(props) {
  return (
    <>
      <Banner />
      <div style={{ position: "relative" }}>
        <OriginalNavbar {...props} />
        {/* ✅ Make sure button is NOT inside an <a> tag */}
        <div
          style={{
            position: "absolute",
            top: "50%",
            right: "80px",
            transform: "translateY(-50%)",
            zIndex: 100,
            display: "flex",
            alignItems: "center",
            pointerEvents: "auto", // ✅ Add this
          }}
        >
          <NavbarLanguageToggle />
        </div>
      </div>
    </>
  );
}