import React, { memo } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import LanguageToggle from '@site/src/components/LanguageToggle';
import styles from './styles.module.css';

const Navbar = memo(function Navbar(props) {
  return (
    <div className={styles.navbarContainer}>
      <OriginalNavbar {...props} />
      <div className={styles.languageToggleWrapper}>
        <LanguageToggle />
      </div>
    </div>
  );
});

export default Navbar;
