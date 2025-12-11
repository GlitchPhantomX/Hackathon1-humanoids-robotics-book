import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useLocation } from '@docusaurus/router';
import AuthButtons from '../auth/AuthButtons';

const CustomNavbar: React.FC = () => {
  const location = useLocation();

  // Only render auth buttons on the client side
  return (
    <BrowserOnly>
      {() => <AuthButtons />}
    </BrowserOnly>
  );
};

export default CustomNavbar;