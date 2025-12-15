import React from 'react';
import { LanguageProvider } from '../context/LanguageContext';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <LanguageProvider>
      {children}
    </LanguageProvider>
  );
}
