import React from 'react';
import { TranslationProvider } from '@site/src/contexts/TranslationContext';

export default function Root({ children }) {
  return (
    <TranslationProvider>
      {children}
    </TranslationProvider>
  );
}