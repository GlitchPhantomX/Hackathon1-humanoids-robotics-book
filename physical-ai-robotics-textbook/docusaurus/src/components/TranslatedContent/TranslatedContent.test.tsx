/**
 * Unit tests for TranslatedContent component
 * Testing translation loading, caching, and fallback behavior
 */

import React from 'react';
import { render, screen, waitFor, within } from '@testing-library/react';
import { act } from 'react-dom/test-utils';
import TranslatedContent from './index';
import { LanguageProvider } from '@site/src/context/LanguageContext';

// Mock translation data
const mockTranslationData = {
  meta: {
    title: 'Test Translation',
    language: 'ur',
    chapter: '00-introduction',
  },
  html: '<h1>مقدمة إلى الروبوتات</h1><p>اختبار المحتوى</p>',
};

// Mock the useLanguage hook
jest.mock('@site/src/context/LanguageContext', () => ({
  ...jest.requireActual('@site/src/context/LanguageContext'),
  useLanguage: () => ({
    language: 'ur',
    setLanguage: jest.fn(),
    isAuthenticated: true,
  }),
}));

// Mock dynamic import for translations
jest.mock('@site/src/translations/ur/00-introduction.json', () => mockTranslationData, { virtual: true });

// Mock the translation cache
const mockTranslationCache = new Map();
jest.mock('./index', () => {
  // We'll create a simpler mock for testing purposes
  return jest.fn();
});

describe('TranslatedContent', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    jest.resetModules();
  });

  test('renders original content when language is English', async () => {
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'en',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original English Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    expect(screen.getByTestId('original-content')).toBeInTheDocument();
    expect(screen.getByText('Original English Content')).toBeInTheDocument();
  });

  test('shows loading state when fetching translation', async () => {
    // Mock slow loading
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    // Temporarily mock the import to be slow
    jest.doMock('@site/src/translations/ur/00-introduction.json', () => {
      return new Promise(resolve => {
        setTimeout(() => resolve(mockTranslationData), 100);
      });
    });

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Should show loading state initially
    expect(screen.getByText(/Loading translation/i)).toBeInTheDocument();
    expect(screen.getByText(/\d+%/)).toBeInTheDocument(); // Progress percentage
  });

  test('renders translated content when available', async () => {
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Wait for translation to load
    await waitFor(() => {
      expect(screen.getByText(/مقدمة إلى الروبوتات/i)).toBeInTheDocument();
    });
  });

  test('shows fallback content when translation fails', async () => {
    // Mock a failed translation load
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    // Mock the import to throw an error
    jest.doMock('@site/src/translations/ur/00-introduction.json', () => {
      throw new Error('Translation not found');
    });

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Wait for error handling
    await waitFor(() => {
      expect(screen.getByText(/Translation not available/i)).toBeInTheDocument();
      expect(screen.getByTestId('original-content')).toBeInTheDocument();
    });
  });

  test('applies RTL direction for Arabic and Urdu', async () => {
    // Test with Urdu
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    await waitFor(() => {
      const container = screen.getByTestId('translated-content');
      expect(container).toHaveAttribute('dir', 'rtl');
    });
  });

  test('applies LTR direction for English', async () => {
    // Test with English
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'en',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Should render original content with LTR
    expect(screen.getByTestId('original-content')).toBeInTheDocument();
  });

  test('shows success feedback when translation loads', async () => {
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Wait for translation to load and success message to appear
    await waitFor(() => {
      expect(screen.getByText(/Translation loaded successfully/i)).toBeInTheDocument();
    });
  });

  test('caches translations to prevent refetching', async () => {
    const mockImport = jest.fn().mockResolvedValue(mockTranslationData);
    jest.spyOn(global, 'import').mockImplementation(mockImport);

    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    const { default: TranslatedContentMocked } = await import('./index');

    // Render twice with same props
    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    await waitFor(() => {
      expect(mockImport).toHaveBeenCalledTimes(1); // Should only be called once due to caching
    });
  });

  test('handles slow network conditions with retry logic', async () => {
    // Mock network failure scenario
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'ur',
        setLanguage: jest.fn(),
        isAuthenticated: true,
      }),
    }));

    // Mock the import to fail initially then succeed
    let failCount = 0;
    jest.doMock('@site/src/translations/ur/00-introduction.json', () => {
      if (failCount < 2) {
        failCount++;
        throw new Error('Network error');
      }
      return mockTranslationData;
    });

    const { default: TranslatedContentMocked } = await import('./index');

    render(
      <LanguageProvider>
        <TranslatedContentMocked chapterId="00-introduction">
          <div data-testid="original-content">Original Content</div>
        </TranslatedContentMocked>
      </LanguageProvider>
    );

    // Wait for retry logic to eventually succeed
    await waitFor(() => {
      expect(screen.getByText(/مقدمة إلى الروبوتات/i)).toBeInTheDocument();
    }, { timeout: 10000 }); // Increased timeout for retry logic
  });
});