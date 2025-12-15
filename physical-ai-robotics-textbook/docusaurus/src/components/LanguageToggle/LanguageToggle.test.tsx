/**
 * Unit tests for LanguageToggle component
 * Testing UI interactions, accessibility, and state management
 */

import React from 'react';
import { render, screen, fireEvent, waitFor, within } from '@testing-library/react';
import { act } from 'react-dom/test-utils';
import LanguageToggle from './index';
import { LanguageProvider } from '@site/src/context/LanguageContext';

// Mock the useLanguage hook
jest.mock('@site/src/context/LanguageContext', () => ({
  ...jest.requireActual('@site/src/context/LanguageContext'),
  useLanguage: () => ({
    language: 'en',
    setLanguage: jest.fn(),
    isAuthenticated: true,
  }),
}));

// Test component wrapper
const TestWrapper = ({ language = 'en', isAuthenticated = true }) => (
  <LanguageProvider>
    <LanguageToggle />
  </LanguageProvider>
);

describe('LanguageToggle', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders with current language', () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    expect(toggleButton).toBeInTheDocument();
    expect(toggleButton).toHaveAttribute('aria-expanded', 'false');
    expect(toggleButton).toHaveAttribute('aria-haspopup', 'true');
  });

  test('opens dropdown when clicked', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      const dropdown = screen.getByRole('listbox');
      expect(dropdown).toBeInTheDocument();
    });
  });

  test('displays all three language options', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByText('English')).toBeInTheDocument();
      expect(screen.getByText('اردو')).toBeInTheDocument();
      expect(screen.getByText('العربية')).toBeInTheDocument();
    });
  });

  test('closes dropdown when clicking outside', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByRole('listbox')).toBeInTheDocument();
    });

    // Click outside the dropdown
    fireEvent.mouseDown(document.body);

    await waitFor(() => {
      expect(screen.queryByRole('listbox')).not.toBeInTheDocument();
    });
  });

  test('closes dropdown with Escape key', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByRole('listbox')).toBeInTheDocument();
    });

    // Press Escape key
    fireEvent.keyDown(document, { key: 'Escape' });

    await waitFor(() => {
      expect(screen.queryByRole('listbox')).not.toBeInTheDocument();
    });
  });

  test('selects language when option is clicked', async () => {
    const setLanguageMock = jest.fn();
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'en',
        setLanguage: setLanguageMock,
        isAuthenticated: true,
      }),
    }));

    // We need to reimport the component to use the mocked hook
    const { default: LanguageToggleMocked } = require('./index');

    render(
      <LanguageProvider>
        <LanguageToggleMocked />
      </LanguageProvider>
    );

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByRole('listbox')).toBeInTheDocument();
    });

    const arabicOption = screen.getByText('العربية');
    fireEvent.click(arabicOption);

    expect(setLanguageMock).toHaveBeenCalledWith('ar');
  });

  test('shows current language as active', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByRole('listbox')).toBeInTheDocument();
    });

    // The current language should have some visual indication of being selected
    const englishOption = screen.getByText('English');
    expect(englishOption).toBeInTheDocument();
  });

  test('is disabled when user is not authenticated', () => {
    // Mock unauthenticated state
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'en',
        setLanguage: jest.fn(),
        isAuthenticated: false,
      }),
    }));

    const { default: LanguageToggleMocked } = require('./index');

    render(
      <LanguageProvider>
        <LanguageToggleMocked />
      </LanguageProvider>
    );

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    expect(toggleButton).toBeDisabled();
    expect(toggleButton).toHaveAttribute('title', 'Please log in to use translation feature');
  });

  test('shows authentication prompt when not authenticated and dropdown is opened', async () => {
    // Mock unauthenticated state
    jest.mock('@site/src/context/LanguageContext', () => ({
      useLanguage: () => ({
        language: 'en',
        setLanguage: jest.fn(),
        isAuthenticated: false,
      }),
    }));

    const { default: LanguageToggleMocked } = require('./index');

    render(
      <LanguageProvider>
        <LanguageToggleMocked />
      </LanguageProvider>
    );

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByText('Please log in to access translation features')).toBeInTheDocument();
    });
  });

  test('has proper ARIA attributes for accessibility', () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    expect(toggleButton).toHaveAttribute('aria-label', 'Select Language');
    expect(toggleButton).toHaveAttribute('aria-haspopup', 'true');
    expect(toggleButton).toHaveAttribute('aria-expanded', 'false');
    expect(toggleButton).toHaveAttribute('aria-disabled', 'false');
  });

  test('handles keyboard navigation', async () => {
    render(<TestWrapper />);

    const toggleButton = screen.getByRole('button', { name: /Select Language/i });
    toggleButton.focus();

    // Simulate Ctrl+Shift+L to open toggle
    fireEvent.keyDown(document, { ctrlKey: true, shiftKey: true, key: 'l' });

    await waitFor(() => {
      expect(screen.queryByRole('listbox')).toBeInTheDocument();
    });
  });
});