/**
 * Unit tests for LanguageContext
 * Testing language state management, authentication, and persistence
 */

import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import { LanguageProvider, useLanguage } from './LanguageContext';

// Mock fetch for auth testing
global.fetch = jest.fn(() =>
  Promise.resolve({
    json: () => Promise.resolve({ isAuthenticated: true }),
  } as Response)
) as jest.Mock;

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};
global.localStorage = localStorageMock;

// Test component to access context
const TestComponent = () => {
  const { language, setLanguage, isAuthenticated } = useLanguage();
  return (
    <div>
      <span data-testid="current-language">{language}</span>
      <span data-testid="is-authenticated">{isAuthenticated.toString()}</span>
      <button onClick={() => setLanguage('ur')}>Set Urdu</button>
      <button onClick={() => setLanguage('ar')}>Set Arabic</button>
      <button onClick={() => setLanguage('en')}>Set English</button>
    </div>
  );
};

describe('LanguageContext', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    localStorageMock.getItem.mockClear();
    localStorageMock.setItem.mockClear();
  });

  test('provides initial language state', async () => {
    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for auth check to complete
    await waitFor(() => {
      expect(screen.getByTestId('current-language')).toHaveTextContent('en');
      expect(screen.getByTestId('is-authenticated')).toHaveTextContent('true');
    });
  });

  test('updates language state when setLanguage is called', async () => {
    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for initial render
    await waitFor(() => {
      expect(screen.getByTestId('current-language')).toHaveTextContent('en');
    });

    // Change language to Urdu
    const setUrduButton = screen.getByText('Set Urdu');
    act(() => {
      setUrduButton.click();
    });

    expect(screen.getByTestId('current-language')).toHaveTextContent('ur');
  });

  test('updates language state to Arabic', async () => {
    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for initial render
    await waitFor(() => {
      expect(screen.getByTestId('current-language')).toHaveTextContent('en');
    });

    // Change language to Arabic
    const setArabicButton = screen.getByText('Set Arabic');
    act(() => {
      setArabicButton.click();
    });

    expect(screen.getByTestId('current-language')).toHaveTextContent('ar');
  });

  test('persists language preference in localStorage', async () => {
    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for initial render
    await waitFor(() => {
      expect(screen.getByTestId('current-language')).toHaveTextContent('en');
    });

    // Change language
    const setUrduButton = screen.getByText('Set Urdu');
    act(() => {
      setUrduButton.click();
    });

    expect(localStorageMock.setItem).toHaveBeenCalledWith('preferred-language', 'ur');
  });

  test('loads language preference from localStorage when authenticated', async () => {
    localStorageMock.getItem.mockReturnValue('ar');

    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for auth check and preference load
    await waitFor(() => {
      expect(screen.getByTestId('current-language')).toHaveTextContent('ar');
    });
  });

  test('does not update language when not authenticated', async () => {
    // Mock auth check to return false
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      json: () => Promise.resolve({ isAuthenticated: false }),
    } as Response);

    // Mock alert to prevent actual alert from appearing
    const alertMock = jest.spyOn(window, 'alert').mockImplementation(() => {});

    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Wait for auth check
    await waitFor(() => {
      expect(screen.getByTestId('is-authenticated')).toHaveTextContent('false');
    });

    // Try to change language
    const setUrduButton = screen.getByText('Set Urdu');
    act(() => {
      setUrduButton.click();
    });

    // Should still be 'en' since user is not authenticated
    expect(screen.getByTestId('current-language')).toHaveTextContent('en');
    expect(alertMock).toHaveBeenCalledWith('Please log in to use translation feature');

    alertMock.mockRestore();
  });

  test('handles auth check failure gracefully', async () => {
    // Mock auth check to throw error
    (global.fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

    render(
      <LanguageProvider>
        <TestComponent />
      </LanguageProvider>
    );

    // Should default to not authenticated on error
    await waitFor(() => {
      expect(screen.getByTestId('is-authenticated')).toHaveTextContent('false');
    });
  });
});