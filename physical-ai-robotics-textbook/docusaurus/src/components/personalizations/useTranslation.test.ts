/**
 * Unit tests for useTranslation hook
 */

import { renderHook, act } from '@testing-library/react';
import { useTranslation } from './useTranslation';

// Mock the fetch API
global.fetch = jest.fn();

describe('useTranslation hook', () => {
  const chapterId = 'test-chapter';
  const initialContent = '# Test Content\n\nThis is test content.';

  beforeEach(() => {
    jest.clearAllMocks();
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockClear();
  });

  test('initializes with correct default state', () => {
    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    expect(result.current.translatedContent).toBe(initialContent);
    expect(result.current.isTranslating).toBe(false);
    expect(result.current.error).toBeNull();
    expect(result.current.currentLanguage).toBe('en');
  });

  test('toggles language from English to Urdu', async () => {
    // Mock successful API response
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        translated_content: '# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔',
        cached: false
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Initially in English
    expect(result.current.currentLanguage).toBe('en');

    // Toggle language
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Should now be in Urdu
    expect(result.current.currentLanguage).toBe('ur');
    expect(result.current.isTranslating).toBe(false);
    expect(result.current.error).toBeNull();
  });

  test('toggles language from Urdu back to English', async () => {
    // First, switch to Urdu
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        translated_content: '# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔',
        cached: false
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Toggle to Urdu first
    await act(async () => {
      await result.current.toggleLanguage();
    });

    expect(result.current.currentLanguage).toBe('ur');

    // Toggle back to English
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Should be back to English
    expect(result.current.currentLanguage).toBe('en');
    expect(result.current.translatedContent).toBe(initialContent);
    expect(result.current.error).toBeNull();
  });

  test('handles API error gracefully', async () => {
    // Mock API error
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: false,
      status: 500,
      json: async () => ({
        detail: 'Internal server error'
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Toggle language (should fail)
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Should remain in English but show error
    expect(result.current.currentLanguage).toBe('en');
    expect(result.current.error).toContain('HTTP error!');
  });

  test('sets loading state during translation', async () => {
    // Mock API to resolve after a delay
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockImplementationOnce(() => {
      return new Promise(resolve => {
        setTimeout(() => {
          resolve({
            ok: true,
            json: async () => ({
              translated_content: '# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔',
              cached: false
            })
          });
        }, 10);
      }) as Promise<Response>;
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Initially not loading
    expect(result.current.isTranslating).toBe(false);

    // Start translation
    const togglePromise = act(async () => {
      await result.current.toggleLanguage();
    });

    // Should be loading during the request
    expect(result.current.isTranslating).toBe(true);

    // Wait for the promise to complete
    await togglePromise;

    // Should no longer be loading after completion
    expect(result.current.isTranslating).toBe(false);
  });

  test('uses cached response when available', async () => {
    // Mock cached response
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        translated_content: '# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔',
        cached: true
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Toggle language
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Should be in Urdu with cached content
    expect(result.current.currentLanguage).toBe('ur');
    expect(result.current.translatedContent).toBe('# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔');
  });

  test('handles authentication status checking', async () => {
    // The hook should check authentication status
    // Mock a successful translation
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        translated_content: '# ترجمہ شدہ مواد\n\nیہ ٹیسٹ مواد ہے۔',
        cached: false
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Toggle language
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Verify fetch was called with correct parameters
    expect(global.fetch).toHaveBeenCalledWith(
      '/api/translation/urdu',
      expect.objectContaining({
        method: 'POST',
        headers: expect.objectContaining({
          'Content-Type': 'application/json'
        }),
        body: expect.any(String)
      })
    );

    // Parse the body to verify content
    const callArgs = (global.fetch as jest.MockedFunction<typeof global.fetch>).mock.calls[0];
    const requestBody = JSON.parse(callArgs[1].body as string);

    expect(requestBody).toEqual({
      chapter_id: chapterId,
      source_language: 'en',
      target_language: 'ur',
      content: initialContent,
      user_id: 'anonymous' // This is the fallback in our implementation
    });
  });

  test('preserves initial content when error occurs during translation', async () => {
    // Mock API error
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
      ok: false,
      status: 401,
      json: async () => ({
        detail: 'Unauthorized'
      })
    });

    const { result } = renderHook(() => useTranslation(chapterId, initialContent));

    // Toggle language (should fail)
    await act(async () => {
      await result.current.toggleLanguage();
    });

    // Should remain in English with original content
    expect(result.current.currentLanguage).toBe('en');
    expect(result.current.translatedContent).toBe(initialContent);
    expect(result.current.error).toContain('HTTP error!');
  });
});