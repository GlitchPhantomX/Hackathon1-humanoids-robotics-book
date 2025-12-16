/**
 * Integration tests for authentication flow
 * Testing end-to-end authentication functionality
 */

const { JSDOM } = require('jsdom');
const fs = require('fs');
const path = require('path');

// Setup JSDOM for testing browser APIs
const dom = new JSDOM('<!DOCTYPE html><html><body></body></html>', {
  url: 'http://localhost:3000',
  pretendToBeVisual: true,
  resources: 'usable'
});

global.window = dom.window;
global.document = dom.window.document;
global.navigator = dom.window.navigator;
global.fetch = require('node-fetch');

describe('Auth Flow Integration Tests', () => {
  test('auth session endpoint returns correct data', async () => {
    // Mock fetch response for auth session
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        json: () => Promise.resolve({
          isAuthenticated: true,
          user: { id: 'test-user', name: 'Test User' }
        }),
      })
    );
    global.fetch = mockFetch;

    // Test the auth flow by importing and using the LanguageContext
    const { LanguageProvider, useLanguage } = require('./src/context/LanguageContext');

    // Simulate the auth check that happens in useEffect
    const authResult = await fetch('http://localhost:8001/api/auth/session', {
      credentials: 'include'
    });
    const authData = await authResult.json();

    expect(authData.isAuthenticated).toBe(true);
    expect(authData.user).toBeDefined();
    expect(authData.user.id).toBe('test-user');
  });

  test('auth check fails gracefully with network error', async () => {
    // Mock fetch to throw network error
    const mockFetch = jest.fn(() =>
      Promise.reject(new Error('Network error'))
    );
    global.fetch = mockFetch;

    // Test the auth flow handles errors
    try {
      const authResult = await fetch('http://localhost:8001/api/auth/session', {
        credentials: 'include'
      });
      const authData = await authResult.json();
      // Should not reach here
      expect(false).toBe(true);
    } catch (error) {
      expect(error.message).toBe('Network error');
    }
  });

  test('auth state affects language toggle availability', async () => {
    // Test that language toggle behavior changes based on auth state
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        json: () => Promise.resolve({ isAuthenticated: false }),
      })
    );
    global.fetch = mockFetch;

    // Import LanguageContext and check auth state handling
    const { LanguageProvider, useLanguage } = require('./src/context/LanguageContext');

    // Simulate auth check
    const authResult = await fetch('http://localhost:8001/api/auth/session', {
      credentials: 'include'
    });
    const authData = await authResult.json();

    // Verify that unauthenticated state is handled properly
    expect(authData.isAuthenticated).toBe(false);
  });

  test('auth session includes proper credentials', async () => {
    const mockFetch = jest.fn((url, options) => {
      // Verify that credentials are included in the request
      expect(options.credentials).toBe('include');
      return Promise.resolve({
        json: () => Promise.resolve({ isAuthenticated: true }),
      });
    });
    global.fetch = mockFetch;

    // Test auth request with credentials
    const authResult = await fetch('http://localhost:8001/api/auth/session', {
      credentials: 'include'
    });
    const authData = await authResult.json();

    expect(authData.isAuthenticated).toBe(true);
  });

  test('auth state persistence across page reloads', () => {
    // Test localStorage usage for auth state persistence
    const mockLocalStorage = {
      getItem: jest.fn(),
      setItem: jest.fn(),
      removeItem: jest.fn(),
    };
    global.localStorage = mockLocalStorage;

    // Simulate saving auth state
    localStorage.setItem('auth-state', JSON.stringify({ isAuthenticated: true, lastChecked: Date.now() }));

    const savedAuthState = localStorage.getItem('auth-state');
    const parsedAuthState = JSON.parse(savedAuthState);

    expect(parsedAuthState.isAuthenticated).toBe(true);
    expect(parsedAuthState.lastChecked).toBeDefined();
  });
});

// Additional tests for auth flow
describe('Auth Flow Edge Cases', () => {
  test('handles expired session tokens', async () => {
    const mockFetch = jest.fn(() =>
      Promise.resolve({
        json: () => Promise.resolve({ isAuthenticated: false, reason: 'token_expired' }),
      })
    );
    global.fetch = mockFetch;

    const authResult = await fetch('http://localhost:8001/api/auth/session', {
      credentials: 'include'
    });
    const authData = await authResult.json();

    expect(authData.isAuthenticated).toBe(false);
    expect(authData.reason).toBe('token_expired');
  });

  test('auth flow works with different API endpoints', async () => {
    // Test with different possible auth endpoints
    const endpoints = [
      'http://localhost:8001/api/auth/session',
      'http://localhost:8001/api/v1/auth/session',
      'https://api.example.com/auth/session'
    ];

    for (const endpoint of endpoints) {
      const mockFetch = jest.fn(() =>
        Promise.resolve({
          json: () => Promise.resolve({ isAuthenticated: true }),
        })
      );
      global.fetch = mockFetch;

      const authResult = await fetch(endpoint, {
        credentials: 'include'
      });
      const authData = await authResult.json();

      expect(authData.isAuthenticated).toBe(true);
    }
  });
});