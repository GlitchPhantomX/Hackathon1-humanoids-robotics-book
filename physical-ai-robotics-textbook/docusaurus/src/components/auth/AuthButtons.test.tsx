// AuthButtons Component Test Suite
// This file documents the expected behavior and test cases for the AuthButtons component

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import AuthButtons from './AuthButtons';

// Mock the useSession hook and auth client
jest.mock('better-auth/react', () => ({
  useSession: jest.fn()
}));

jest.mock('../../lib/auth-client', () => ({
  authClient: {
    signOut: jest.fn()
  }
}));

// Import the mocked modules
const { useSession } = require('better-auth/react');
const { authClient } = require('../../lib/auth-client');

describe('AuthButtons', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('shows login/signup buttons when user is not authenticated', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: false
    });

    render(<AuthButtons />);

    expect(screen.getByText('Login')).toBeInTheDocument();
    expect(screen.getByText('Sign Up')).toBeInTheDocument();
  });

  test('shows loading state while checking session', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: true
    });

    render(<AuthButtons />);

    expect(screen.getByText('Loading...')).toBeInTheDocument();
  });

  test('shows user profile and logout button when user is authenticated', () => {
    const mockSession = {
      user: {
        name: 'John Doe',
        email: 'john@example.com'
      }
    };

    (useSession as jest.Mock).mockReturnValue({
      data: mockSession,
      isPending: false
    });

    render(<AuthButtons />);

    expect(screen.getByText('Welcome, John Doe!')).toBeInTheDocument();
    expect(screen.getByText('Logout')).toBeInTheDocument();
  });

  test('calls signOut when logout button is clicked', async () => {
    const mockSession = {
      user: {
        name: 'John Doe',
        email: 'john@example.com'
      }
    };

    (useSession as jest.Mock).mockReturnValue({
      data: mockSession,
      isPending: false
    });

    (authClient.signOut as jest.Mock).mockResolvedValue({ error: null });

    render(<AuthButtons />);

    fireEvent.click(screen.getByText('Logout'));

    await waitFor(() => {
      expect(authClient.signOut).toHaveBeenCalled();
    });
  });

  test('handles logout error gracefully', async () => {
    const mockSession = {
      user: {
        name: 'John Doe',
        email: 'john@example.com'
      }
    };

    (useSession as jest.Mock).mockReturnValue({
      data: mockSession,
      isPending: false
    });

    const mockError = { message: 'Logout failed' };
    (authClient.signOut as jest.Mock).mockResolvedValue({ error: mockError });

    console.error = jest.fn(); // Mock console.error to avoid actual logging

    render(<AuthButtons />);

    fireEvent.click(screen.getByText('Logout'));

    await waitFor(() => {
      expect(authClient.signOut).toHaveBeenCalled();
      expect(console.error).toHaveBeenCalledWith('Logout error:', mockError);
    });
  });

  test('opens login modal when login button is clicked', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: false
    });

    render(<AuthButtons />);

    fireEvent.click(screen.getByText('Login'));

    // The modal behavior would be tested in integration tests
    expect(screen.getByText('Sign In')).toBeInTheDocument(); // Assuming LoginModal renders this
  });

  test('opens signup modal when signup button is clicked', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: false
    });

    render(<AuthButtons />);

    fireEvent.click(screen.getByText('Sign Up'));

    // The modal behavior would be tested in integration tests
    expect(screen.getByText('Create Account')).toBeInTheDocument(); // Assuming SignupModal renders this
  });
});

// Manual testing checklist for Phase 4:
// 1. Verify LoginModal opens and displays email/password form
// 2. Verify login form validation works (email format, required fields)
// 3. Verify login submission calls Better Auth API correctly
// 4. Verify error messages display for invalid login attempts
// 5. Verify loading states show during login process
// 6. Verify AuthButtons component shows login/signup when not authenticated
// 7. Verify AuthButtons component shows user profile and logout when authenticated
// 8. Verify session state updates correctly after login/logout
// 9. Verify logout functionality works and updates UI accordingly
// 10. Verify session persists across page refreshes (requires browser testing)
// 11. Verify useSession hook properly tracks authentication state
// 12. Verify all components properly handle loading states