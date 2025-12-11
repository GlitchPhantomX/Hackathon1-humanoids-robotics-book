// UserProfile Component Test Suite
// This file documents the expected behavior and test cases for the user profile display functionality

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

describe('UserProfile (within AuthButtons)', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('displays user avatar with first initial when authenticated', () => {
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

    // Check that the avatar displays the first initial
    expect(screen.getByText('J')).toBeInTheDocument();
  });

  test('shows user profile dropdown when avatar is clicked', async () => {
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

    // Click the avatar button
    fireEvent.click(screen.getByText('J'));

    // Wait for dropdown to appear
    await waitFor(() => {
      expect(screen.getByText('John Doe')).toBeInTheDocument();
      expect(screen.getByText('john@example.com')).toBeInTheDocument();
    });
  });

  test('hides dropdown when clicking outside', async () => {
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

    // Click the avatar button to show dropdown
    fireEvent.click(screen.getByText('J'));

    await waitFor(() => {
      expect(screen.getByText('John Doe')).toBeInTheDocument();
    });

    // Click outside the dropdown area
    fireEvent.mouseDown(document.body);

    // Wait for dropdown to disappear
    await waitFor(() => {
      expect(screen.queryByText('John Doe')).not.toBeInTheDocument();
    });
  });

  test('calls logout when sign out is clicked in dropdown', async () => {
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

    // Click the avatar button to show dropdown
    fireEvent.click(screen.getByText('J'));

    // Wait for dropdown to appear and click sign out
    await waitFor(() => {
      fireEvent.click(screen.getByText('Sign Out'));
    });

    await waitFor(() => {
      expect(authClient.signOut).toHaveBeenCalled();
    });
  });

  test('displays correct user information in dropdown', async () => {
    const mockSession = {
      user: {
        name: 'Jane Smith',
        email: 'jane@example.com'
      }
    };

    (useSession as jest.Mock).mockReturnValue({
      data: mockSession,
      isPending: false
    });

    render(<AuthButtons />);

    // Click the avatar button to show dropdown
    fireEvent.click(screen.getByText('J')); // First initial of Jane

    // Wait for dropdown to appear and verify content
    await waitFor(() => {
      expect(screen.getByText('Jane Smith')).toBeInTheDocument();
      expect(screen.getByText('jane@example.com')).toBeInTheDocument();
    });
  });

  test('handles user with no name gracefully', () => {
    const mockSession = {
      user: {
        name: '',
        email: 'user@example.com'
      }
    };

    (useSession as jest.Mock).mockReturnValue({
      data: mockSession,
      isPending: false
    });

    render(<AuthButtons />);

    // Should show 'U' as default initial when name is empty
    expect(screen.getByText('U')).toBeInTheDocument();
  });
});

// Manual testing checklist for Phase 5:
// 1. Verify user avatar displays first initial of user name
// 2. Verify clicking avatar opens dropdown menu
// 3. Verify dropdown shows user name and email
// 4. Verify dropdown closes when clicking outside
// 5. Verify logout button in dropdown works correctly
// 6. Verify dropdown styling matches overall theme
// 7. Verify dropdown positioning is correct in navbar
// 8. Verify avatar is circular and properly sized
// 9. Verify user information updates when session changes
// 10. Verify dropdown works on different screen sizes (responsive)