// Navbar Integration Test Suite
// This file documents the expected behavior and test cases for the navbar authentication integration

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { AuthProvider } from 'better-auth/react';
import AuthButtons from './AuthButtons';
import { authClient } from '../../lib/auth-client';

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

describe('Navbar Integration', () => {
  const MockApp = ({ sessionData = null }) => (
    <AuthProvider client={authClient}>
      <AuthButtons />
    </AuthProvider>
  );

  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders login/signup buttons when not authenticated in navbar context', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: false
    });

    render(<MockApp />);

    expect(screen.getByText('Login')).toBeInTheDocument();
    expect(screen.getByText('Sign Up')).toBeInTheDocument();
  });

  test('renders user avatar and dropdown when authenticated in navbar context', async () => {
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

    render(<MockApp />);

    // Check that the avatar displays the first initial
    expect(screen.getByText('J')).toBeInTheDocument();

    // Click the avatar to open dropdown
    fireEvent.click(screen.getByText('J'));

    await waitFor(() => {
      expect(screen.getByText('John Doe')).toBeInTheDocument();
      expect(screen.getByText('john@example.com')).toBeInTheDocument();
    });
  });

  test('navbar auth buttons have proper styling classes', () => {
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: false
    });

    render(<MockApp />);

    const loginButton = screen.getByText('Login');
    const signupButton = screen.getByText('Sign Up');

    // Check that buttons have expected classes for navbar styling
    expect(loginButton).toHaveClass('authButton');
    expect(signupButton).toHaveClass('authButton');
  });

  test('navbar user dropdown has proper styling classes', async () => {
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

    render(<MockApp />);

    // Click the avatar to open dropdown
    fireEvent.click(screen.getByText('J'));

    await waitFor(() => {
      const dropdown = document.querySelector('.navbarDropdown');
      expect(dropdown).toBeInTheDocument();

      const header = document.querySelector('.navbarDropdownHeader');
      expect(header).toBeInTheDocument();

      const logoutButton = screen.getByText('Sign Out');
      expect(logoutButton).toHaveClass('navbarDropdownItem');
    });
  });

  test('navbar avatar has proper styling classes', () => {
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

    render(<MockApp />);

    const avatar = screen.getByText('J');
    expect(avatar).toHaveClass('navbarUserAvatar');
  });

  test('navbar integration works with SSR considerations', () => {
    // This test verifies that the component handles SSR properly
    // by checking that it renders without errors initially
    (useSession as jest.Mock).mockReturnValue({
      data: null,
      isPending: true // Simulate loading state
    });

    const { container } = render(<MockApp />);

    // Should render some loading indicator or empty state
    expect(container).toBeInTheDocument();
  });
});

// Manual testing checklist for Phase 6:
// 1. Verify AuthButtons appear in the top-right corner of the navbar area
// 2. Verify buttons have appropriate styling that matches Docusaurus theme
// 3. Verify login/signup buttons show when not authenticated
// 4. Verify user avatar and dropdown show when authenticated
// 5. Verify dropdown menu has proper styling and positioning
// 6. Verify all auth functionality works from navbar location
// 7. Verify responsive design on mobile and tablet screens
// 8. Verify no conflicts with other navbar elements
// 9. Verify proper z-index so dropdown appears above other content
// 10. Verify components render correctly after page refresh (SSR compatibility)