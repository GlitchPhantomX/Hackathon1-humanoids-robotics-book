// SignupModal Component Test Suite
// This file documents the expected behavior and test cases for the SignupModal component

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import SignupModal from './SignupModal';

// Mock the auth client
jest.mock('../../lib/auth-client', () => ({
  authClient: {
    signUp: {
      email: jest.fn()
    }
  }
}));

// Test suite for SignupModal component
describe('SignupModal', () => {
  const mockOnClose = jest.fn();
  const mockOnSignupSuccess = jest.fn();

  const defaultProps = {
    isOpen: true,
    onClose: mockOnClose,
    onSignupSuccess: mockOnSignupSuccess
  };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders signup modal with 2-step form structure', () => {
    render(<SignupModal {...defaultProps} />);

    // Check that the title is present
    expect(screen.getByText('Create Account')).toBeInTheDocument();

    // Check that the first step form elements are present
    expect(screen.getByLabelText('Full Name')).toBeInTheDocument();
    expect(screen.getByLabelText('Email')).toBeInTheDocument();
    expect(screen.getByLabelText('Password')).toBeInTheDocument();
    expect(screen.getByLabelText('Confirm Password')).toBeInTheDocument();
  });

  test('validates required fields in step 1', async () => {
    render(<SignupModal {...defaultProps} />);

    // Submit without filling any fields
    fireEvent.click(screen.getByText('Next'));

    // Wait for validation errors to appear
    await waitFor(() => {
      expect(screen.getByText('Name is required')).toBeInTheDocument();
      expect(screen.getByText('Email is required')).toBeInTheDocument();
      expect(screen.getByText('Password is required')).toBeInTheDocument();
    });
  });

  test('validates password match in step 1', async () => {
    render(<SignupModal {...defaultProps} />);

    // Fill in mismatched passwords
    fireEvent.change(screen.getByLabelText('Password'), { target: { value: 'password123' } });
    fireEvent.change(screen.getByLabelText('Confirm Password'), { target: { value: 'different123' } });

    fireEvent.click(screen.getByText('Next'));

    await waitFor(() => {
      expect(screen.getByText('Passwords do not match')).toBeInTheDocument();
    });
  });

  test('allows navigation to step 2 after valid step 1', async () => {
    render(<SignupModal {...defaultProps} />);

    // Fill in valid step 1 fields
    fireEvent.change(screen.getByLabelText('Full Name'), { target: { value: 'John Doe' } });
    fireEvent.change(screen.getByLabelText('Email'), { target: { value: 'john@example.com' } });
    fireEvent.change(screen.getByLabelText('Password'), { target: { value: 'password123' } });
    fireEvent.change(screen.getByLabelText('Confirm Password'), { target: { value: 'password123' } });

    fireEvent.click(screen.getByText('Next'));

    // Wait for navigation to step 2
    await waitFor(() => {
      expect(screen.getByLabelText('Software Background')).toBeInTheDocument();
      expect(screen.getByLabelText('Hardware Background')).toBeInTheDocument();
    });
  });

  test('validates required background fields in step 2', async () => {
    render(<SignupModal {...defaultProps} />);

    // Navigate to step 2 with valid step 1 data
    fireEvent.change(screen.getByLabelText('Full Name'), { target: { value: 'John Doe' } });
    fireEvent.change(screen.getByLabelText('Email'), { target: { value: 'john@example.com' } });
    fireEvent.change(screen.getByLabelText('Password'), { target: { value: 'password123' } });
    fireEvent.change(screen.getByLabelText('Confirm Password'), { target: { value: 'password123' } });

    fireEvent.click(screen.getByText('Next'));

    // Submit step 2 without filling required fields
    await waitFor(() => {
      fireEvent.click(screen.getByText('Create Account'));
    });

    await waitFor(() => {
      expect(screen.getByText('Software background is required')).toBeInTheDocument();
      expect(screen.getByText('Hardware background is required')).toBeInTheDocument();
      expect(screen.getByText('Robotics experience is required')).toBeInTheDocument();
      expect(screen.getByText('AI/ML experience is required')).toBeInTheDocument();
    });
  });

  test('allows signup with all required fields filled', async () => {
    const { authClient } = require('../../lib/auth-client');
    authClient.signUp.email.mockResolvedValue({ error: null });

    render(<SignupModal {...defaultProps} />);

    // Fill step 1
    fireEvent.change(screen.getByLabelText('Full Name'), { target: { value: 'John Doe' } });
    fireEvent.change(screen.getByLabelText('Email'), { target: { value: 'john@example.com' } });
    fireEvent.change(screen.getByLabelText('Password'), { target: { value: 'password123' } });
    fireEvent.change(screen.getByLabelText('Confirm Password'), { target: { value: 'password123' } });

    fireEvent.click(screen.getByText('Next'));

    // Fill step 2 with all background information
    await waitFor(() => {
      fireEvent.change(screen.getByLabelText('Software Background'), { target: { value: 'intermediate' } });
      fireEvent.change(screen.getByLabelText('Hardware Background'), { target: { value: 'basic' } });
      fireEvent.change(screen.getByLabelText('Robotics Experience'), { target: { value: 'hobbyist' } });
      fireEvent.change(screen.getByLabelText('AI/ML Experience'), { target: { value: 'intermediate' } });
      fireEvent.change(screen.getByLabelText('Programming Languages'), { target: { value: 'Python, JavaScript' } });
      fireEvent.change(screen.getByLabelText('Learning Goals'), { target: { value: 'Learn advanced robotics' } });

      // Check the checkboxes
      fireEvent.click(screen.getByLabelText('Do you have ROS experience?'));
      fireEvent.click(screen.getByLabelText('Do you have access to a GPU?'));
    });

    // Submit the form
    await waitFor(() => {
      fireEvent.click(screen.getByText('Create Account'));
    });

    // Wait for signup to complete
    await waitFor(() => {
      expect(authClient.signUp.email).toHaveBeenCalledWith({
        name: 'John Doe',
        email: 'john@example.com',
        password: 'password123'
      });
    });
  });
});

// Manual testing checklist:
// 1. Verify the modal opens and closes properly
// 2. Verify step 1 validation works (name, email, password, confirm password)
// 3. Verify navigation between steps works
// 4. Verify step 2 validation works (all required background fields)
// 5. Verify all 8 background fields can be filled and submitted
// 6. Verify loading states show during submission
// 7. Verify error messages display correctly
// 8. Verify successful signup redirects user appropriately
// 9. Verify backend properly stores all background information