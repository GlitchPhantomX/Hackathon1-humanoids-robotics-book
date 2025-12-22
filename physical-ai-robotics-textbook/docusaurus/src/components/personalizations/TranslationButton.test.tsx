/**
 * Unit tests for TranslationButton component
 */

import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import TranslationButton from './TranslationButton';

// Mock the CSS module
jest.mock('./rtlStyles.css', () => ({}));

describe('TranslationButton', () => {
  const defaultProps = {
    onToggle: jest.fn(),
    currentLanguage: 'en',
    isTranslating: false,
  };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders English to Urdu toggle button correctly', () => {
    render(<TranslationButton {...defaultProps} currentLanguage="en" />);

    const button = screen.getByRole('button');
    expect(button).toBeInTheDocument();
    expect(button).toHaveTextContent('EN → اردو');
    expect(button).not.toBeDisabled();
    expect(screen.queryByText('Translating...')).not.toBeInTheDocument();
  });

  test('renders Urdu to English toggle button correctly', () => {
    render(<TranslationButton {...defaultProps} currentLanguage="ur" />);

    const button = screen.getByRole('button');
    expect(button).toBeInTheDocument();
    expect(button).toHaveTextContent('اردو → EN');
  });

  test('calls onToggle when clicked', () => {
    const mockOnToggle = jest.fn();
    render(<TranslationButton {...defaultProps} onToggle={mockOnToggle} />);

    const button = screen.getByRole('button');
    fireEvent.click(button);

    expect(mockOnToggle).toHaveBeenCalledTimes(1);
  });

  test('disables button and shows loading state when isTranslating is true', () => {
    render(<TranslationButton {...defaultProps} isTranslating={true} />);

    const button = screen.getByRole('button');
    expect(button).toBeDisabled();
    expect(button).toHaveAttribute('aria-busy', 'true');
    expect(button).toHaveTextContent('Translating...');
  });

  test('shows loading spinner when translating', () => {
    render(<TranslationButton {...defaultProps} isTranslating={true} />);

    const spinner = screen.getByText('🔄');
    expect(spinner).toBeInTheDocument();
  });

  test('has correct aria-label when translating', () => {
    render(<TranslationButton {...defaultProps} isTranslating={true} />);

    const button = screen.getByRole('button');
    expect(button).toHaveAttribute('aria-label', 'Translating content, please wait');
  });

  test('has correct aria-label when in English mode', () => {
    render(<TranslationButton {...defaultProps} currentLanguage="en" isTranslating={false} />);

    const button = screen.getByRole('button');
    expect(button).toHaveAttribute('aria-label', 'Switch to Urdu');
  });

  test('has correct aria-label when in Urdu mode', () => {
    render(<TranslationButton {...defaultProps} currentLanguage="ur" isTranslating={false} />);

    const button = screen.getByRole('button');
    expect(button).toHaveAttribute('aria-label', 'Switch to English');
  });

  test('applies correct CSS classes based on state', () => {
    const { rerender } = render(<TranslationButton {...defaultProps} currentLanguage="en" />);

    const button = screen.getByRole('button');
    expect(button).toHaveClass('translation-toggle-btn');
    expect(button).toHaveClass('english-active');
    expect(button).not.toHaveClass('urdu-active');

    rerender(<TranslationButton {...defaultProps} currentLanguage="ur" />);
    expect(button).toHaveClass('urdu-active');
    expect(button).not.toHaveClass('english-active');

    rerender(<TranslationButton {...defaultProps} isTranslating={true} />);
    expect(button).toHaveClass('translating');
  });
});