import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from './index';

// Mock the TextSelectionFeature component
jest.mock('./TextSelectionPopup', () => () => <div data-testid="text-selection-popup" />);

describe('ChatWidget', () => {
  beforeEach(() => {
    // Clear sessionStorage before each test
    sessionStorage.clear();
  });

  test('renders floating chat button initially', () => {
    render(<ChatWidget />);
    
    const floatButton = screen.getByLabelText(/Open chat/i);
    expect(floatButton).toBeInTheDocument();
  });

  test('opens chat window when floating button is clicked', () => {
    render(<ChatWidget />);
    
    const floatButton = screen.getByLabelText(/Open chat/i);
    fireEvent.click(floatButton);
    
    expect(screen.getByText(/Textbook Assistant/i)).toBeInTheDocument();
    expect(screen.getByPlaceholderText(/Ask about the textbook\.\.\./i)).toBeInTheDocument();
  });

  test('closes chat window when close button is clicked', () => {
    render(<ChatWidget />);
    
    // Open the chat
    const floatButton = screen.getByLabelText(/Open chat/i);
    fireEvent.click(floatButton);
    
    // Close the chat
    const closeButton = screen.getByLabelText(/Close chat/i);
    fireEvent.click(closeButton);
    
    // The chat window should no longer be in the document
    expect(screen.queryByText(/Textbook Assistant/i)).not.toBeInTheDocument();
    // The floating button should be back
    expect(screen.getByLabelText(/Open chat/i)).toBeInTheDocument();
  });

  test('allows user to type and send a message', async () => {
    // Mock the fetch function
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({
          response: 'Test response',
          conversation_id: 'test-conversation-id',
          sources: []
        })
      })
    );

    render(<ChatWidget />);
    
    // Open the chat
    const floatButton = screen.getByLabelText(/Open chat/i);
    fireEvent.click(floatButton);
    
    // Type a message
    const inputArea = screen.getByPlaceholderText(/Ask about the textbook\.\.\./i);
    fireEvent.change(inputArea, { target: { value: 'Hello, world!' } });
    
    // Click the send button
    const sendButton = screen.getByText(/Send/i);
    fireEvent.click(sendButton);
    
    // Wait for the message to appear
    await waitFor(() => {
      expect(screen.getByText(/Hello, world!/i)).toBeInTheDocument();
    });
    
    // Check that fetch was called with the correct parameters
    expect(global.fetch).toHaveBeenCalledWith(
      '/api/chat',
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({
          message: 'Hello, world!',
          selected_text: '',
          conversation_id: expect.any(String)
        })
      })
    );
  });

  test('shows typing indicator when loading', async () => {
    // Create a promise that doesn't resolve immediately to simulate loading
    let resolvePromise;
    const fetchPromise = new Promise((resolve) => {
      resolvePromise = resolve;
    });
    
    global.fetch = jest.fn(() => fetchPromise);

    render(<ChatWidget />);
    
    // Open the chat
    const floatButton = screen.getByLabelText(/Open chat/i);
    fireEvent.click(floatButton);
    
    // Type a message
    const inputArea = screen.getByPlaceholderText(/Ask about the textbook\.\.\./i);
    fireEvent.change(inputArea, { target: { value: 'Test message' } });
    
    // Click the send button
    const sendButton = screen.getByText(/Send/i);
    fireEvent.click(sendButton);
    
    // The typing indicator should appear while loading
    expect(screen.getByText(/Ask about the textbook\.\.\./i)).toBeInTheDocument();
    
    // Resolve the promise to complete the test
    resolvePromise({
      ok: true,
      json: () => Promise.resolve({
        response: 'Test response',
        conversation_id: 'test-conversation-id',
        sources: []
      })
    });
  });

  test('stores conversation ID in sessionStorage', () => {
    render(<ChatWidget />);
    
    // Get the conversation ID from session storage
    const conversationId = sessionStorage.getItem('chat-conversation-id');
    
    // Verify that a conversation ID was set
    expect(conversationId).toBeTruthy();
    expect(typeof conversationId).toBe('string');
    expect(conversationId.startsWith('conv_')).toBe(true);
  });
});