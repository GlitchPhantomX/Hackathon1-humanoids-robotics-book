import React, { useState, useEffect, useRef } from 'react';
import TextSelectionFeature from './TextSelectionPopup';
import './ChatWidget.css';

// API URL configuration for Docusaurus
const API_URL = 'http://localhost:8000';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Initialize conversation ID from sessionStorage
  // Initialize conversation ID - backend will create it on first message
useEffect(() => {
  // Check if we have a conversation ID from previous session
  const savedConversationId = sessionStorage.getItem('chat-conversation-id');
  if (savedConversationId) {
    setConversationId(savedConversationId);
  }
  // If no saved ID, we'll let backend create one on first message (conversationId stays null)
}, []);

  // Handle text selection events
  useEffect(() => {
    const handleTextSelection = (e) => {
      const { detail } = e;
      if (detail.selectedText) {
        setSelectedText(detail.selectedText);
        if (detail.initialQuery) {
          setInputValue(`${detail.initialQuery}: "${detail.selectedText.substring(0, 100)}${detail.selectedText.length > 100 ? '...' : ''}"`);
        } else {
          setInputValue(`"${detail.selectedText.substring(0, 100)}${detail.selectedText.length > 100 ? '...' : ''}"`);
        }
        // Open the chat if it's closed
        if (!isOpen) {
          setIsOpen(true);
        }
      }
    };

    document.addEventListener('textSelectionEvent', handleTextSelection);

    return () => {
      document.removeEventListener('textSelectionEvent', handleTextSelection);
    };
  }, [isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          selected_text: selectedText,
          conversation_id: conversationId,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message to chat
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, assistantMessage]);

     if (data.conversation_id && data.conversation_id !== conversationId) {
  setConversationId(data.conversation_id);
  sessionStorage.setItem('chat-conversation-id', data.conversation_id);
}
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: "Sorry, I encountered an error. Please try again later.",
        sources: [],
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText('');
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Text selection popup */}
      <TextSelectionFeature />

      {/* Floating chat button */}
      {!isOpen && (
        <button
          className="chat-float-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Textbook Assistant</h3>
            <button
              className="chat-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              &times;
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
              >
                <div className="message-content">
                  {message.content}

                  {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
                    <div className="sources-section">
                      <strong>Sources:</strong>
                      <div className="sources-grid">
                        {message.sources.map((source, idx) => (
                          <div
                            key={idx}
                            className="source-card"
                            onClick={() => {
                              // Navigate to the relevant textbook section based on module and chapter
                              const path = `/docs/${source.module.toLowerCase().replace(/\s+/g, '-')}/${source.chapter.toLowerCase().replace(/\s+/g, '-')}`;
                              window.location.href = path;
                            }}
                          >
                            <div className="source-header">
                              <span className="source-module">{source.module}</span>
                              <span className="source-chapter">{source.chapter}</span>
                            </div>
                            <div className="source-preview">
                              {source.content.substring(0, 100)}{source.content.length > 100 ? '...' : ''}
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={selectedText ? `Ask about selected text: "${selectedText.substring(0, 20)}..."` : "Ask about the textbook..."}
              disabled={isLoading}
              rows="1"
            />
            <button
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="send-button"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;