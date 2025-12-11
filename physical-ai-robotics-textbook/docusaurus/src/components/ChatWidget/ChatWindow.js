import React, { useState, useEffect, useRef } from 'react';
import MessageList from './MessageList';
import InputBox from './InputBox';
import './ChatWindow.css';

const ChatWindow = ({ 
  messages, 
  onSendMessage, 
  isLoading, 
  conversationId, 
  onClose 
}) => {
  const messagesEndRef = useRef(null);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  return (
    <div className="chat-window">
      <div className="chat-header">
        <h3>Textbook Assistant</h3>
        <button 
          className="close-button" 
          onClick={onClose}
          aria-label="Close chat"
        >
          &times;
        </button>
      </div>
      
      <MessageList 
        messages={messages} 
        isLoading={isLoading} 
      />
      
      <InputBox 
        onSendMessage={onSendMessage}
        isLoading={isLoading}
        conversationId={conversationId}
      />
      
      <div ref={messagesEndRef} />
    </div>
  );
};

export default ChatWindow;