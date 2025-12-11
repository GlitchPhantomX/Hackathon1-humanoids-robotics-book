import React, { useState } from 'react';
import './InputBox.css';

const InputBox = ({ onSendMessage, isLoading, conversationId }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;
    
    onSendMessage(inputValue, conversationId);
    setInputValue('');
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form className="input-box" onSubmit={handleSubmit}>
      <textarea
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Ask about the textbook..."
        disabled={isLoading}
        rows="1"
      />
      <button 
        type="submit" 
        disabled={!inputValue.trim() || isLoading}
        className="send-button"
      >
        Send
      </button>
    </form>
  );
};

export default InputBox;