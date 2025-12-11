import React from 'react';
import SourceCard from './SourceCard';
import './Message.css';

const Message = ({ message }) => {
  return (
    <div className={`message ${message.role}`}>
      <div className="message-content">
        {message.content}
        
        {message.role === 'assistant' && message.sources && message.sources.length > 0 && (
          <div className="sources-section">
            <strong>Sources:</strong>
            <div className="sources-grid">
              {message.sources.map((source, idx) => (
                <SourceCard 
                  key={`${message.id}-${idx}`} 
                  source={source} 
                />
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default Message;