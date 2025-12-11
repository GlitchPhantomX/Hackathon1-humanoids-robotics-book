import React from 'react';
import './SourceCard.css';

const SourceCard = ({ source }) => {
  const handleClick = () => {
    // Navigate to the relevant textbook section based on module and chapter
    const path = `/docs/${source.module.toLowerCase().replace(/\s+/g, '-')}/${source.chapter.toLowerCase().replace(/\s+/g, '-')}`;
    window.location.href = path;
  };

  return (
    <div className="source-card" onClick={handleClick}>
      <div className="source-header">
        <span className="source-module">{source.module}</span>
        <span className="source-chapter">{source.chapter}</span>
      </div>
      <div className="source-preview">
        {source.content.substring(0, 100)}{source.content.length > 100 ? '...' : ''}
      </div>
    </div>
  );
};

export default SourceCard;