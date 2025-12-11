import React from 'react';

const ReadingTime = ({ minutes }) => {
  return (
    <div className="flex items-center text-gray-400 text-sm mt-2 mb-3">
      <span className="mr-1">⌚</span>
      <span>{minutes} min read</span>
    </div>
  );
};

export default ReadingTime;
