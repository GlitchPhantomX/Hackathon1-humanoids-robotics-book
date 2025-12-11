import React, { useState } from 'react';
import styles from './ViewToggle.module.css';

const ViewToggle = () => {
  const [view, setView] = useState('full'); // 'full' or 'summary'

  return (
    <div className={styles.viewToggleContainer}>
      <div className={styles.buttonGroup}>
        <button
          className={`${styles.viewToggleBtn} ${view === 'full' ? styles.active : ''}`}
          onClick={() => setView('full')}
          aria-label="Show full lesson"
        >
          📖 Full Lesson
        </button>
        <button
          className={`${styles.viewToggleBtn} ${view === 'summary' ? styles.active : ''}`}
          onClick={() => setView('summary')}
          aria-label="Show summary"
        >
          ⚡ Summary
        </button>
      </div>
      <div className={styles.viewIndicator}>
        <span className={styles.viewLabel}>
          {view === 'full' ? '📚 Reading full content' : '⚡ Quick summary mode'}
        </span>
      </div>
    </div>
  );
};

export default ViewToggle;