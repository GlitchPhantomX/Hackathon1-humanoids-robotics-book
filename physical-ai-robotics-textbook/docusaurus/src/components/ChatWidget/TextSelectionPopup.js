import { useEffect, useState } from 'react';

const TextSelectionFeature = () => {
  const [isVisible, setIsVisible] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [popupRef, setPopupRef] = useState(null);

  useEffect(() => {
  const handleSelection = () => {
  const selection = window.getSelection();
  const text = selection.toString().trim();

  if (text.length === 0) {
    setIsVisible(false);
    return;
  }

  // Check if selection is within a content area (not UI elements)
  const anchorNode = selection.anchorNode;
  const focusNode = selection.focusNode;

  // Get parent element (text nodes don't have .closest, so we need parentElement)
  const anchorElement = anchorNode?.nodeType === Node.TEXT_NODE 
    ? anchorNode.parentElement 
    : anchorNode;
  const focusElement = focusNode?.nodeType === Node.TEXT_NODE 
    ? focusNode.parentElement 
    : focusNode;

  const contentElement = anchorElement?.closest('.markdown') || 
                        anchorElement?.closest('article') || 
                        focusElement?.closest('.markdown') || 
                        focusElement?.closest('article');

  // Only show for text selections in content areas and within length limits
  if (!contentElement || text.length < 5 || text.length > 2000) {
    setIsVisible(false);
    return;
  }

  // Get position for popup
  const range = selection.getRangeAt(0);
  const rect = range.getBoundingClientRect();
  
  setSelectedText(text);
  setPosition({ 
    x: rect.left + window.scrollX, 
    y: rect.top + window.scrollY - 40 
  });
  setIsVisible(true);
};

    const handleClickOutside = (e) => {
      if (popupRef && !popupRef.contains(e.target)) {
        // Check if the click is on the selected text
        const currentSelection = window.getSelection();
        if (currentSelection.rangeCount > 0) {
          const range = currentSelection.getRangeAt(0);
          const element = document.elementFromPoint(e.clientX, e.clientY);
          if (range.intersectsNode(element) || 
              (element && (range.startContainer === element || element.contains(range.startContainer))) ||
              (element && (range.endContainer === element || element.contains(range.endContainer)))) {
            return; // Click is inside the selection, don't hide
          }
        }
        setIsVisible(false);
      }
    };

    const handleMouseUp = () => {
      // Small delay to allow selection to register
      setTimeout(handleSelection, 10);
    };

    // Initialize the feature
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClickOutside);
    
    // Listen for scroll and resize events to hide popup
    window.addEventListener('scroll', () => setIsVisible(false));
    window.addEventListener('resize', () => setIsVisible(false));

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClickOutside);
      window.removeEventListener('scroll', () => setIsVisible(false));
      window.removeEventListener('resize', () => setIsVisible(false));
    };
  }, [popupRef]);

  const handleAskQuestion = () => {
    // Dispatch a custom event with the selected text
    const event = new CustomEvent('textSelectionEvent', {
      detail: { selectedText }
    });
    document.dispatchEvent(event);
    setIsVisible(false);
  };

  const handleExplainDetail = () => {
    // Dispatch a custom event with the selected text and request for detailed explanation
    const event = new CustomEvent('textSelectionEvent', {
      detail: { 
        selectedText,
        initialQuery: "Explain this in detail"
      }
    });
    document.dispatchEvent(event);
    setIsVisible(false);
  };

  if (!isVisible || !selectedText) return null;

  return (
    <div
      ref={setPopupRef}
      style={{
        position: 'absolute',
        left: `${position.x}px`,
        top: `${position.y}px`,
        zIndex: 10000,
        backgroundColor: '#4f46e5',
        color: 'white',
        padding: '8px 12px',
        borderRadius: '6px',
        boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        fontSize: '14px',
        display: 'flex',
        gap: '8px',
        maxWidth: '300px',
        transform: 'translateY(-100%)',
        fontFamily: '-apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif',
      }}
    >
      <button
        onClick={handleAskQuestion}
        style={{
          background: 'none',
          border: 'none',
          color: 'white',
          cursor: 'pointer',
          fontSize: 'inherit',
          padding: '2px 6px',
          borderRadius: '3px',
          fontWeight: '500',
        }}
        onMouseDown={(e) => e.preventDefault()} // Prevent blur on button click
      >
        💬 Ask about this
      </button>
      <button
        onClick={handleExplainDetail}
        style={{
          background: 'rgba(255, 255, 255, 0.2)',
          border: 'none',
          color: 'white',
          cursor: 'pointer',
          padding: '2px 6px',
          borderRadius: '3px',
          fontSize: 'inherit',
          fontWeight: '500',
        }}
        onMouseDown={(e) => e.preventDefault()} // Prevent blur on button click
      >
        Explain in detail
      </button>
      <button
        onClick={() => setIsVisible(false)}
        style={{
          background: 'none',
          border: 'none',
          color: 'white',
          cursor: 'pointer',
          fontSize: '18px',
          width: '20px',
          height: '20px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          marginLeft: '4px',
        }}
        onMouseDown={(e) => e.preventDefault()} // Prevent blur on button click
      >
        ×
      </button>
    </div>
  );
};

export default TextSelectionFeature;