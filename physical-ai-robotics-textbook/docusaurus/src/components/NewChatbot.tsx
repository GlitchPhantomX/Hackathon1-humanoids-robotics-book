import React, { useState, useEffect, useRef, useCallback, CSSProperties } from 'react';
import { useSession } from '../lib/auth-client';
import LoginModal from './auth/LoginModal';
import SignupModal from './auth/SignupModal';
import { Send, X, Bot, Loader2, MessageSquare, Sparkles } from 'lucide-react';

interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

const CHAT_HISTORY_KEY = 'chatbot_conversation_history';

const NewChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState<'login' | 'signup' | null>(null);
  const [selectedText, setSelectedText] = useState<string>('');
  const [streamingContent, setStreamingContent] = useState('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const { data: session, isLoading: sessionLoading } = useSession();

  // Load conversation history from localStorage on mount
  useEffect(() => {
    try {
      const savedHistory = localStorage.getItem(CHAT_HISTORY_KEY);
      if (savedHistory) {
        const parsedHistory = JSON.parse(savedHistory);
        const historyWithDates = parsedHistory.map((msg: any) => ({
          ...msg,
          timestamp: new Date(msg.timestamp),
        }));
        setMessages(historyWithDates);
      }
    } catch (error) {
      console.error('Error loading chat history:', error);
    }
  }, []);

  // Save conversation history to localStorage whenever messages change
  useEffect(() => {
    if (messages.length > 0) {
      try {
        localStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
      } catch (error) {
        console.error('Error saving chat history:', error);
      }
    }
  }, [messages]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, streamingContent]);

  // Focus input when chatbot opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const checkAuth = useCallback(() => {
    if (!session?.user) {
      setShowAuthModal('login');
      return false;
    }
    return true;
  }, [session]);

  // Markdown to HTML converter with dark mode support
  function parseMarkdown(text: string): string {
    let html = text;

    // Headers
    html = html.replace(/^### (.*$)/gim, '<h3 style="color: var(--color-accent); margin-top: 16px; margin-bottom: 8px; font-size: 16px; font-weight: 600;">$1</h3>');
    html = html.replace(/^## (.*$)/gim, '<h2 style="color: var(--color-accent); margin-top: 20px; margin-bottom: 12px; font-size: 18px; font-weight: 700;">$1</h2>');
    html = html.replace(/^# (.*$)/gim, '<h1 style="color: var(--color-accent); margin-top: 24px; margin-bottom: 16px; font-size: 20px; font-weight: 700;">$1</h1>');

    // Bold
    html = html.replace(/\*\*(.*?)\*\*/g, '<strong style="color: var(--color-text); font-weight: 600;">$1</strong>');

    // Code blocks
    html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre style="background: var(--color-profile-card-background-color); padding: 12px; border-radius: 8px; overflow-x: auto; margin: 12px 0; font-family: monospace; font-size: 13px; border-left: 3px solid var(--color-accent); color: var(--color-text);"><code>$2</code></pre>');

    // Inline code
    html = html.replace(/`([^`]+)`/g, '<code style="background: var(--color-profile-language-bg-color); color: var(--color-accent); padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 13px;">$1</code>');

    // Bullet points (multiple formats)
    html = html.replace(/^[•\-\*] (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: var(--color-card-description-text);">$1</li>');

    // Numbered lists
    html = html.replace(/^(\d+)\. (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: var(--color-card-description-text); list-style-type: decimal;">$2</li>');

    // Wrap consecutive <li> in <ul>
    html = html.replace(/(<li[^>]*>.*?<\/li>(\s*<li[^>]*>.*?<\/li>)*)/gs, '<ul style="margin: 8px 0; padding-left: 0;">$1</ul>');

    // Tables (basic support)
    html = html.replace(/\|(.+)\|/g, (match) => {
      const cells = match.split('|').filter(cell => cell.trim());
      return '<tr>' + cells.map(cell => `<td style="padding: 8px; border: 1px solid var(--color-input-border); color: var(--color-text);">${cell.trim()}</td>`).join('') + '</tr>';
    });
    html = html.replace(/(<tr>.*<\/tr>)+/gs, '<table style="width: 100%; border-collapse: collapse; margin: 12px 0;">$&</table>');

    // Line breaks
    html = html.replace(/\n/g, '<br>');

    return html;
  }

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    if (!checkAuth()) {
      return;
    }

    const userMessage = inputValue.trim();
    setInputValue('');

    // Add user message to chat
    setMessages((prev) => [...prev, {
      id: Date.now().toString(),
      text: userMessage,
      sender: 'user',
      timestamp: new Date(),
    }]);

    setIsLoading(true);
    setStreamingContent('');

    try {
      const res = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: userMessage }),
      });

      if (!res.ok) {
        throw new Error('Network error');
      }

      const reader = res.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error('No reader available');
      }

      let fullContent = '';

      while (true) {
        const { done, value } = await reader.read();

        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = JSON.parse(line.slice(6));

              if (data.type === 'content') {
                fullContent += data.data;
                setStreamingContent(fullContent);
              } else if (data.type === 'done') {
                setMessages((prev) => [
                  ...prev,
                  {
                    id: `bot-${Date.now()}`,
                    text: fullContent,
                    sender: 'bot',
                    timestamp: new Date(),
                  },
                ]);
                setStreamingContent('');
                setIsLoading(false);
              }
            } catch (e) {
              // Skip invalid JSON
            }
          }
        }
      }
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        {
          id: `error-${Date.now()}`,
          text: '❌ **Error:** Could not connect to server. Please ensure the backend is running on `http://127.0.0.1:8000`',
          sender: 'bot',
          timestamp: new Date(),
        },
      ]);
      setIsLoading(false);
      setStreamingContent('');
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleLoginSuccess = () => {
    setShowAuthModal(null);
  };

  const handleSignupSuccess = () => {
    setShowAuthModal(null);
  };

  const handleSwitchToSignup = () => {
    setShowAuthModal('signup');
  };

  const handleCloseAuthModal = () => {
    setShowAuthModal(null);
  };

  const toggleChatbot = () => {
    if (!isOpen) {
      if (!checkAuth()) {
        return;
      }
      setIsOpen(true);
    } else {
      setIsOpen(false);
    }
  };

  const clearChatHistory = () => {
    setMessages([]);
    localStorage.removeItem(CHAT_HISTORY_KEY);
  };

  const handleTextSelection = useCallback((e: MouseEvent) => {
    const target = e.target as HTMLElement;
    if (target.closest('[data-chatbot-modal]') || target.closest('[data-selection-button]')) {
      return;
    }

    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';

    if (text.length > 0 && text.length <= 5000) {
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        setSelectedText(text);
        setSelectionPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 50,
        });
        setShowSelectionButton(true);
      }
    } else {
      setShowSelectionButton(false);
    }
  }, []);

  const handleSelectionButtonClick = () => {
    if (!checkAuth()) {
      return;
    }

    setInputValue(`Explain this: "${selectedText}"`);
    setIsOpen(true);
    setShowSelectionButton(false);

    setTimeout(() => {
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }, 100);
  };

  useEffect(() => {
    document.addEventListener('mouseup', handleTextSelection);
    
    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (!target.closest('[data-selection-button]') && !window.getSelection()?.toString()) {
        setShowSelectionButton(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleTextSelection]);

  return (
    <>
      <style>{`
        @keyframes slideIn {
          from {
            opacity: 0;
            transform: translateY(20px) scale(0.95);
          }
          to {
            opacity: 1;
            transform: translateY(0) scale(1);
          }
        }

        @keyframes spin {
          from {
            transform: rotate(0deg);
          }
          to {
            transform: rotate(360deg);
          }
        }

        @keyframes pulse {
          0%, 100% {
            transform: scale(1);
            opacity: 0.5;
          }
          50% {
            transform: scale(1.5);
            opacity: 0;
          }
        }

        @keyframes bounceIn {
          0% {
            opacity: 0;
            transform: translate(-50%, -10px) scale(0.8);
          }
          60% {
            opacity: 1;
            transform: translate(-50%, 0) scale(1.1);
          }
          100% {
            opacity: 1;
            transform: translate(-50%, 0) scale(1);
          }
        }

        [data-theme='light'] div::-webkit-scrollbar {
          width: 6px;
        }

        [data-theme='light'] div::-webkit-scrollbar-track {
          background: #F3F4F6;
        }

        [data-theme='light'] div::-webkit-scrollbar-thumb {
          background: #FF6B35;
          border-radius: 3px;
        }

        [data-theme='light'] div::-webkit-scrollbar-thumb:hover {
          background: #FF8E53;
        }

        [data-theme='dark'] div::-webkit-scrollbar {
          width: 6px;
        }

        [data-theme='dark'] div::-webkit-scrollbar-track {
          background: #262729;
        }

        [data-theme='dark'] div::-webkit-scrollbar-thumb {
          background: #ffa879;
          border-radius: 3px;
        }

        [data-theme='dark'] div::-webkit-scrollbar-thumb:hover {
          background: #ff8c42;
        }
      `}</style>

      {showSelectionButton && (
        <button
          data-selection-button
          onClick={handleSelectionButtonClick}
          style={{
            position: 'fixed',
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y}px`,
            transform: 'translateX(-50%)',
            background: 'linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)',
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '48px',
            height: '48px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(255, 107, 53, 0.5)',
            zIndex: 9999,
            animation: 'bounceIn 0.3s ease',
            transition: 'all 0.2s ease',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'translateX(-50%) scale(1.1)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'translateX(-50%) scale(1)';
          }}
          aria-label="Ask AI about selected text"
        >
          <Sparkles size={24} />
        </button>
      )}

      {!isOpen && (
        <button
          onClick={toggleChatbot}
          style={styles.floatingButton}
          aria-label="Open chat"
        >
          <MessageSquare size={28} color="white" />
          <span style={styles.badge}>AI</span>
          <div style={styles.pulse}></div>
        </button>
      )}

      {isOpen && (
        <div style={styles.chatContainer} data-chatbot-modal>
          <div style={styles.header}>
            <div style={styles.headerLeft}>
              <Bot size={24} color="white" />
              <div>
                <div style={styles.headerTitle}>AI Tutor</div>
                <div style={styles.headerSubtitle}>
                  {isLoading ? 'Thinking...' : 'Online'}
                </div>
              </div>
            </div>
            <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
              {messages.length > 0 && (
                <button
                  onClick={clearChatHistory}
                  style={{
                    background: 'rgba(255, 255, 255, 0.2)',
                    border: 'none',
                    borderRadius: '8px',
                    padding: '8px 12px',
                    color: 'white',
                    fontSize: '12px',
                    cursor: 'pointer',
                    transition: 'all 0.2s ease',
                  }}
                  onMouseEnter={(e) => {
                    e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                  }}
                  onMouseLeave={(e) => {
                    e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                  }}
                  aria-label="Clear chat history"
                >
                  Clear
                </button>
              )}
              <button
                onClick={() => setIsOpen(false)}
                style={styles.closeButton}
                aria-label="Close chat"
              >
                <X size={24} color="white" />
              </button>
            </div>
          </div>

          <div style={styles.messagesContainer}>
            {messages.length === 0 && !streamingContent && (
              <div
                style={{
                  textAlign: 'center',
                  padding: '20px',
                  color: 'var(--color-muted)',
                }}
              >
                <p>Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖</p>
                <p style={{ fontSize: '12px', marginTop: '10px', fontStyle: 'italic', color: 'var(--color-description-text)' }}>
                  💡 Tip: Select any text from the book and click the ✨ icon to ask about it!
                </p>
              </div>
            )}

            {messages.map((msg) => (
              <div
                key={msg.id}
                style={{
                  ...styles.messageWrapper,
                  justifyContent: msg.sender === 'user' ? 'flex-end' : 'flex-start',
                }}
              >
                <div
                  style={{
                    ...styles.message,
                    ...(msg.sender === 'user' ? styles.userMessage : styles.assistantMessage),
                  }}
                  dangerouslySetInnerHTML={{ __html: parseMarkdown(msg.text) }}
                />
              </div>
            ))}

            {streamingContent && (
              <div style={styles.messageWrapper}>
                <div
                  style={{ ...styles.message, ...styles.assistantMessage }}
                  dangerouslySetInnerHTML={{ __html: parseMarkdown(streamingContent) }}
                />
              </div>
            )}

            {isLoading && !streamingContent && (
              <div style={styles.messageWrapper}>
                <div style={{ 
                  ...styles.message, 
                  ...styles.assistantMessage,
                  display: 'flex',
                  alignItems: 'center',
                  gap: '8px'
                }}>
                  <Loader2 size={16} style={styles.spinner} />
                  <span>Thinking...</span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div style={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={session?.user ? 'Ask about ROS2, Isaac Sim, URDF...' : 'Please log in to chat'}
              style={styles.input}
              rows={1}
              disabled={!session?.user || sessionLoading || isLoading}
            />
            <button
              onClick={handleSendMessage}
              disabled={!session?.user || sessionLoading || isLoading || !inputValue.trim()}
              style={{
                ...styles.sendButton,
                ...(!session?.user || sessionLoading || isLoading || !inputValue.trim() ? styles.sendButtonDisabled : {}),
              }}
              aria-label="Send message"
            >
              <Send size={20} color="white" />
            </button>
          </div>

          <div style={styles.footer}>
            Powered by AI • Humanoid Robotics Tutor
          </div>
        </div>
      )}

      <LoginModal
        isOpen={showAuthModal === 'login'}
        onClose={handleCloseAuthModal}
        onLoginSuccess={handleLoginSuccess}
        onSwitchToSignup={handleSwitchToSignup}
      />

      <SignupModal
        isOpen={showAuthModal === 'signup'}
        onClose={handleCloseAuthModal}
        onSignupSuccess={handleSignupSuccess}
      />
    </>
  );
};

export default NewChatbot;

const styles: { [key: string]: CSSProperties } = {
  floatingButton: {
    position: 'fixed' as const,
    bottom: '24px',
    right: '24px',
    width: '64px',
    height: '64px',
    borderRadius: '50%',
    background: 'linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)',
    border: 'none',
    boxShadow: '0 8px 24px rgba(255, 107, 53, 0.4)',
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    transition: 'all 0.3s ease',
    zIndex: 10000,
  },

  pulse: {
    position: 'absolute' as const,
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    borderRadius: '50%',
    background: '#FF6B35',
    animation: 'pulse 2s ease-in-out infinite',
    zIndex: -1,
  },

  badge: {
    position: 'absolute' as const,
    top: '-4px',
    right: '-4px',
    background: 'linear-gradient(135deg, #00D9FF 0%, #00B8D4 100%)',
    color: 'white',
    fontSize: '10px',
    fontWeight: 'bold' as const,
    padding: '4px 8px',
    borderRadius: '12px',
    boxShadow: '0 2px 8px rgba(0, 217, 255, 0.4)',
  },

  chatContainer: {
    position: 'fixed' as const,
    bottom: '24px',
    right: '24px',
    width: '420px',
    height: '600px',
    maxHeight: 'calc(100vh - 48px)',
    background: 'var(--color-bg)',
    borderRadius: '16px',
    boxShadow: '0 12px 48px rgba(0, 0, 0, 0.2)',
    display: 'flex',
    flexDirection: 'column' as const,
    zIndex: 10000,
    animation: 'slideIn 0.3s ease',
  },

  header: {
    background: 'linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)',
    padding: '20px',
    borderRadius: '16px 16px 0 0',
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    color: 'white',
  },

  headerLeft: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
  },

  headerTitle: {
    fontSize: '18px',
    fontWeight: '700' as const,
  },

  headerSubtitle: {
    fontSize: '12px',
    opacity: 0.9,
    marginTop: '2px',
  },

  closeButton: {
    background: 'rgba(255, 255, 255, 0.2)',
    border: 'none',
    borderRadius: '8px',
    width: '36px',
    height: '36px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  },

  messagesContainer: {
    flex: 1,
    overflowY: 'auto' as const,
    padding: '20px',
    background: 'var(--color-bg)',
    display: 'flex',
    flexDirection: 'column' as const,
    gap: '12px',
  },

  messageWrapper: {
    display: 'flex',
    width: '100%',
  },

  message: {
    maxWidth: '80%',
    padding: '12px 16px',
    borderRadius: '12px',
    fontSize: '14px',
    lineHeight: '1.5',
    wordWrap: 'break-word' as const,
  },

  userMessage: {
    background: 'linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)',
    color: 'white',
    borderBottomRightRadius: '4px',
    marginLeft: 'auto',
  },

  assistantMessage: {
    background: 'var(--color-profile-card-background-color)',
    color: 'var(--color-text)',
    border: '1px solid var(--color-input-border)',
    borderBottomLeftRadius: '4px',
  },

  spinner: {
    animation: 'spin 1s linear infinite',
  },

  inputContainer: {
    padding: '16px 20px',
    background: 'var(--color-bg)',
    borderTop: '1px solid var(--color-input-border)',
    display: 'flex',
    gap: '12px',
    alignItems: 'flex-end',
  },

  input: {
    flex: 1,
    padding: '12px 16px',
    border: '1px solid var(--color-input-border)',
    borderRadius: '12px',
    fontSize: '14px',
    fontFamily: 'inherit',
    resize: 'none' as const,
    outline: 'none',
    maxHeight: '120px',
    transition: 'border-color 0.2s ease',
    background: 'var(--color-bg)',
    color: 'var(--color-text)',
  },

  sendButton: {
    background: 'linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)',
    border: 'none',
    borderRadius: '12px',
    width: '44px',
    height: '44px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    cursor: 'pointer',
    transition: 'all 0.2s ease',
    flexShrink: 0,
  },

  sendButtonDisabled: {
    opacity: 0.5,
    cursor: 'not-allowed',
  },

  footer: {
    padding: '12px 20px',
    background: 'var(--color-bg)',
    borderTop: '1px solid var(--color-input-border)',
    borderRadius: '0 0 16px 16px',
    fontSize: '11px',
    color: 'var(--color-muted)',
    textAlign: 'center' as const,
  },
};