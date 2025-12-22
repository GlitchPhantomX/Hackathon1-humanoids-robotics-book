import React, {
  useState,
  useRef,
  useEffect,
  useCallback,
  CSSProperties,
  KeyboardEvent,
} from "react";
import {
  Send,
  X,
  Bot,
  Loader2,
  MessageSquare,
  Languages,
  Trash2,
  Sparkles,
} from "lucide-react";

interface Message {
  role: "user" | "assistant";
  content: string;
  timestamp: Date;
}

const CHAT_HISTORY_KEY = 'bilingual_chatbot_history';

export default function ProfessionalChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [message, setMessage] = useState("");
  const [chatHistory, setChatHistory] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const [streamingContent, setStreamingContent] = useState("");
  const [language, setLanguage] = useState<"en" | "ur">("en");
  const [isTyping, setIsTyping] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
  const chatEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

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
        setChatHistory(historyWithDates);
      } else {
        // Initialize with welcome message
        const welcomeMessage: Message = {
          role: "assistant",
          content: "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖",
          timestamp: new Date(),
        };
        setChatHistory([welcomeMessage]);
      }
    } catch (error) {
      console.error('Error loading chat history:', error);
      const welcomeMessage: Message = {
        role: "assistant",
        content: "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖",
        timestamp: new Date(),
      };
      setChatHistory([welcomeMessage]);
    }
  }, []);

  // Save conversation history to localStorage whenever messages change
  useEffect(() => {
    if (chatHistory.length > 0) {
      try {
        localStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(chatHistory));
      } catch (error) {
        console.error('Error saving chat history:', error);
      }
    }
  }, [chatHistory]);

  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chatHistory, streamingContent]);

  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
      console.log("🔥 Chatbot opened");
    }
  }, [isOpen]);

  useEffect(() => {
    console.log("🌐 Current language:", language);
  }, [language]);

  function toggleLanguage() {
    console.log("🔄 Toggle clicked! Current lang:", language);
    const newLang = language === "en" ? "ur" : "en";
    console.log("✅ Switching to:", newLang);
    setLanguage(newLang);

    const welcomeMessage: Message =
      newLang === "ur"
        ? {
            role: "assistant",
            content: "السلام علیکم! میں Physical AI اور Humanoid Robotics کا AI ٹیوٹر ہوں۔ مجھ سے کچھ بھی پوچھیں! 🤖",
            timestamp: new Date(),
          }
        : {
            role: "assistant",
            content: "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖",
            timestamp: new Date(),
          };

    setChatHistory([welcomeMessage]);
  }

  function clearChat() {
    const welcomeMessage: Message =
      language === "ur"
        ? {
            role: "assistant",
            content: "السلام علیکم! میں Physical AI اور Humanoid Robotics کا AI ٹیوٹر ہوں۔ مجھ سے کچھ بھی پوچھیں! 🤖",
            timestamp: new Date(),
          }
        : {
            role: "assistant",
            content: "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖",
            timestamp: new Date(),
          };

    setChatHistory([welcomeMessage]);
    setStreamingContent("");
    setMessage("");
    localStorage.removeItem(CHAT_HISTORY_KEY);
  }

  function parseMarkdown(text: string): string {
    let html = text;

    html = html.replace(
      /^### (.*$)/gim,
      '<h3 style="color: #FF6B35; margin-top: 16px; margin-bottom: 8px; font-size: 16px; font-weight: 600;">$1</h3>'
    );
    html = html.replace(
      /^## (.*$)/gim,
      '<h2 style="color: #FF6B35; margin-top: 20px; margin-bottom: 12px; font-size: 18px; font-weight: 700;">$1</h2>'
    );
    html = html.replace(
      /^# (.*$)/gim,
      '<h1 style="color: #FF6B35; margin-top: 24px; margin-bottom: 16px; font-size: 20px; font-weight: 700;">$1</h1>'
    );

    html = html.replace(
      /\*\*(.*?)\*\*/g,
      '<strong style="color: var(--color-text); font-weight: 600;">$1</strong>'
    );

    html = html.replace(
      /```(\w+)?\n([\s\S]*?)```/g,
      '<pre style="background: var(--color-moving-demo-button-shadow-color); padding: 12px; border-radius: 8px; overflow-x: auto; margin: 12px 0; font-family: monospace; font-size: 13px; border-left: 3px solid #FF6B35;"><code>$2</code></pre>'
    );

    html = html.replace(
      /`([^`]+)`/g,
      '<code style="background: #FEF3F0; color: #FF6B35; padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 13px;">$1</code>'
    );

    html = html.replace(
      /^[•\-\*] (.*$)/gim,
      '<li style="margin-left: 20px; margin-bottom: 6px; color: var(--color-text);">$1</li>'
    );

    html = html.replace(
      /^(\d+)\. (.*$)/gim,
      '<li style="margin-left: 20px; margin-bottom: 6px; color: var(--color-text); list-style-type: decimal;">$2</li>'
    );

    html = html.replace(
      /(<li[^>]*>.*?<\/li>(\s*<li[^>]*>.*?<\/li>)*)/gs,
      '<ul style="margin: 8px 0; padding-left: 0;">$1</ul>'
    );

    html = html.replace(/\n/g, "<br>");

    return html;
  }

  async function sendMessage() {
    if (!message.trim() || loading) return;

    const userMessage = message.trim();
    setMessage("");

    setChatHistory((prev) => [...prev, { 
      role: "user", 
      content: userMessage,
      timestamp: new Date(),
    }]);

    setLoading(true);
    setIsTyping(true);
    setStreamingContent("");

    console.log("📤 Sending to backend:", { message: userMessage, language: language });

    try {
      const res = await fetch("http://127.0.0.1:8000/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          message: userMessage,
          language: language,
        }),
      });

      console.log("📥 Response status:", res.status);

      if (!res.ok) {
        throw new Error("Network error");
      }

      const reader = res.body?.getReader();
      const decoder = new TextDecoder();

      if (!reader) {
        throw new Error("No reader available");
      }

      let fullContent = "";

      while (true) {
        const { done, value } = await reader.read();

        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split("\n");

        for (const line of lines) {
          if (line.startsWith("data: ")) {
            try {
              const data = JSON.parse(line.slice(6));

              if (data.type === "content") {
                fullContent += data.data;
                setStreamingContent(fullContent);
              } else if (data.type === "done") {
                setChatHistory((prev) => [
                  ...prev,
                  { 
                    role: "assistant", 
                    content: fullContent,
                    timestamp: new Date(),
                  },
                ]);
                setStreamingContent("");
                setLoading(false);
                setIsTyping(false);
              }
            } catch (e) {
              // Skip invalid JSON
            }
          }
        }
      }
    } catch (error) {
      const errorMsg =
        language === "ur"
          ? "❌ **خرابی:** سرور سے رابطہ نہیں ہو سکا۔ براہ کرم یقینی بنائیں کہ backend `http://127.0.0.1:8000` پر چل رہا ہے"
          : "❌ **Error:** Could not connect to server. Please ensure the backend is running on `http://127.0.0.1:8000`";

      setChatHistory((prev) => [
        ...prev,
        { 
          role: "assistant", 
          content: errorMsg,
          timestamp: new Date(),
        },
      ]);
      setLoading(false);
      setIsTyping(false);
      setStreamingContent("");
    }
  }

  function handleKeyPress(e: KeyboardEvent<HTMLTextAreaElement>) {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  }

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
    const promptText = language === "ur" 
      ? `اس کی وضاحت کریں: "${selectedText}"`
      : `Explain this: "${selectedText}"`;
    
    setMessage(promptText);
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
          onClick={() => setIsOpen(true)}
          style={styles.floatingChatBtn}
          aria-label="Open chat"
        >
          <MessageSquare size={28} color="white" />
          <span style={styles.aiBadge}>AI</span>
          <div style={styles.pulseAnimation}></div>
        </button>
      )}

      {isOpen && (
        <div style={styles.chatbotContainer} data-chatbot-modal>
          <div style={styles.chatbotHeader}>
            <div style={styles.chatbotHeaderLeft}>
              <Bot size={24} color="white" />
              <div>
                <div style={styles.chatbotHeaderTitle}>
                  {language === 'ur' ? 'AI ٹیوٹر' : 'AI Tutor'}
                </div>
                <div style={styles.statusIndicator}>
                  <div style={styles.onlineDot}></div>
                  <span style={styles.statusText}>
                    {isTyping 
                      ? (language === 'ur' ? 'ٹائپ کر رہا ہے...' : 'Typing...') 
                      : (language === 'ur' ? 'آن لائن' : 'Online')}
                  </span>
                </div>
              </div>
            </div>

            <div style={{ display: 'flex', gap: '8px', alignItems: 'center'}}>
              <button
                onClick={clearChat}
                style={{
                  background: 'rgba(255, 255, 255, 0.25)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(255, 255, 255, 0.4)',
                  borderRadius: '8px',
                  padding: '8px',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  cursor: 'pointer',
                  transition: 'all 0.2s ease',
                }}
                title={language === 'ur' ? 'چیٹ صاف کریں' : 'Clear chat'}
              >
                <Trash2 size={16} color="white" />
              </button>

              <button
                onClick={() => {
                  console.log("🎯 Language toggle clicked!");
                  toggleLanguage();
                }}
                style={{
                  background: 'rgba(255, 255, 255, 0.25)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(255, 255, 255, 0.4)',
                  borderRadius: '8px',
                  padding: '6px 10px',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px',
                  color: 'white',
                  cursor: 'pointer',
                  fontSize: '11px',
                  fontWeight: '700',
                  transition: 'all 0.2s ease',
                }}
                title={language === 'ur' ? 'Switch to English' : 'اردو میں تبدیل کریں'}
              >
                <Languages size={16} />
                <span style={{ fontSize: '11px', fontWeight: '700', letterSpacing: '0.5px' }}>
                  {language.toUpperCase()}
                </span>
              </button>

              <button
                onClick={() => setIsOpen(false)}
                style={styles.chatbotCloseButton}
              >
                <X size={22} color="white" />
              </button>
            </div>
          </div>

          <div
            style={{
              ...styles.chatMessagesArea,
              direction: language === "ur" ? "rtl" : "ltr",
              fontFamily:
                language === "ur"
                  ? "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif"
                  : "inherit",
            }}
          >
            {chatHistory.length === 1 && !streamingContent && (
              <div
                style={{
                  textAlign: 'center',
                  padding: '10px 0',
                  color: 'var(--color-description-text)',
                  fontSize: '12px',
                  fontStyle: 'italic',
                }}
              >
                💡 {language === 'ur' 
                  ? 'نکتہ: کتاب سے کوئی بھی متن منتخب کریں اور ✨ آئیکن پر کلک کریں!'
                  : 'Tip: Select any text from the book and click the ✨ icon to ask about it!'}
              </div>
            )}

            {chatHistory.map((msg, index) => (
              <div
                key={index}
                style={{
                  ...styles.messageRow,
                  justifyContent:
                    msg.role === "user" ? "flex-end" : "flex-start",
                }}
              >
                <div
                  style={{
                    ...styles.messageBubble,
                    ...(msg.role === "user"
                      ? styles.userBubble
                      : styles.assistantBubble),
                    textAlign: language === "ur" ? "right" : "left",
                    fontFamily:
                      language === "ur" && msg.role === "assistant"
                        ? "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif"
                        : "inherit",
                  }}
                  dangerouslySetInnerHTML={{
                    __html: parseMarkdown(msg.content),
                  }}
                />
              </div>
            ))}

            {streamingContent && (
              <div style={styles.messageRow}>
                <div
                  style={{
                    ...styles.messageBubble,
                    ...styles.assistantBubble,
                    textAlign: language === "ur" ? "right" : "left",
                    fontFamily:
                      language === "ur"
                        ? "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif"
                        : "inherit",
                  }}
                  dangerouslySetInnerHTML={{
                    __html: parseMarkdown(streamingContent),
                  }}
                />
              </div>
            )}

            {loading && !streamingContent && (
              <div style={styles.messageRow}>
                <div style={{ 
                  ...styles.messageBubble, 
                  ...styles.assistantBubble,
                  fontFamily:
                    language === "ur"
                      ? "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif"
                      : "inherit",
                  display: 'flex',
                  alignItems: 'center',
                  gap: '8px',
                }}>
                  <Loader2 size={16} style={styles.spinnerIcon} />
                  <span>
                    {language === "ur" ? "سوچ رہا ہے..." : "Thinking..."}
                  </span>
                </div>
              </div>
            )}

            <div ref={chatEndRef} />
          </div>

          <div style={styles.chatInputArea}>
            <textarea
              ref={inputRef}
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={
                language === "ur"
                  ? "ROS2, Isaac Sim, URDF ke bare mein poochein..."
                  : "Ask about ROS2, Isaac Sim, URDF..."
              }
              style={{
                ...styles.messageInput,
                direction: "ltr",
                textAlign: "left",
                fontFamily: "inherit",
              }}
              rows={1}
              disabled={loading}
            />
            <button
              onClick={sendMessage}
              disabled={loading || !message.trim()}
              style={{
                ...styles.sendMessageBtn,
                ...(loading || !message.trim()
                  ? styles.sendBtnDisabled
                  : {}),
              }}
              aria-label="Send message"
            >
              <Send size={20} color="white" />
            </button>
          </div>

          <div style={styles.chatbotFooter}>
            {language === "ur"
              ? "AI سے چلایا گیا • Humanoid Robotics ٹیوٹر"
              : "Powered by AI • Humanoid Robotics Tutor"}
          </div>
        </div>
      )}

      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

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
          from { transform: rotate(0deg); }
          to { transform: rotate(360deg); }
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

        @keyframes blink {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.3; }
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

        div::-webkit-scrollbar {
          width: 6px;
        }

        div::-webkit-scrollbar-track {
          background: var(--color-moving-demo-button-shadow-color);
        }

        div::-webkit-scrollbar-thumb {
          background: #FF6B35;
          borderRadius: 3px;
        }

        div::-webkit-scrollbar-thumb:hover {
          background: #FF8E53;
        }

        button:hover {
          transform: scale(1.05);
          opacity: 0.9;
        }

        button:active {
          transform: scale(0.98);
        }
      `}</style>
    </>
  );
}

const styles: { [key: string]: CSSProperties } = {
  floatingChatBtn: {
    position: "fixed" as const,
    bottom: "24px",
    right: "24px",
    width: "64px",
    height: "64px",
    borderRadius: "50%",
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    border: "none",
    boxShadow: "0 8px 24px rgba(255, 107, 53, 0.4)",
    cursor: "pointer",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    transition: "all 0.3s ease",
    zIndex: 1000,
  },

  pulseAnimation: {
    position: "absolute" as const,
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    borderRadius: "50%",
    background: "#FF6B35",
    animation: "pulse 2s ease-in-out infinite",
    zIndex: -1,
  },

  aiBadge: {
    position: "absolute" as const,
    top: "-4px",
    right: "-4px",
    background: "linear-gradient(135deg, #00D9FF 0%, #00B8D4 100%)",
    color: "white",
    fontSize: "10px",
    fontWeight: "bold" as const,
    padding: "4px 8px",
    borderRadius: "12px",
    boxShadow: "0 2px 8px rgba(0, 217, 255, 0.4)",
  },

  chatbotContainer: {
    position: "fixed" as const,
    bottom: "24px",
    right: "24px",
    width: "420px",
    height: "520px",
    maxHeight: "calc(100vh - 48px)",
    background: "var(--color-bg)",
    borderRadius: "16px",
    boxShadow: "0 12px 48px rgba(0, 0, 0, 0.2)",
    display: "flex",
    flexDirection: "column" as const,
    zIndex: 1000,
    animation: "slideIn 0.3s ease",
  },

  chatbotHeader: {
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    padding: "18px 20px",
    borderRadius: "16px 16px 0 0",
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    color: "white",
    boxShadow: "0 2px 8px rgba(0, 0, 0, 0.1)",
    width: "100%",
    boxSizing: "border-box",
    position: "relative" as const,
  },

  chatbotHeaderLeft: {
    display: "flex",
    alignItems: "center",
    gap: "12px",
    maxWidth: "50%",
    overflow: "hidden",
  },

  chatbotHeaderTitle: {
    fontSize: "18px",
    fontWeight: "700" as const,
    letterSpacing: "-0.02em",
  },

  statusIndicator: {
    display: "flex",
    alignItems: "center",
    gap: "6px",
    marginTop: "4px",
  },

  onlineDot: {
    width: "8px",
    height: "8px",
    borderRadius: "50%",
    background: "#10B981",
    animation: "blink 2s ease-in-out infinite",
  },

  statusText: {
    fontSize: "11px",
    fontWeight: "500" as const,
    opacity: 0.9,
  },

  chatbotCloseButton: {
    background: "rgba(255, 255, 255, 0.25)",
    backdropFilter: "blur(10px)",
    border: "1px solid rgba(255, 255, 255, 0.3)",
    borderRadius: "10px",
    width: "36px",
    height: "36px",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    cursor: "pointer",
    transition: "all 0.2s ease",
    boxShadow: "0 2px 8px rgba(0, 0, 0, 0.1)",
  },

  chatMessagesArea: {
    flex: 1,
    overflowY: "auto" as const,
    padding: "20px",
    background: "var(--color-bg)",
    display: "flex",
    flexDirection: "column" as const,
    gap: "12px",
  },

  messageRow: {
    display: "flex",
    width: "100%",
  },

  messageBubble: {
    maxWidth: "80%",
    padding: "12px 16px",
    borderRadius: "12px",
    fontSize: "14px",
    lineHeight: "1.6",
    wordWrap: "break-word" as const,
    boxShadow: "0 1px 2px rgba(0, 0, 0, 0.05)",
  },

  userBubble: {
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    color: "white",
    borderBottomRightRadius: "4px",
    marginLeft: "auto",
  },

  assistantBubble: {
    background: "var(--color-bg)",
    color: "var(--color-text)",
    border: "1px solid var(--color-input-border)",
    borderBottomLeftRadius: "4px",
  },

  spinnerIcon: {
    animation: "spin 1s linear infinite",
  },

  chatInputArea: {
    padding: "16px 20px",
    background: "var(--color-bg)",
    borderTop: "1px solid var(--color-input-border)",
    display: "flex",
    gap: "12px",
    alignItems: "flex-end",
  },

  messageInput: {
    flex: 1,
    padding: "12px 16px",
    border: "1px solid var(--color-input-border)",
    borderRadius: "12px",
    fontSize: "14px",
    fontFamily: "inherit",
    resize: "none" as const,
    outline: "none",
    maxHeight: "120px",
    transition: "border-color 0.2s ease",
    background: "var(--color-bg)",
    color: "var(--color-text)",
  },

  sendMessageBtn: {
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    border: "none",
    borderRadius: "12px",
    width: "44px",
    height: "44px",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    cursor: "pointer",
    transition: "all 0.2s ease",
    flexShrink: 0,
    boxShadow: "0 2px 8px rgba(255, 107, 53, 0.3)",
  },

  sendBtnDisabled: {
    opacity: 0.5,
    cursor: "not-allowed",
  },

  chatbotFooter: {
    padding: "12px 20px",
    background: "var(--color-bg)",
    borderTop: "1px solid var(--color-input-border)",
    borderRadius: "0 0 16px 16px",
    fontSize: "11px",
    color: "var(--color-muted)",
    textAlign: "center" as const,
    fontWeight: "500" as const,
  },
};