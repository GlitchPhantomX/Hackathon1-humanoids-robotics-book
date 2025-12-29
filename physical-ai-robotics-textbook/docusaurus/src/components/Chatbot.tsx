import React, { useState, useRef, useEffect, CSSProperties, KeyboardEvent } from "react";
import { Send, X, Bot, Loader2, MessageSquare, Trash2, Languages } from "lucide-react";

interface Message {
  role: "user" | "assistant";
  content: string;
}

export default function ProfessionalChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [message, setMessage] = useState("");
  const [chatHistory, setChatHistory] = useState<Message[]>([
    {
      role: "assistant",
      content: "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖",
    },
  ]);
  const [loading, setLoading] = useState(false);
  const [streamingContent, setStreamingContent] = useState("");
  const [language, setLanguage] = useState<'en' | 'ur'>('en'); // ✅ Language state
  const chatEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chatHistory, streamingContent]);

  // Focus input when chatbot opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  // ✅ Toggle language
  function toggleLanguage() {
    const newLang = language === 'en' ? 'ur' : 'en';
    setLanguage(newLang);
    
    // Update welcome message based on language
    const welcomeMessage = newLang === 'ur'
      ? "السلام علیکم! میں Physical AI اور Humanoid Robotics کا AI ٹیوٹر ہوں۔ مجھ سے کچھ بھی پوچھیں! 🤖"
      : "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖";
    
    setChatHistory([
      {
        role: "assistant",
        content: welcomeMessage,
      },
    ]);
  }

  // ✅ Clear chat history
  function clearChat() {
    const welcomeMessage = language === 'ur'
      ? "السلام علیکم! میں Physical AI اور Humanoid Robotics کا AI ٹیوٹر ہوں۔ مجھ سے کچھ بھی پوچھیں! 🤖"
      : "Hi! I'm your AI tutor for Physical AI & Humanoid Robotics. Ask me anything! 🤖";
    
    setChatHistory([
      {
        role: "assistant",
        content: welcomeMessage,
      },
    ]);
  }

  // Markdown parser
  function parseMarkdown(text: string): string {
    let html = text;

    // Headers
    html = html.replace(/^### (.*$)/gim, '<h3 style="color: #FF6B35; margin-top: 16px; margin-bottom: 8px; font-size: 16px; font-weight: 600;">$1</h3>');
    html = html.replace(/^## (.*$)/gim, '<h2 style="color: #FF6B35; margin-top: 20px; margin-bottom: 12px; font-size: 18px; font-weight: 700;">$1</h2>');
    html = html.replace(/^# (.*$)/gim, '<h1 style="color: #FF6B35; margin-top: 24px; margin-bottom: 16px; font-size: 20px; font-weight: 700;">$1</h1>');

    // Bold
    html = html.replace(/\*\*(.*?)\*\*/g, '<strong style="color: #1F2937; font-weight: 600;">$1</strong>');

    // Code blocks
    html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre style="background: #F3F4F6; padding: 12px; border-radius: 8px; overflow-x: auto; margin: 12px 0; font-family: monospace; font-size: 13px; border-left: 3px solid #FF6B35;"><code>$2</code></pre>');

    // Inline code
    html = html.replace(/`([^`]+)`/g, '<code style="background: #FEF3F0; color: #FF6B35; padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 13px;">$1</code>');

    // Bullet points
    html = html.replace(/^[•\-\*] (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: #374151;">$1</li>');

    // Numbered lists
    html = html.replace(/^(\d+)\. (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: #374151; list-style-type: decimal;">$2</li>');

    // Wrap lists
    html = html.replace(/(<li[^>]*>.*?<\/li>(\s*<li[^>]*>.*?<\/li>)*)/gs, '<ul style="margin: 8px 0; padding-left: 0;">$1</ul>');

    // Line breaks
    html = html.replace(/\n/g, '<br>');

    return html;
  }

  async function sendMessage() {
    if (!message.trim() || loading) return;

    const userMessage = message.trim();
    setMessage("");

    // Add user message
    setChatHistory((prev) => [...prev, { role: "user", content: userMessage }]);

    setLoading(true);
    setStreamingContent("");

    try {
     const API_URL = process.env.NEXT_PUBLIC_API_URL;
const res = await fetch(`${API_URL}/chat`, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({ message: userMessage, language }),
});

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
                  { role: "assistant", content: fullContent },
                ]);
                setStreamingContent("");
                setLoading(false);
              }
            } catch (e) {
              // Skip invalid JSON
            }
          }
        }
      }
    } catch (error) {
      const errorMsg = language === 'ur'
        ? "❌ **خرابی:** سرور سے رابطہ نہیں ہو سکا۔ براہ کرم یقینی بنائیں کہ backend `http://127.0.0.1:8000` پر چل رہا ہے"
        : "❌ **Error:** Could not connect to server. Please ensure the backend is running on `http://127.0.0.1:8000`";
      
      setChatHistory((prev) => [
        ...prev,
        { role: "assistant", content: errorMsg },
      ]);
      setLoading(false);
      setStreamingContent("");
    }
  }

  function handleKeyPress(e: KeyboardEvent<HTMLTextAreaElement>) {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  }

  return (
    <>
      {/* Floating Chat Icon */}
      {!isOpen && (
        <button
          onClick={() => setIsOpen(true)}
          style={styles.floatingButton}
          aria-label="Open chat"
        >
          <MessageSquare size={28} color="white" />
          <span style={styles.badge}>AI</span>
          <div style={styles.pulse}></div>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div style={styles.chatContainer}>
          {/* Header */}
          <div style={styles.header}>
            <div style={styles.headerLeft}>
              <Bot size={24} color="white" />
              <div>
                <div style={styles.headerTitle}>
                  {language === 'ur' ? 'AI ٹیوٹر' : 'AI Tutor'}
                </div>
                <div style={styles.headerSubtitle}>
                  {loading 
                    ? (language === 'ur' ? 'سوچ رہا ہے...' : 'Thinking...') 
                    : (language === 'ur' ? 'آن لائن' : 'Online')
                  }
                </div>
              </div>
            </div>
            
            {/* ✅ Action Buttons */}
            <div style={styles.headerActions}>
              {/* Language Toggle */}
              <button
                onClick={toggleLanguage}
                style={styles.actionButton}
                aria-label="Toggle language"
                title={language === 'ur' ? 'Switch to English' : 'اردو میں تبدیل کریں'}
              >
                <Languages size={20} color="white" />
                <span style={styles.langLabel}>{language.toUpperCase()}</span>
              </button>
              
              {/* Clear Chat */}
              <button
                onClick={clearChat}
                style={styles.actionButton}
                aria-label="Clear chat"
                title={language === 'ur' ? 'چیٹ صاف کریں' : 'Clear chat'}
              >
                <Trash2 size={20} color="white" />
              </button>
              
              {/* Close */}
              <button
                onClick={() => setIsOpen(false)}
                style={styles.closeButton}
                aria-label="Close chat"
              >
                <X size={24} color="white" />
              </button>
            </div>
          </div>

          {/* Chat Messages */}
          <div 
            style={{
              ...styles.messagesContainer,
              direction: language === 'ur' ? 'rtl' : 'ltr',
              fontFamily: language === 'ur' 
                ? "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif" 
                : 'inherit'
            }}
          >
            {chatHistory.map((msg, index) => (
              <div
                key={index}
                style={{
                  ...styles.messageWrapper,
                  justifyContent: msg.role === "user" ? "flex-end" : "flex-start",
                }}
              >
                <div
                  style={{
                    ...styles.message,
                    ...(msg.role === "user" ? styles.userMessage : styles.assistantMessage),
                    textAlign: language === 'ur' ? 'right' : 'left',
                  }}
                  dangerouslySetInnerHTML={{ __html: parseMarkdown(msg.content) }}
                />
              </div>
            ))}

            {/* Streaming message */}
            {streamingContent && (
              <div style={styles.messageWrapper}>
                <div 
                  style={{ 
                    ...styles.message, 
                    ...styles.assistantMessage,
                    textAlign: language === 'ur' ? 'right' : 'left',
                  }}
                  dangerouslySetInnerHTML={{ __html: parseMarkdown(streamingContent) }}
                />
              </div>
            )}

            {/* Loading indicator */}
            {loading && !streamingContent && (
              <div style={styles.messageWrapper}>
                <div style={{ ...styles.message, ...styles.assistantMessage }}>
                  <Loader2 size={16} style={styles.spinner} />
                  <span>{language === 'ur' ? 'سوچ رہا ہے...' : 'Thinking...'}</span>
                </div>
              </div>
            )}

            <div ref={chatEndRef} />
          </div>

          {/* Input Area */}
          <div style={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={
                language === 'ur' 
                  ? 'ROS2، Isaac Sim، URDF کے بارے میں پوچھیں...'
                  : 'Ask about ROS2, Isaac Sim, URDF...'
              }
              style={{
                ...styles.input,
                direction: language === 'ur' ? 'rtl' : 'ltr',
                textAlign: language === 'ur' ? 'right' : 'left',
                fontFamily: language === 'ur' 
                  ? "'Noto Nastaliq Urdu', Arial, sans-serif" 
                  : 'inherit'
              }}
              rows={1}
              disabled={loading}
            />
            <button
              onClick={sendMessage}
              disabled={loading || !message.trim()}
              style={{
                ...styles.sendButton,
                ...(loading || !message.trim() ? styles.sendButtonDisabled : {}),
              }}
              aria-label="Send message"
            >
              <Send size={20} color="white" />
            </button>
          </div>

          {/* Footer */}
          <div style={styles.footer}>
            {language === 'ur' 
              ? 'AI سے چلایا گیا • Humanoid Robotics ٹیوٹر'
              : 'Powered by AI • Humanoid Robotics Tutor'
            }
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

        div::-webkit-scrollbar {
          width: 6px;
        }

        div::-webkit-scrollbar-track {
          background: #F3F4F6;
        }

        div::-webkit-scrollbar-thumb {
          background: #FF6B35;
          borderRadius: 3px;
        }

        div::-webkit-scrollbar-thumb:hover {
          background: #FF8E53;
        }
      `}</style>
    </>
  );
}

// ============================================
// STYLES
// ============================================

const styles: { [key: string]: CSSProperties } = {
  floatingButton: {
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

  pulse: {
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

  badge: {
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

  chatContainer: {
    position: "fixed" as const,
    bottom: "24px",
    right: "24px",
    width: "420px",
    height: "600px",
    maxHeight: "calc(100vh - 48px)",
    background: "white",
    borderRadius: "16px",
    boxShadow: "0 12px 48px rgba(0, 0, 0, 0.2)",
    display: "flex",
    flexDirection: "column" as const,
    zIndex: 1000,
    animation: "slideIn 0.3s ease",
  },

  header: {
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    padding: "20px",
    borderRadius: "16px 16px 0 0",
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    color: "white",
  },

  headerLeft: {
    display: "flex",
    alignItems: "center",
    gap: "12px",
  },

  headerTitle: {
    fontSize: "18px",
    fontWeight: "700" as const,
  },

  headerSubtitle: {
    fontSize: "12px",
    opacity: 0.9,
    marginTop: "2px",
  },

  // ✅ Header Actions Container
  headerActions: {
    display: "flex",
    gap: "8px",
    alignItems: "center",
  },

  // ✅ Action Buttons (Language, Clear)
  actionButton: {
    background: "rgba(255, 255, 255, 0.2)",
    border: "none",
    borderRadius: "8px",
    padding: "8px 12px",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    gap: "6px",
    cursor: "pointer",
    transition: "all 0.2s ease",
    fontSize: "11px",
    fontWeight: "600" as const,
    color: "white",
  },

  langLabel: {
    fontSize: "11px",
    fontWeight: "700" as const,
  },

  closeButton: {
    background: "rgba(255, 255, 255, 0.2)",
    border: "none",
    borderRadius: "8px",
    width: "36px",
    height: "36px",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    cursor: "pointer",
    transition: "all 0.2s ease",
  },

  messagesContainer: {
    flex: 1,
    overflowY: "auto" as const,
    padding: "20px",
    background: "#F9FAFB",
    display: "flex",
    flexDirection: "column" as const,
    gap: "12px",
  },

  messageWrapper: {
    display: "flex",
    width: "100%",
  },

  message: {
    maxWidth: "80%",
    padding: "12px 16px",
    borderRadius: "12px",
    fontSize: "14px",
    lineHeight: "1.6",
    wordWrap: "break-word" as const,
  },

  userMessage: {
    background: "linear-gradient(135deg, #FF6B35 0%, #FF8E53 100%)",
    color: "white",
    borderBottomRightRadius: "4px",
    marginLeft: "auto",
  },

  assistantMessage: {
    background: "white",
    color: "#1F2937",
    border: "1px solid #E5E7EB",
    borderBottomLeftRadius: "4px",
  },

  spinner: {
    animation: "spin 1s linear infinite",
  },

  inputContainer: {
    padding: "16px 20px",
    background: "white",
    borderTop: "1px solid #E5E7EB",
    display: "flex",
    gap: "12px",
    alignItems: "flex-end",
  },

  input: {
    flex: 1,
    padding: "12px 16px",
    border: "1px solid #E5E7EB",
    borderRadius: "12px",
    fontSize: "14px",
    fontFamily: "inherit",
    resize: "none" as const,
    outline: "none",
    maxHeight: "120px",
    transition: "border-color 0.2s ease",
  },

  sendButton: {
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
  },

  sendButtonDisabled: {
    opacity: 0.5,
    cursor: "not-allowed",
  },

  footer: {
    padding: "12px 20px",
    background: "#F9FAFB",
    borderTop: "1px solid #E5E7EB",
    borderRadius: "0 0 16px 16px",
    fontSize: "11px",
    color: "#6B7280",
    textAlign: "center" as const,
  },
};