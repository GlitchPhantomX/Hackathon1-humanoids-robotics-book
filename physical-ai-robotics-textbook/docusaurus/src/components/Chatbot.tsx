import React, { useState, useRef, useEffect, CSSProperties, KeyboardEvent } from "react";
import { Send, X, Bot, Loader2, MessageSquare, Sparkles } from "lucide-react";

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
  const chatEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [chatHistory, streamingContent]);

  // Focus input when chatbot opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  // Markdown to HTML converter (simple version)
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

    // Bullet points (multiple formats)
    html = html.replace(/^[•\-\*] (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: #374151;">$1</li>');

    // Numbered lists
    html = html.replace(/^(\d+)\. (.*$)/gim, '<li style="margin-left: 20px; margin-bottom: 6px; color: #374151; list-style-type: decimal;">$2</li>');

    // Wrap consecutive <li> in <ul>
    html = html.replace(/(<li[^>]*>.*?<\/li>(\s*<li[^>]*>.*?<\/li>)*)/gs, '<ul style="margin: 8px 0; padding-left: 0;">$1</ul>');

    // Tables (basic support)
    html = html.replace(/\|(.+)\|/g, (match) => {
      const cells = match.split('|').filter(cell => cell.trim());
      return '<tr>' + cells.map(cell => `<td style="padding: 8px; border: 1px solid #E5E7EB;">${cell.trim()}</td>`).join('') + '</tr>';
    });
    html = html.replace(/(<tr>.*<\/tr>)+/gs, '<table style="width: 100%; border-collapse: collapse; margin: 12px 0;">$&</table>');

    // Line breaks
    html = html.replace(/\n/g, '<br>');

    return html;
  }

  async function sendMessage() {
    if (!message.trim() || loading) return;

    const userMessage = message.trim();
    setMessage("");

    // Add user message to chat
    setChatHistory((prev) => [...prev, { role: "user", content: userMessage }]);

    setLoading(true);
    setStreamingContent("");

    try {
      const res = await fetch("http://127.0.0.1:8000/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ message: userMessage }),
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
                // Streaming complete
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
      setChatHistory((prev) => [
        ...prev,
        { 
          role: "assistant", 
          content: "❌ **Error:** Could not connect to server. Please ensure the backend is running on `http://127.0.0.1:8000`" 
        },
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
                <div style={styles.headerTitle}>AI Tutor</div>
                <div style={styles.headerSubtitle}>
                  {loading ? "Thinking..." : "Online"}
                </div>
              </div>
            </div>
            <button
              onClick={() => setIsOpen(false)}
              style={styles.closeButton}
              aria-label="Close chat"
            >
              <X size={24} color="white" />
            </button>
          </div>

          {/* Chat Messages */}
          <div style={styles.messagesContainer}>
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
                  }}
                >
                  {msg.content}
                </div>
              </div>
            ))}

            {/* Loading indicator */}
            {loading && (
              <div style={styles.messageWrapper}>
                <div style={{ ...styles.message, ...styles.assistantMessage }}>
                  <Loader2 size={16} style={styles.spinner} />
                  <span>Thinking...</span>
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
              placeholder="Ask about ROS2, Isaac Sim, URDF..."
              style={styles.input}
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
            Powered by AI • Humanoid Robotics Tutor
          </div>
        </div>
      )}

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

        /* Custom scrollbar */
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
// STYLES (with proper TypeScript types)
// ============================================

const styles: { [key: string]: CSSProperties } = {
  // Floating button
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


  // Chat container
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

  // Header
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

  // Messages
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
    lineHeight: "1.5",
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
    display: "flex",
    alignItems: "center",
    gap: "8px",
  },

  spinner: {
    animation: "spin 1s linear infinite",
  },

  // Input area
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

  // Footer
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