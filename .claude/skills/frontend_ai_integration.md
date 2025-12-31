# Frontend AI Integration Skill

---
name: frontend_ai_integration
description: Standardize frontend UI interactions with backend AI endpoints for chatbot and chapter help
version: 1.0.0
project: physical-ai-robotics-textbook
parameters:
  - name: framework
    description: Frontend framework (react, vue, svelte)
    required: false
    default: react
  - name: state_management
    description: State management approach (hooks, redux, zustand)
    required: false
    default: hooks
  - name: features
    description: Comma-separated features (chatbot, chapter-help, explain)
    required: false
    default: chatbot,chapter-help
---

## Purpose

Provide standardized patterns for frontend AI integration that:
- **Define interaction flows** with FastAPI backend endpoints
- **Support chatbot and chapter help UI** components
- **Manage AI response states** (loading, streaming, error)
- **Ensure type safety** with TypeScript interfaces
- **Enable reusability** across different UI components
- **Handle edge cases** (timeouts, retries, errors)

## Core Principles

**MUST FOLLOW:**
1. **Frontend-Only**: Zero backend logic, pure UI/API client concerns
2. **Type Safety**: TypeScript interfaces for all API interactions
3. **Reusability**: Hooks and utilities work across components
4. **Error Handling**: Graceful degradation with user feedback
5. **Loading States**: Visual feedback for async operations
6. **Accessibility**: ARIA labels, keyboard navigation, screen reader support
7. **Separation of Concerns**: API client → Hooks → Components

## Architecture Overview

```
Frontend AI Integration Architecture:

┌─────────────────────────────────────────────────────────┐
│                   UI Components Layer                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Chatbot    │  │ Chapter Help │  │   Explain    │  │
│  │   Widget     │  │    Button    │  │   Dialog     │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                   Custom Hooks Layer                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ useChatbot   │  │useChapterHelp│  │  useExplain  │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│                    API Client Layer                      │
│         ┌────────────────────────────────┐              │
│         │      AIServiceClient           │              │
│         │  - query()                     │              │
│         │  - getChapterHelp()            │              │
│         │  - explainConcept()            │              │
│         └────────────────────────────────┘              │
└─────────────────────────────────────────────────────────┘
                          ↓
                   FastAPI Backend
              (rag_context_control +
              robotics_reasoning_core)
```

## Instructions

### Step 1: Define TypeScript Interfaces

Create type-safe interfaces for all API interactions:

```typescript
// src/types/ai-service.types.ts

/**
 * Common types for AI service interactions
 */

// ============================================================================
// Request Types
// ============================================================================

export interface ChatbotQueryRequest {
  question: string;
  conversationId?: string;
  context?: Record<string, any>;
}

export interface ChapterHelpRequest {
  chapterId: string;
  question: string;
  section?: string;
  codeContext?: string;
}

export interface ExplainRequest {
  topic: string;
  depth: 'beginner' | 'intermediate' | 'advanced';
  includeCode?: boolean;
  includeMath?: boolean;
}

// ============================================================================
// Response Types
// ============================================================================

export interface Source {
  chapter: string;
  section: string;
  contentSnippet: string;
  relevanceScore: number;
}

export interface ChatbotResponse {
  answer: string;
  sources: Source[];
  conversationId: string;
  timestamp: string;
  grounded: boolean;
  confidence: 'none' | 'low' | 'medium' | 'high';
}

export interface ChapterHelpResponse {
  answer: string;
  relatedConcepts: string[];
  sources: Source[];
  suggestedExercises?: string[];
}

export interface RoboticsExplanation {
  conceptId: string;
  conceptName: string;
  domain: string;
  explanation: string;
  depthLevel: string;
  prerequisites: Array<{
    id: string;
    name: string;
    description: string;
  }>;
  relatedConcepts: string[];
  mathematicalFormulations?: Array<{
    name: string;
    latex: string;
    description: string;
  }>;
  codeExamples?: Array<{
    language: string;
    title: string;
    code: string;
  }>;
  practicalApplications: string[];
  commonPitfalls: string[];
  references: string[];
}

// ============================================================================
// State Types
// ============================================================================

export interface AIRequestState<T> {
  data: T | null;
  loading: boolean;
  error: Error | null;
}

export interface StreamingState {
  isStreaming: boolean;
  partialResponse: string;
  complete: boolean;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface AIServiceConfig {
  baseUrl: string;
  timeout?: number;
  retries?: number;
  headers?: Record<string, string>;
}
```

### Step 2: Implement API Client

Create a type-safe API client for backend communication:

```typescript
// src/services/ai-service.client.ts

import type {
  ChatbotQueryRequest,
  ChatbotResponse,
  ChapterHelpRequest,
  ChapterHelpResponse,
  ExplainRequest,
  RoboticsExplanation,
  AIServiceConfig,
} from '@/types/ai-service.types';

/**
 * API Client for AI services
 * Handles all communication with FastAPI backend
 */
export class AIServiceClient {
  private baseUrl: string;
  private timeout: number;
  private retries: number;
  private headers: Record<string, string>;

  constructor(config: AIServiceConfig) {
    this.baseUrl = config.baseUrl;
    this.timeout = config.timeout || 30000; // 30s default
    this.retries = config.retries || 2;
    this.headers = config.headers || {
      'Content-Type': 'application/json',
    };
  }

  /**
   * Generic fetch wrapper with timeout and retry logic
   */
  private async fetchWithRetry<T>(
    endpoint: string,
    options: RequestInit,
    attempt = 0
  ): Promise<T> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        ...options,
        headers: {
          ...this.headers,
          ...options.headers,
        },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          errorData.detail || `API Error: ${response.status} ${response.statusText}`
        );
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutId);

      // Retry on network errors
      if (attempt < this.retries && this.shouldRetry(error)) {
        await this.delay(Math.pow(2, attempt) * 1000); // Exponential backoff
        return this.fetchWithRetry<T>(endpoint, options, attempt + 1);
      }

      throw error;
    }
  }

  private shouldRetry(error: any): boolean {
    // Retry on network errors, not on 4xx errors
    return (
      error.name === 'AbortError' ||
      error.message.includes('fetch') ||
      error.message.includes('network')
    );
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ============================================================================
  // Chatbot API
  // ============================================================================

  /**
   * Send a query to the chatbot
   */
  async queryChatbot(request: ChatbotQueryRequest): Promise<ChatbotResponse> {
    return this.fetchWithRetry<ChatbotResponse>('/api/chatbot/query', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  // ============================================================================
  // Chapter Help API
  // ============================================================================

  /**
   * Request chapter-specific help
   */
  async getChapterHelp(request: ChapterHelpRequest): Promise<ChapterHelpResponse> {
    return this.fetchWithRetry<ChapterHelpResponse>('/api/chapter/help', {
      method: 'POST',
      body: JSON.stringify(request),
    });
  }

  // ============================================================================
  // Robotics Explanation API
  // ============================================================================

  /**
   * Get detailed explanation of a robotics concept
   */
  async explainConcept(request: ExplainRequest): Promise<RoboticsExplanation> {
    const params = new URLSearchParams({
      topic: request.topic,
      depth: request.depth,
      include_code: String(request.includeCode ?? true),
      include_math: String(request.includeMath ?? true),
    });

    return this.fetchWithRetry<RoboticsExplanation>(
      `/api/explain/concept?${params}`,
      {
        method: 'GET',
      }
    );
  }

  /**
   * Stream chatbot responses (Server-Sent Events)
   */
  async *streamChatbot(request: ChatbotQueryRequest): AsyncGenerator<string> {
    const response = await fetch(`${this.baseUrl}/api/chatbot/stream`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Stream error: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    const decoder = new TextDecoder();

    if (!reader) {
      throw new Error('Stream not supported');
    }

    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value);
        const lines = chunk.split('\n');

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = line.slice(6);
            if (data === '[DONE]') return;
            yield data;
          }
        }
      }
    } finally {
      reader.releaseLock();
    }
  }
}

// ============================================================================
// Singleton Instance
// ============================================================================

// Create singleton instance
let aiServiceClient: AIServiceClient | null = null;

export function getAIServiceClient(config?: AIServiceConfig): AIServiceClient {
  if (!aiServiceClient) {
    aiServiceClient = new AIServiceClient(
      config || {
        baseUrl: import.meta.env.VITE_AI_SERVICE_URL || 'http://localhost:8000',
      }
    );
  }
  return aiServiceClient;
}
```

### Step 3: Create Custom React Hooks

Implement reusable hooks for AI features:

```typescript
// src/hooks/useChatbot.ts

import { useState, useCallback, useRef } from 'react';
import { getAIServiceClient } from '@/services/ai-service.client';
import type {
  ChatbotQueryRequest,
  ChatbotResponse,
  AIRequestState,
} from '@/types/ai-service.types';

/**
 * Hook for chatbot interactions
 */
export function useChatbot() {
  const [state, setState] = useState<AIRequestState<ChatbotResponse>>({
    data: null,
    loading: false,
    error: null,
  });

  const [conversationHistory, setConversationHistory] = useState<
    Array<{ role: 'user' | 'assistant'; content: string; timestamp: Date }>
  >([]);

  const conversationIdRef = useRef<string | null>(null);
  const abortControllerRef = useRef<AbortController | null>(null);

  /**
   * Send a query to the chatbot
   */
  const query = useCallback(
    async (question: string, context?: Record<string, any>) => {
      // Cancel previous request if still pending
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }

      abortControllerRef.current = new AbortController();

      setState({ data: null, loading: true, error: null });

      // Add user message to history
      setConversationHistory((prev) => [
        ...prev,
        { role: 'user', content: question, timestamp: new Date() },
      ]);

      try {
        const client = getAIServiceClient();
        const request: ChatbotQueryRequest = {
          question,
          conversationId: conversationIdRef.current || undefined,
          context,
        };

        const response = await client.queryChatbot(request);

        // Store conversation ID for continuity
        conversationIdRef.current = response.conversationId;

        // Add assistant response to history
        setConversationHistory((prev) => [
          ...prev,
          { role: 'assistant', content: response.answer, timestamp: new Date() },
        ]);

        setState({ data: response, loading: false, error: null });

        return response;
      } catch (error) {
        const err = error instanceof Error ? error : new Error('Unknown error');
        setState({ data: null, loading: false, error: err });
        throw err;
      }
    },
    []
  );

  /**
   * Clear conversation history
   */
  const clearHistory = useCallback(() => {
    setConversationHistory([]);
    conversationIdRef.current = null;
    setState({ data: null, loading: false, error: null });
  }, []);

  /**
   * Retry last failed request
   */
  const retry = useCallback(() => {
    const lastUserMessage = conversationHistory
      .filter((msg) => msg.role === 'user')
      .pop();

    if (lastUserMessage) {
      return query(lastUserMessage.content);
    }
  }, [conversationHistory, query]);

  return {
    ...state,
    query,
    clearHistory,
    retry,
    conversationHistory,
    hasConversation: conversationHistory.length > 0,
  };
}

// ============================================================================
// src/hooks/useChapterHelp.ts
// ============================================================================

import { useState, useCallback } from 'react';
import { getAIServiceClient } from '@/services/ai-service.client';
import type {
  ChapterHelpRequest,
  ChapterHelpResponse,
  AIRequestState,
} from '@/types/ai-service.types';

/**
 * Hook for chapter-specific help
 */
export function useChapterHelp(chapterId: string) {
  const [state, setState] = useState<AIRequestState<ChapterHelpResponse>>({
    data: null,
    loading: false,
    error: null,
  });

  const getHelp = useCallback(
    async (question: string, section?: string, codeContext?: string) => {
      setState({ data: null, loading: true, error: null });

      try {
        const client = getAIServiceClient();
        const request: ChapterHelpRequest = {
          chapterId,
          question,
          section,
          codeContext,
        };

        const response = await client.getChapterHelp(request);
        setState({ data: response, loading: false, error: null });

        return response;
      } catch (error) {
        const err = error instanceof Error ? error : new Error('Unknown error');
        setState({ data: null, loading: false, error: err });
        throw err;
      }
    },
    [chapterId]
  );

  const reset = useCallback(() => {
    setState({ data: null, loading: false, error: null });
  }, []);

  return {
    ...state,
    getHelp,
    reset,
  };
}

// ============================================================================
// src/hooks/useRoboticsExplain.ts
// ============================================================================

import { useState, useCallback } from 'react';
import { getAIServiceClient } from '@/services/ai-service.client';
import type {
  ExplainRequest,
  RoboticsExplanation,
  AIRequestState,
} from '@/types/ai-service.types';

/**
 * Hook for robotics concept explanations
 */
export function useRoboticsExplain() {
  const [state, setState] = useState<AIRequestState<RoboticsExplanation>>({
    data: null,
    loading: false,
    error: null,
  });

  const explain = useCallback(
    async (
      topic: string,
      depth: 'beginner' | 'intermediate' | 'advanced' = 'intermediate',
      options?: { includeCode?: boolean; includeMath?: boolean }
    ) => {
      setState({ data: null, loading: true, error: null });

      try {
        const client = getAIServiceClient();
        const request: ExplainRequest = {
          topic,
          depth,
          includeCode: options?.includeCode,
          includeMath: options?.includeMath,
        };

        const response = await client.explainConcept(request);
        setState({ data: response, loading: false, error: null });

        return response;
      } catch (error) {
        const err = error instanceof Error ? error : new Error('Unknown error');
        setState({ data: null, loading: false, error: err });
        throw err;
      }
    },
    []
  );

  const reset = useCallback(() => {
    setState({ data: null, loading: false, error: null });
  }, []);

  return {
    ...state,
    explain,
    reset,
  };
}
```

### Step 4: Create UI Components

Implement reusable UI components:

```typescript
// src/components/ChatWidget/ChatWidget.tsx

import React, { useState, useRef, useEffect } from 'react';
import { useChatbot } from '@/hooks/useChatbot';
import { MessageList } from './MessageList';
import { ChatInput } from './ChatInput';
import { ErrorMessage } from './ErrorMessage';
import { LoadingIndicator } from './LoadingIndicator';
import './ChatWidget.css';

interface ChatWidgetProps {
  /** Initial context for the chatbot */
  context?: Record<string, any>;
  /** Placeholder text for input */
  placeholder?: string;
  /** Maximum height of chat window */
  maxHeight?: string;
  /** Show/hide sources in responses */
  showSources?: boolean;
}

export function ChatWidget({
  context,
  placeholder = 'Ask a question about robotics...',
  maxHeight = '500px',
  showSources = true,
}: ChatWidgetProps) {
  const { query, loading, error, conversationHistory, clearHistory, retry } =
    useChatbot();

  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom on new messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [conversationHistory]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!input.trim() || loading) return;

    const question = input.trim();
    setInput('');

    try {
      await query(question, context);
    } catch (error) {
      // Error is handled by the hook
      console.error('Chat error:', error);
    }
  };

  return (
    <div className="chat-widget" role="region" aria-label="AI Chatbot">
      {/* Header */}
      <div className="chat-widget__header">
        <h3>AI Assistant</h3>
        {conversationHistory.length > 0 && (
          <button
            onClick={clearHistory}
            className="chat-widget__clear"
            aria-label="Clear conversation"
          >
            Clear
          </button>
        )}
      </div>

      {/* Messages */}
      <div
        className="chat-widget__messages"
        style={{ maxHeight }}
        role="log"
        aria-live="polite"
        aria-atomic="false"
      >
        {conversationHistory.length === 0 ? (
          <div className="chat-widget__empty">
            <p>👋 Hi! Ask me anything about robotics and Physical AI.</p>
          </div>
        ) : (
          <MessageList messages={conversationHistory} showSources={showSources} />
        )}

        {loading && <LoadingIndicator />}

        {error && (
          <ErrorMessage
            error={error}
            onRetry={retry}
            onDismiss={() => {}}
          />
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <form onSubmit={handleSubmit} className="chat-widget__form">
        <ChatInput
          value={input}
          onChange={setInput}
          placeholder={placeholder}
          disabled={loading}
          onSubmit={handleSubmit}
        />
      </form>
    </div>
  );
}

// ============================================================================
// src/components/ChapterHelpButton/ChapterHelpButton.tsx
// ============================================================================

import React, { useState } from 'react';
import { useChapterHelp } from '@/hooks/useChapterHelp';
import { Dialog } from '@/components/ui/Dialog';
import { Button } from '@/components/ui/Button';
import './ChapterHelpButton.css';

interface ChapterHelpButtonProps {
  chapterId: string;
  section?: string;
  /** Button text */
  label?: string;
  /** Button variant */
  variant?: 'primary' | 'secondary' | 'outline';
}

export function ChapterHelpButton({
  chapterId,
  section,
  label = 'Get AI Help',
  variant = 'outline',
}: ChapterHelpButtonProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [question, setQuestion] = useState('');
  const { getHelp, loading, data, error, reset } = useChapterHelp(chapterId);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!question.trim()) return;

    try {
      await getHelp(question, section);
    } catch (error) {
      console.error('Chapter help error:', error);
    }
  };

  const handleClose = () => {
    setIsOpen(false);
    reset();
    setQuestion('');
  };

  return (
    <>
      <Button
        variant={variant}
        onClick={() => setIsOpen(true)}
        className="chapter-help-button"
        aria-label={`Get AI help for chapter ${chapterId}`}
      >
        🤖 {label}
      </Button>

      <Dialog
        isOpen={isOpen}
        onClose={handleClose}
        title={`Chapter ${chapterId} Help`}
        size="large"
      >
        <div className="chapter-help-dialog">
          {!data ? (
            <form onSubmit={handleSubmit}>
              <label htmlFor="help-question">
                What do you need help with?
              </label>
              <textarea
                id="help-question"
                value={question}
                onChange={(e) => setQuestion(e.target.value)}
                placeholder="Ask a question about this chapter..."
                rows={4}
                disabled={loading}
                aria-required="true"
              />

              {error && (
                <div className="error-message" role="alert">
                  {error.message}
                </div>
              )}

              <div className="dialog-actions">
                <Button
                  type="button"
                  variant="secondary"
                  onClick={handleClose}
                >
                  Cancel
                </Button>
                <Button
                  type="submit"
                  variant="primary"
                  disabled={loading || !question.trim()}
                  loading={loading}
                >
                  {loading ? 'Getting help...' : 'Ask'}
                </Button>
              </div>
            </form>
          ) : (
            <div className="chapter-help-response">
              <h4>Answer</h4>
              <div className="answer-content">{data.answer}</div>

              {data.relatedConcepts.length > 0 && (
                <div className="related-concepts">
                  <h5>Related Concepts</h5>
                  <ul>
                    {data.relatedConcepts.map((concept, idx) => (
                      <li key={idx}>{concept}</li>
                    ))}
                  </ul>
                </div>
              )}

              {data.sources.length > 0 && (
                <div className="sources">
                  <h5>Sources</h5>
                  <ul>
                    {data.sources.map((source, idx) => (
                      <li key={idx}>
                        <strong>
                          {source.chapter} - {source.section}
                        </strong>
                        <p>{source.contentSnippet}</p>
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              <div className="dialog-actions">
                <Button variant="secondary" onClick={handleClose}>
                  Close
                </Button>
                <Button
                  variant="outline"
                  onClick={() => {
                    reset();
                    setQuestion('');
                  }}
                >
                  Ask Another Question
                </Button>
              </div>
            </div>
          )}
        </div>
      </Dialog>
    </>
  );
}
```

### Step 5: Error Handling Patterns

Implement robust error handling:

```typescript
// src/utils/error-handling.ts

/**
 * Error types for AI service interactions
 */
export class AIServiceError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'AIServiceError';
  }
}

export class NetworkError extends AIServiceError {
  constructor(message: string) {
    super(message, 'NETWORK_ERROR');
  }
}

export class TimeoutError extends AIServiceError {
  constructor(message: string) {
    super(message, 'TIMEOUT_ERROR');
  }
}

export class ValidationError extends AIServiceError {
  constructor(message: string) {
    super(message, 'VALIDATION_ERROR', 400);
  }
}

/**
 * Error boundary component for AI features
 */
export function AIErrorBoundary({
  children,
  fallback,
}: {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}) {
  return (
    <ErrorBoundary
      fallback={
        fallback || (
          <div className="ai-error-fallback">
            <h3>AI Feature Unavailable</h3>
            <p>
              The AI assistant is temporarily unavailable. Please try again later.
            </p>
          </div>
        )
      }
    >
      {children}
    </ErrorBoundary>
  );
}

/**
 * User-friendly error messages
 */
export function getErrorMessage(error: unknown): string {
  if (error instanceof AIServiceError) {
    switch (error.code) {
      case 'NETWORK_ERROR':
        return 'Unable to connect to AI service. Please check your internet connection.';
      case 'TIMEOUT_ERROR':
        return 'The request took too long. Please try again.';
      case 'VALIDATION_ERROR':
        return 'Invalid input. Please check your question and try again.';
      default:
        return error.message;
    }
  }

  if (error instanceof Error) {
    return error.message;
  }

  return 'An unexpected error occurred. Please try again.';
}
```

### Step 6: Loading States

Implement consistent loading indicators:

```typescript
// src/components/ui/LoadingIndicator.tsx

export function LoadingIndicator({ message = 'Thinking...' }) {
  return (
    <div className="loading-indicator" role="status" aria-live="polite">
      <div className="loading-spinner" aria-hidden="true">
        <div className="dot"></div>
        <div className="dot"></div>
        <div className="dot"></div>
      </div>
      <span className="loading-message">{message}</span>
    </div>
  );
}

// src/components/ui/Skeleton.tsx

export function Skeleton({ width, height, variant = 'text' }) {
  return (
    <div
      className={`skeleton skeleton--${variant}`}
      style={{ width, height }}
      aria-hidden="true"
    />
  );
}
```

### Step 7: Integration Examples

#### Example 1: Chatbot in Docusaurus

```typescript
// src/components/ChatWidget/DocusaurusChatWidget.tsx

import React from 'react';
import { ChatWidget } from './ChatWidget';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function DocusaurusChatWidget() {
  return (
    <BrowserOnly fallback={<div>Loading chat...</div>}>
      {() => {
        const currentPage = window.location.pathname;
        const chapterId = currentPage.match(/\/docs\/([\w-]+)/)?.[1];

        return (
          <ChatWidget
            context={{
              currentPage,
              chapterId,
            }}
            showSources={true}
          />
        );
      }}
    </BrowserOnly>
  );
}
```

#### Example 2: Chapter Help Integration

```mdx
<!-- In a chapter MDX file -->

import { ChapterHelpButton } from '@site/src/components/ChapterHelpButton';

## 3. Code Explanation

{/* Content here */}

<ChapterHelpButton
  chapterId="03-inverse-kinematics"
  section="code-explanation"
  label="Need help understanding the code?"
/>
```

### Step 8: Validation Checklist

After implementation, verify:

- [ ] **Type Safety**: All API calls use TypeScript interfaces
- [ ] **Error Handling**: Graceful degradation with user-friendly messages
- [ ] **Loading States**: Visual feedback during async operations
- [ ] **Accessibility**: ARIA labels, keyboard navigation, screen readers
- [ ] **No Backend Logic**: Zero backend code in frontend
- [ ] **Reusability**: Hooks work across different components
- [ ] **Retry Logic**: Failed requests can be retried
- [ ] **Timeouts**: Requests timeout after reasonable duration
- [ ] **Cancellation**: Pending requests can be cancelled
- [ ] **State Management**: Clean separation of concerns

## Configuration

```typescript
// src/config/ai-service.config.ts

export const AI_SERVICE_CONFIG = {
  // Base URL for AI service
  baseUrl: import.meta.env.VITE_AI_SERVICE_URL || 'http://localhost:8000',

  // Request timeout (ms)
  timeout: 30000,

  // Number of retries
  retries: 2,

  // Enable streaming responses
  enableStreaming: true,

  // Show sources by default
  showSources: true,

  // Default depth for explanations
  defaultDepth: 'intermediate' as const,
};
```

## Example Usage

```typescript
// Example 1: Use chatbot hook
function MyChatComponent() {
  const { query, loading, conversationHistory } = useChatbot();

  const handleAsk = async (question: string) => {
    try {
      const response = await query(question);
      console.log('Response:', response);
    } catch (error) {
      console.error('Error:', error);
    }
  };

  return <div>{/* UI */}</div>;
}

// Example 2: Use chapter help
function MyChapterPage({ chapterId }: { chapterId: string }) {
  return (
    <div>
      <h1>Chapter Content</h1>
      <ChapterHelpButton chapterId={chapterId} />
    </div>
  );
}

// Example 3: Use robotics explain
function ConceptExplainer() {
  const { explain, data, loading } = useRoboticsExplain();

  return (
    <button onClick={() => explain('inverse_kinematics', 'intermediate')}>
      Explain Inverse Kinematics
    </button>
  );
}
```

## Output

When this skill is invoked, it generates:

1. **Type Definitions** (`src/types/ai-service.types.ts`)
2. **API Client** (`src/services/ai-service.client.ts`)
3. **Custom Hooks** (`src/hooks/useChatbot.ts`, `useChapterHelp.ts`, `useRoboticsExplain.ts`)
4. **UI Components** (`ChatWidget`, `ChapterHelpButton`)
5. **Error Handling** (`src/utils/error-handling.ts`)
6. **Configuration** (`src/config/ai-service.config.ts`)

## Success Criteria

- ✅ **Frontend-Only**: Zero backend logic
- ✅ **Type-Safe**: TypeScript interfaces throughout
- ✅ **Reusable**: Works across all UI components
- ✅ **Error Handling**: Graceful degradation
- ✅ **Loading States**: Visual feedback
- ✅ **Accessible**: WCAG 2.1 compliant
- ✅ **Consistent**: Standardized patterns
