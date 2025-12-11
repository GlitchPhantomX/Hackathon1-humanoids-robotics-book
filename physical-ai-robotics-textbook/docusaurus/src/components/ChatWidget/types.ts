// Types for the RAG Chatbot frontend components

export interface Message {
  id: number;
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceCitation[];
  timestamp: string;
}

export interface SourceCitation {
  source_file: string;
  module: string;
  chapter: string;
  chunk_index: number;
  relevance_score: number;
  content: string;
}

export interface ChatRequest {
  message: string;
  conversation_id?: string;
  selected_text?: string;
}

export interface ChatResponse {
  response: string;
  conversation_id: string;
  sources: SourceCitation[];
  timestamp: string;
}