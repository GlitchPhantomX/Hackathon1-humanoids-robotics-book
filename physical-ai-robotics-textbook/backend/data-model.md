# Data Model Documentation: RAG Chatbot for Physical AI & Robotics Textbook

## Overview

This document describes the data models used in the RAG chatbot system, including both the PostgreSQL database schema and the vector database payload structure.

## PostgreSQL Database Schema

### conversations Table
Stores conversation metadata and relationships.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the conversation |
| user_id | VARCHAR(255) | | Optional user identifier |
| title | VARCHAR(500) | | Conversation title (auto-generated from first message) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | When the conversation was created |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | When the conversation was last updated |

### messages Table
Stores individual messages within conversations.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the message |
| conversation_id | UUID | FOREIGN KEY, NOT NULL | Reference to the parent conversation |
| role | VARCHAR(20) | NOT NULL, CHECK | 'user' or 'assistant' |
| content | TEXT | NOT NULL | The message content |
| timestamp | TIMESTAMP | NOT NULL, DEFAULT NOW() | When the message was created |
| sources | JSONB | | Source citations in JSON format (for assistant messages) |

## Vector Database Payload Structure (Qdrant)

### Document Chunk Payload
Each document chunk stored in Qdrant contains the following fields:

| Field | Type | Description |
|-------|------|-------------|
| content | String | The actual text content of the chunk |
| module | String | Module name from textbook structure |
| chapter | String | Chapter name from textbook structure |
| source_file | String | Original file path |
| chunk_index | Integer | Sequential index of the chunk within the document |
| embedding | Vector | The embedding vector for similarity search |

### Metadata Structure
Additional metadata stored with each vector:

```json
{
  "module": "string",
  "chapter": "string",
  "source_file": "string",
  "chunk_index": "integer",
  "created_at": "timestamp"
}
```

## Pydantic Models

### Request/Response Models
- `ChatRequest`: Contains message, conversation_id, selected_text, and user_id
- `ChatResponse`: Contains response, conversation_id, sources, and timestamp
- `IngestRequest`: Contains force_refresh flag and source_path
- `IngestResponse`: Contains success status, message, and statistics
- `QueryRequest`: Contains query, conversation_id, top_k, and filters
- `QueryResponse`: Contains query, results, relevance_scores, and processing_time
- `HealthResponse`: Contains status and service health information

### Internal Models
- `ConversationBase`: Base model for conversation data
- `MessageBase`: Base model for message data
- `DocumentChunk`: Model for document chunks stored in vector database
- `RetrievedChunk`: Model for chunks retrieved from vector database

## Relationships

- One conversation can have many messages (1:N relationship)
- Each message belongs to exactly one conversation
- Document chunks are stored in Qdrant vector database, not PostgreSQL
- Source citations in messages reference document chunks in Qdrant

## Indexes

### PostgreSQL
- conversations table: index on (user_id, updated_at)
- messages table: index on (conversation_id, timestamp)

### Qdrant
- Vector index on embedding field for similarity search
- Field indexes on module, chapter, and source_file for filtering

## Data Flow

1. Documents are parsed and split into chunks
2. Chunks are stored in Qdrant with embeddings and metadata
3. User conversations are stored in PostgreSQL
4. When responding to queries:
   - Query is embedded and searched in Qdrant
   - Relevant chunks are retrieved with relevance scores
   - Context is passed to LLM to generate response
   - Response is stored in messages table with source citations