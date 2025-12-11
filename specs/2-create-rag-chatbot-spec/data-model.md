# Data Model for RAG Chatbot

This document describes the data models for the Qdrant vector database and the Neon serverless Postgres database.

## Qdrant Collection Schema

**Collection Name**: `physical_ai_textbook`

**Vector Size**: 1536 (from OpenAI's `text-embedding-3-small`, will need to be confirmed for Gemini)

**Distance Metric**: Cosine

**Payload Structure**:
The payload for each vector will contain the following metadata:

| Field | Type | Description |
|---|---|---|
| `content` | `string` | The text content of the document chunk. |
| `module` | `string` | The name of the module the document belongs to (e.g., "Module 1"). |
| `chapter` | `string` | The name of the chapter the document belongs to (e.g., "ROS 2 Fundamentals"). |
| `source_file` | `string` | The path to the source markdown file (e.g., "docs/module-1/ros2.md"). |
| `chunk_index` | `integer` | The index of the chunk within the original document. |
| `total_chunks` | `integer` | The total number of chunks in the original document. |
| `word_count` | `integer` | The number of words in the chunk. |
| `created_at` | `string` | The ISO timestamp of when the chunk was created. |

## Postgres Database Schema

The Postgres database will store conversation history and related metadata.

### `conversations` table

This table stores metadata for each conversation.

| Column | Type | Constraints | Description |
|---|---|---|---|
| `id` | `VARCHAR(255)` | `PRIMARY KEY` | A unique identifier for the conversation (e.g., a UUID). |
| `user_id` | `VARCHAR(255)` | | A foreign key to a future `users` table for authenticated users. |
| `created_at` | `TIMESTAMP` | `DEFAULT CURRENT_TIMESTAMP` | The timestamp of when the conversation was created. |
| `updated_at` | `TIMESTAMP` | `DEFAULT CURRENT_TIMESTAMP` | The timestamp of the last message in the conversation. |
| `metadata` | `JSONB` | | A flexible column to store additional metadata about the conversation. |

### `messages` table

This table stores each message within a conversation.

| Column | Type | Constraints | Description |
|---|---|---|---|
| `id` | `SERIAL` | `PRIMARY KEY` | A unique identifier for the message. |
| `conversation_id` | `VARCHAR(255)` | `NOT NULL, REFERENCES conversations(id) ON DELETE CASCADE` | A foreign key to the `conversations` table. |
| `role` | `VARCHAR(50)` | `NOT NULL, CHECK (role IN ('user', 'assistant', 'system'))` | The role of the message sender. |
| `content` | `TEXT` | `NOT NULL` | The text content of the message. |
| `sources` | `JSONB` | | An array of source objects that the assistant used to generate the response. |
| `created_at` | `TIMESTAMP` | `DEFAULT CURRENT_TIMESTAMP` | The timestamp of when the message was created. |
| `token_count` | `INT` | | The number of tokens in the message, for future analytics. |
| `latency_ms` | `INT` | | The time it took to generate the assistant's response. |

### Indexes

- `idx_messages_conversation` on `messages(conversation_id, created_at)`
- `idx_conversations_updated` on `conversations(updated_at DESC)`

## SQL Schema Definition

```sql
-- Table 1: conversations
CREATE TABLE conversations (
    id VARCHAR(255) PRIMARY KEY,
    user_id VARCHAR(255), -- Future: for auth
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB -- Future: user preferences
);

-- Table 2: messages
CREATE TABLE messages (
    id SERIAL PRIMARY KEY,
    conversation_id VARCHAR(255) NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(50) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    sources JSONB, -- Array of source objects
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    token_count INT, -- For future analytics
    latency_ms INT -- Response time tracking
);

-- Indexes
CREATE INDEX idx_messages_conversation ON messages(conversation_id, created_at);
CREATE INDEX idx_conversations_updated ON conversations(updated_at DESC);
```