# Data Model: RAG Chatbot Integration With Auth + Advanced Features

## Overview
This document defines the data models and entities for the authenticated RAG chatbot system with advanced features.

## Core Entities

### 1. Chat Request
**Purpose**: Represents a user's query to the chatbot system

**Fields**:
- `id` (string): Unique identifier for the request
- `message` (string): The user's question or message
- `selected_text` (string, optional): Text selected by user that restricts the answer scope
- `user_id` (string, optional): Identifier of the authenticated user
- `session_token` (string, optional): Authentication session token
- `timestamp` (datetime): When the request was made
- `streaming_enabled` (boolean): Whether to stream the response

**Validation Rules**:
- `message` must be 1-2000 characters
- `selected_text` must be 1-10000 characters if provided
- `user_id` required if authentication is enforced
- `session_token` must be valid if provided

### 2. Chat Response
**Purpose**: Represents the chatbot's response to a user query

**Fields**:
- `id` (string): Unique identifier for the response
- `request_id` (string): Reference to the original request
- `answer` (string): The chatbot's answer to the query
- `sources` (array of SourceCitation, optional): Citations for RAG-based answers
- `status` (string): Status of the response (e.g., "complete", "streaming", "error")
- `timestamp` (datetime): When the response was generated
- `stream_token` (string, optional): Token for streaming continuation

**Validation Rules**:
- `answer` must not exceed 10000 characters
- `sources` required when using RAG (not selected-text only)
- `status` must be one of predefined values

### 3. Source Citation
**Purpose**: Represents a citation to the source document used in the response

**Fields**:
- `title` (string): Title of the source document or section
- `page` (string): Page number or section identifier
- `url` (string, optional): URL to the source location
- `score` (number): Relevance score (0.0 to 1.0)
- `content_preview` (string): Brief preview of the cited content

**Validation Rules**:
- `title` must be 1-200 characters
- `score` must be between 0.0 and 1.0
- `page` or `url` must be provided

### 4. User Session
**Purpose**: Represents an authenticated user session

**Fields**:
- `user_id` (string): Unique identifier for the user
- `session_token` (string): Authentication token
- `expires_at` (datetime): When the session expires
- `created_at` (datetime): When the session was created
- `last_activity` (datetime): Last interaction timestamp

**Validation Rules**:
- `session_token` must be unique and properly formatted
- `expires_at` must be in the future
- `user_id` must exist in the auth system

### 5. Chat Session
**Purpose**: Represents a conversation session between user and chatbot

**Fields**:
- `session_id` (string): Unique identifier for the chat session
- `user_id` (string, optional): Associated user (null for anonymous sessions)
- `created_at` (datetime): When the session started
- `last_message_at` (datetime): When the last message was sent
- `message_count` (integer): Number of messages in the session
- `active` (boolean): Whether the session is currently active

**Validation Rules**:
- `message_count` must be >= 0
- `active` must be boolean
- `user_id` optional for anonymous sessions

## State Transitions

### Chat Request States
1. `pending` → Request received, processing started
2. `processing` → RAG system working on response
3. `streaming` → Response being sent in chunks (if streaming enabled)
4. `complete` → Response fully generated
5. `error` → Error occurred during processing

### Chat Session States
1. `active` → Session is ongoing
2. `inactive` → Session has no recent activity
3. `terminated` → Session ended by user or timeout

## Relationships

```
User Session (1) ←→ (0..n) Chat Session
Chat Session (1) ←→ (0..n) Chat Request
Chat Request (1) → (0..n) Source Citation
```

## Data Flow

1. **Request Flow**: User sends Chat Request → System validates → Processes through RAG → Generates Chat Response with optional Source Citations

2. **Session Flow**: User authenticates → Creates User Session → Initiates Chat Session → Multiple Chat Requests/Responses → Session termination

## Constraints

- All timestamps use ISO 8601 format
- Text fields are UTF-8 encoded
- Session tokens use secure, random generation
- Rate limiting applies per user session
- Selected text restriction applies only when field is provided