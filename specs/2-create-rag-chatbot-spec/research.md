# Research for RAG Chatbot

This document outlines the research needed to resolve ambiguities and make technical decisions for the RAG Chatbot project.

## Research Tasks

### 1. Gemini API Integration

**Question**: What are the specific Gemini API endpoints and SDKs to be used for embeddings and text generation, as mandated by the constitution?

**Context**: The specification mentions the OpenAI API, but the constitution requires the use of the Gemini API. This research task is to identify the exact Gemini APIs that will be used to fulfill the requirements of the project.

**Acceptance Criteria**:
- Identify the Gemini API endpoint for text embeddings.
- Identify the Gemini API endpoint for text generation (equivalent to GPT-4o).
- Document the SDK or library to be used in the Python backend to interact with the Gemini API.
- Provide code examples for authentication and basic API calls.

### 2. Docusaurus ChatKit Integration

**Question**: What is the best approach to integrate a ChatKit-like UI into Docusaurus?

**Context**: The spec mentions using the ChatKit Python SDK, but the frontend is a React-based Docusaurus site. This task is to determine the best way to build or integrate a chat widget into the Docusaurus frontend.

**Acceptance Criteria**:
- Evaluate existing React chat component libraries.
- Outline the steps to create a custom chat widget in Docusaurus.
- Decide on a strategy for managing chat state across page navigations.

### 3. Text Selection in Docusaurus

**Question**: How can user text selections be reliably captured in Docusaurus and mapped to the source documents?

**Context**: A key feature is allowing users to select text and ask questions about it. This requires a robust mechanism to capture the selected text and, ideally, map it back to the original source document and chunk.

**Acceptance Criteria**:
- Research methods for capturing text selection in a React application.
- Propose a strategy for mapping selected text to the source document chunk. This may involve annotating the HTML during the build process.
- Define the data format to be sent to the backend when a user asks a question about selected text.