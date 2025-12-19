# Quickstart Guide: RAG Chatbot Integration With Auth + Advanced Features

## Overview
This guide provides a quick setup and development workflow for the authenticated RAG chatbot system with advanced features.

## Prerequisites
- Python 3.11+ with pip
- Node.js 18+ with npm/yarn
- Docusaurus development environment
- Existing auth-backend running
- RAG system (LangChain/LlamaIndex) configured

## Development Setup

### 1. Clone and Navigate
```bash
cd C:\new - Copy\physical-ai-robotics-textbook
```

### 2. Backend Setup (RAG Chatbot)
```bash
cd rag-chatbot
pip install -r requirements.txt  # if exists
# Or install required packages:
pip install fastapi uvicorn python-multipart
```

### 3. Frontend Setup (Docusaurus)
```bash
cd physical-ai-robotics-textbook/docusaurus
npm install
```

### 4. Auth Backend (Ensure Running)
```bash
cd auth-backend
# Make sure auth backend is running on expected port (typically 8001)
```

## Running the System

### 1. Start Auth Backend (Terminal 1)
```bash
cd auth-backend
python main.py
```

### 2. Start RAG Chatbot (Terminal 2)
```bash
cd rag-chatbot
python main.py
```

### 3. Start Docusaurus (Terminal 3)
```bash
cd physical-ai-robotics-textbook/docusaurus
npm start
```

## Key Development Commands

### Backend Development
```bash
# Run tests
cd rag-chatbot
python -m pytest tests/

# Format code
black .
```

### Frontend Development
```bash
# Build for production
npm run build

# Serve production build locally
npm run serve
```

## API Testing
```bash
# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "message": "Explain inverse kinematics",
    "selected_text": "Inverse kinematics is the process of computing joint angles...",
    "streaming": false
  }'
```

## Implementation Tasks

### 1. Add Chatbot Component
Create `docusaurus/src/components/Chatbot/index.tsx` with:
- Authentication check using existing auth context
- Modal integration for login/signup
- Selected text handling
- Streaming response display

### 2. Update RAG Backend
Modify `rag-chatbot/api/chat.py` to:
- Validate authentication tokens
- Handle selected_text parameter
- Implement streaming responses
- Add source citations

### 3. Add API Endpoints
Create endpoints in `rag-chatbot/main.py`:
- POST `/api/chat` - Main chat endpoint
- POST `/api/chat/session` - Create chat session
- GET `/api/chat/session/{id}` - Get session info

### 4. Integrate with Existing UI
- Add chatbot icon to Docusaurus pages
- Implement text selection detection
- Connect to existing auth modals

## Environment Variables
Create `.env` files as needed:

### rag-chatbot/.env
```
AUTH_BACKEND_URL=http://localhost:8001
RAG_MODEL_NAME=gpt-4  # or your preferred model
```

## Testing the Features

### Authentication Flow
1. Navigate to Docusaurus site without logging in
2. Click chatbot icon → Should show login/signup modal
3. After login → Chatbot should be accessible

### Selected Text Feature
1. Select text on any page
2. Click chatbot icon → Selected text should appear in input
3. Ask question → Response should be based on selected text

### Streaming Response
1. Enable streaming in request
2. Observe progressive response display
3. Verify smooth loading indicators

### Source Citations
1. Ask a question that uses RAG data
2. Verify sources appear below response
3. Check source format matches requirements

## Troubleshooting

### Common Issues
- **Auth not working**: Verify auth-backend is running and accessible
- **Streaming not working**: Check SSE support in browser and backend
- **Selected text not appearing**: Verify text selection detection logic
- **CLI broken**: Ensure backward compatibility changes are isolated

### Debugging API
```bash
# Enable debug logging
export DEBUG=1
```

## Next Steps
1. Implement the chatbot component in Docusaurus
2. Enhance the RAG backend with authentication and streaming
3. Test all features together
4. Update documentation
5. Create detailed task list with `/sp.tasks`