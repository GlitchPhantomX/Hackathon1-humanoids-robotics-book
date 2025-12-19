"""
Smart Humanoid Robotics Agent with STREAMING
Complete FastAPI Backend - Ready to Run!
"""

import os
import json
from dotenv import load_dotenv
from openai import AsyncOpenAI
import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

# ======================================================
# ENV SETUP
# ======================================================

load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")

if not OPENAI_API_KEY:
    raise RuntimeError("❌ OPENAI_API_KEY missing")

# ======================================================
# CLIENTS
# ======================================================

client = AsyncOpenAI(api_key=OPENAI_API_KEY)

cohere_client = cohere.Client(COHERE_API_KEY) if COHERE_API_KEY else None

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
) if QDRANT_API_KEY and QDRANT_URL else None

COLLECTION_NAME = "humanoid_ai_book"

# ======================================================
# FASTAPI APP
# ======================================================

app = FastAPI(title="Physical AI Textbook Chatbot")

# CORS Configuration - Allow all origins for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str

# ======================================================
# TOOL: EMBEDDINGS + RETRIEVAL
# ======================================================

def get_embedding(text: str):
    """Generate embeddings using Cohere"""
    if not cohere_client:
        return None
    try:
        res = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",
            texts=[text]
        )
        return res.embeddings[0]
    except Exception as e:
        print(f"❌ Embedding Error: {e}")
        return None


def retrieve_textbook(query: str) -> dict:
    """Retrieve relevant content from Qdrant vector database"""
    try:
        if not qdrant or not cohere_client:
            return {
                "found": False,
                "content": "RAG system not configured. Please check QDRANT_URL and COHERE_API_KEY.",
                "sources": []
            }

        embedding = get_embedding(query)
        if not embedding:
            return {
                "found": False,
                "content": "Could not generate embedding for query.",
                "sources": []
            }

        result = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=5,
            score_threshold=0.35
        )

        if not result.points:
            return {
                "found": False,
                "content": "No relevant textbook content found.",
                "sources": []
            }

        content = []
        sources = []

        for i, p in enumerate(result.points, 1):
            text = p.payload.get("text", "")
            title = p.payload.get("page_title", "Unknown")

            content.append(f"[Source {i}: {title}]\n{text}")
            sources.append({
                "title": title,
                "relevance": f"{p.score:.2f}"
            })

        return {
            "found": True,
            "content": "\n\n".join(content),
            "sources": sources
        }

    except Exception as e:
        print(f"❌ Retrieval Error: {e}")
        return {
            "found": False,
            "content": f"Error retrieving content: {str(e)}",
            "sources": []
        }

# ======================================================
# TOOL SCHEMA (OpenAI Function Calling)
# ======================================================

TOOLS = [{
    "type": "function",
    "function": {
        "name": "retrieve_textbook",
        "description": "Retrieve relevant sections from the Humanoid Robotics textbook",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "Search query for retrieving textbook content"
                }
            },
            "required": ["query"]
        }
    }
}]

# ======================================================
# SMART AGENT WITH STREAMING
# ======================================================

async def smart_agent_stream(user_query: str):
    """
    Main agent function with streaming support
    """
    messages = [
        {
            "role": "system",
            "content": """You are a **Professional AI Tutor** for Physical AI & Humanoid Robotics.

## YOUR ROLE:
You provide expert guidance on robotics, ROS2, simulation, and humanoid systems with exceptional clarity and structure.

## RESPONSE FORMATTING RULES:

### 1. ALWAYS USE RICH MARKDOWN:
- Use **## Main Headings** for topics
- Use **### Subheadings** for sections  
- Use **bullet points (•)** for lists
- Use **numbered lists (1. 2. 3.)** for steps
- Use **bold** for emphasis on key terms
- Use **code blocks** with ```language for code
- Use **tables** for comparisons when appropriate

### 2. STRUCTURE PATTERN:
```
## Main Topic

Brief introduction in 1-2 sentences.

### Key Concept 1
Explanation with details.

**Important Points:**
• First point with explanation
• Second point with explanation
• Third point with explanation

### Key Concept 2
Another explanation.

**Implementation Steps:**
1. First step with details
2. Second step with details
3. Third step with details

### Quick Reference Table (when applicable)
| Feature | Description |
|---------|-------------|
| Item 1  | Details     |
| Item 2  | Details     |
```

### 3. VISUAL ELEMENTS:
- Use ✅ for positive points
- Use ❌ for warnings/issues
- Use 🔥 for important highlights
- Use 💡 for tips
- Use ⚡ for quick facts

### 4. CONTENT RULES:
- **FOR TEXTBOOK QUESTIONS**: Call `retrieve_textbook` first
- Answer based on retrieved content when available
- If no content found, provide general knowledge
- Always cite sources: `*Source: [Title]*`
- Be technically accurate but accessible

### 5. GREETING RESPONSES:
For casual greetings like "hi", "hello", "hey":
- Respond warmly with emojis
- Show what you can help with in a formatted list
- Keep it concise and inviting

### 6. CODE EXAMPLES:
When providing code, always use proper syntax highlighting:
```python
# Your code here
```

Remember: Every response should be visually appealing and easy to scan!"""
        },
        {"role": "user", "content": user_query}
    ]

    # First pass - check if we need to call tools
    try:
        initial_response = await client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            tools=TOOLS,
            temperature=0.3
        )

        msg = initial_response.choices[0].message

        # If tool call needed, execute it first
        if msg.tool_calls:
            tool_call = msg.tool_calls[0]
            args = json.loads(tool_call.function.arguments)
            
            print(f"🔍 Searching textbook for: {args['query']}")
            tool_result = retrieve_textbook(args["query"])
            
            messages.append({
                "role": "assistant",
                "tool_calls": [tool_call]
            })
            
            messages.append({
                "role": "tool",
                "tool_call_id": tool_call.id,
                "content": json.dumps(tool_result)
            })

        # Now stream the actual response
        stream = await client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            temperature=0.3,
            stream=True
        )

        async for chunk in stream:
            if chunk.choices[0].delta.content:
                content = chunk.choices[0].delta.content
                yield f"data: {json.dumps({'type': 'content', 'data': content})}\n\n"
        
        # Send completion signal
        yield f"data: {json.dumps({'type': 'done'})}\n\n"

    except Exception as e:
        print(f"❌ Agent Error: {e}")
        error_msg = f"Sorry, I encountered an error: {str(e)}"
        yield f"data: {json.dumps({'type': 'content', 'data': error_msg})}\n\n"
        yield f"data: {json.dumps({'type': 'done'})}\n\n"

# ======================================================
# API ENDPOINTS
# ======================================================

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "online",
        "message": "🤖 AI Tutor Backend with Streaming",
        "version": "2.0",
        "endpoints": {
            "chat": "/chat (POST)",
            "health": "/ (GET)"
        }
    }

@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint with streaming support
    
    Request body:
    {
        "message": "Your question here"
    }
    
    Returns: Server-Sent Events (SSE) stream
    """
    return StreamingResponse(
        smart_agent_stream(request.message),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"  # Disable nginx buffering
        }
    )

@app.get("/health")
async def health_check():
    """System health check"""
    return {
        "status": "healthy",
        "openai": "connected" if OPENAI_API_KEY else "missing",
        "cohere": "connected" if cohere_client else "not configured",
        "qdrant": "connected" if qdrant else "not configured"
    }

# ======================================================
# RUN SERVER
# ======================================================

if __name__ == "__main__":
    import uvicorn
    print("\n" + "="*50)
    print("🤖 AI TUTOR BACKEND STARTING...")
    print("="*50)
    print(f"✅ OpenAI: {'Connected' if OPENAI_API_KEY else '❌ Missing'}")
    print(f"✅ Cohere: {'Connected' if cohere_client else '⚠️ Not configured'}")
    print(f"✅ Qdrant: {'Connected' if qdrant else '⚠️ Not configured'}")
    print("="*50)
    print("🚀 Server running at: http://127.0.0.1:8000")
    print("📚 Docs available at: http://127.0.0.1:8000/docs")
    print("="*50 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000)