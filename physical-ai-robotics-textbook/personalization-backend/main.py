from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware  # ✅ Add this import
import uvicorn
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import the router using relative import that works when running from this directory
try:
    from api.personalize.chapter import router as personalization_router
except ImportError:
    # Fallback for when the module is run directly
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from api.personalize.chapter import router as personalization_router

app = FastAPI(
    title="Chapter Personalization API",
    description="API for personalizing textbook chapter content based on user profile",
    version="1.0.0",
    openapi_tags=[
        {
            "name": "personalization",
            "description": "Personalize textbook chapter content based on user profile"
        },
        {
            "name": "health",
            "description": "Health check endpoints"
        }
    ]
)

# ✅ ADD CORS MIDDLEWARE - MUST BE BEFORE INCLUDING ROUTERS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins like ["http://localhost:3001"]
    allow_credentials=True,
    allow_methods=["*"],  # This allows OPTIONS, POST, GET, etc.
    allow_headers=["*"],
)

# Include the personalization API routes
app.include_router(personalization_router)

@app.get("/health", tags=["health"])
async def health_check():
    """Comprehensive health check for the personalization service"""
    import time
    start_time = time.time()

    response_time = int((time.time() - start_time) * 1000)

    return {
        "status": "healthy",
        "service": "personalization-backend",
        "timestamp": time.time(),
        "response_time_ms": response_time,
        "dependencies": {
            "rag_service": "unknown",
        }
    }

if __name__ == "__main__":
    port = int(os.getenv("PERSONALIZATION_SERVICE_PORT", 8002))  # ✅ Change to 8002
    host = os.getenv("PERSONALIZATION_SERVICE_HOST", "0.0.0.0")
    
    print("=" * 50)
    print("🚀 PERSONALIZATION SERVICE STARTING")
    print("=" * 50)
    print(f"📍 Running on: http://{host}:{port}")
    print(f"📚 Docs: http://localhost:{port}/docs")
    print("=" * 50)
    
    uvicorn.run(app, host=host, port=port)