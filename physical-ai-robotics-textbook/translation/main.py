from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from router import router
import uvicorn
import logging
from dotenv import load_dotenv

# ✅ Load .env file FIRST
load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

logger.info("=" * 50)
logger.info("🚀 TRANSLATION SERVICE WITH GEMINI")
logger.info("=" * 50)

app = FastAPI(
    title="Translation Service API",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(router)

@app.get("/")
async def root():
    return {
        "status": "online",
        "message": "🌐 Translation Service (Gemini)",
        "version": "1.0.0"
    }

@app.get("/health")
async def health():
    return {"status": "healthy", "service": "translation-gemini"}

if __name__ == "__main__":
    logger.info("🎬 Starting server...")
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )