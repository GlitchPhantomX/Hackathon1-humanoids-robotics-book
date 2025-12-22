from fastapi import FastAPI
from translation.router import router
import uvicorn
import os

# Create FastAPI application instance
app = FastAPI(
    title="Urdu Translation API",
    description="API for translating content to Urdu with markdown preservation",
    version="1.0.0"
)

# Include the translation router
app.include_router(router)

# Add a basic health check endpoint
@app.get("/")
async def root():
    return {"message": "Urdu Translation API is running"}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "urdu-translation-api"}

# If this file is run directly, start the server
if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)