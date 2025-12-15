"""
Micro Ingestion - Paste text directly, no file reading
Save as: micro_ingest.py
Run: python micro_ingest.py
Then paste your text when prompted
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).parent)

from app.config import settings
from app.services.embedding_service import embedding_service
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import hashlib

async def ingest_text():
    """Ingest text directly from input"""
    
    print("="*60)
    print("MICRO INGESTION")
    print("="*60)
    
    # Get chapter name
    chapter = input("\nEnter chapter name (e.g., 'Introduction'): ").strip()
    
    print("\nPaste your text below (press Ctrl+Z then Enter on Windows when done):")
    print("-" * 60)
    
    # Read text from stdin
    lines = []
    try:
        while True:
            line = input()
            lines.append(line)
    except EOFError:
        pass
    
    text = '\n'.join(lines)
    
    if len(text.strip()) < 20:
        print("❌ Text too short!")
        return
    
    print("\n" + "="*60)
    print(f"✓ Got {len(text)} characters")
    
    # Simple chunk (300 chars each)
    chunk_size = 300
    chunks = []
    
    for i in range(0, len(text), chunk_size):
        chunk = text[i:i+chunk_size].strip()
        if len(chunk) > 20:
            chunks.append(chunk)
        if len(chunks) >= 10:  # Max 10 chunks
            break
    
    print(f"✓ Created {len(chunks)} chunks")
    
    # Connect to Qdrant
    print("✓ Connecting to Qdrant...")
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )
    
    # Process chunks
    print("✓ Processing...\n")
    
    for i, chunk in enumerate(chunks):
        print(f"  [{i+1}/{len(chunks)}] Embedding... ", end='')
        
        embedding = await embedding_service.generate_embedding(chunk)
        
        chunk_id = hashlib.md5(f"{chapter}_{i}".encode()).hexdigest()
        
        point = PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                'content': chunk,
                'source_file': 'manual_input',
                'module': 'manual',
                'chapter': chapter,
                'chunk_index': i,
                'total_chunks': len(chunks)
            }
        )
        
        client.upsert(
            collection_name=settings.qdrant_collection_name,
            points=[point]
        )
        
        print("✓")
        await asyncio.sleep(0.5)
    
    print("\n" + "="*60)
    print(f"✅ SUCCESS! Uploaded {len(chunks)} chunks")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(ingest_text())