"""
Ultra-Safe Mini Ingestion - Won't hang your laptop
Save as: safe_ingest.py
Run: python safe_ingest.py path/to/file.md
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

def simple_chunk(text: str, size: int = 500) -> list:
    """Super simple chunking that won't hang"""
    chunks = []
    words = text.split()
    
    current_chunk = []
    current_size = 0
    
    for word in words:
        current_chunk.append(word)
        current_size += len(word) + 1
        
        if current_size >= size:
            chunks.append(' '.join(current_chunk))
            current_chunk = []
            current_size = 0
    
    if current_chunk:
        chunks.append(' '.join(current_chunk))
    
    return chunks

async def safe_ingest(file_path: str):
    """Safely ingest one file"""
    
    file_path = Path(file_path)
    
    if not file_path.exists():
        print(f"❌ File not found: {file_path}")
        return
    
    print("="*60)
    print(f"PROCESSING: {file_path.name}")
    print("="*60)
    
    try:
        # Read file
        print("Step 1: Reading file...")
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        file_size = len(content)
        print(f"✓ File size: {file_size} chars")
        
        # Skip if too large
        if file_size > 100000:
            print("⚠️  File too large! Split it manually first.")
            return
        
        # Simple chunking
        print("Step 2: Chunking (simple method)...")
        chunks = simple_chunk(content, size=500)
        
        # Limit chunks
        max_chunks = 20  # Process max 20 chunks at a time
        if len(chunks) > max_chunks:
            print(f"⚠️  Too many chunks ({len(chunks)}), limiting to {max_chunks}")
            chunks = chunks[:max_chunks]
        
        print(f"✓ Created {len(chunks)} chunks")
        
        # Metadata
        chapter = file_path.stem.replace('-', ' ').title()
        
        # Connect to Qdrant
        print("Step 3: Connecting to Qdrant...")
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        print("✓ Connected")
        
        # Process ONE chunk at a time
        print("Step 4: Processing chunks one by one...")
        uploaded = 0
        
        for i, chunk in enumerate(chunks):
            try:
                print(f"\n  Processing chunk {i+1}/{len(chunks)}...")
                
                # Skip empty chunks
                if len(chunk.strip()) < 20:
                    print("  ⏭️  Too short, skipping")
                    continue
                
                # Generate embedding
                print("    - Generating embedding...")
                embedding = await embedding_service.generate_embedding(chunk)
                print("    ✓ Embedding created")
                
                # Create point
                chunk_id = hashlib.md5(
                    f"{file_path}_{i}_{hash(chunk[:50])}".encode()
                ).hexdigest()
                
                point = PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        'content': chunk,
                        'source_file': str(file_path),
                        'module': file_path.parent.name,
                        'chapter': chapter,
                        'chunk_index': i,
                        'total_chunks': len(chunks)
                    }
                )
                
                # Upload immediately (one at a time)
                print("    - Uploading to Qdrant...")
                client.upsert(
                    collection_name=settings.qdrant_collection_name,
                    points=[point]  # Single point
                )
                print("    ✓ Uploaded")
                
                uploaded += 1
                
                # Wait between chunks (important!)
                await asyncio.sleep(1)
                
            except Exception as e:
                print(f"    ❌ Chunk {i+1} failed: {e}")
                print("    Continuing to next chunk...")
                continue
        
        # Summary
        print("\n" + "="*60)
        print(f"✅ SUCCESS!")
        print(f"Uploaded: {uploaded}/{len(chunks)} chunks")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python safe_ingest.py <file_path>")
        print("\nExample:")
        print('  python safe_ingest.py "../docs/00-introduction/index.md"')
    else:
        asyncio.run(safe_ingest(sys.argv[1]))