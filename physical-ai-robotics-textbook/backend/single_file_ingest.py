"""
Single File Ingestion - Process one file at a time
Save as: single_file_ingest.py
Run: python single_file_ingest.py path/to/file.md
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).parent)

from app.config import settings
from app.services.embedding_service import embedding_service
from app.utils.text_processing import text_processor
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import hashlib

async def ingest_single_file(file_path: str):
    """Ingest one markdown file"""
    
    file_path = Path(file_path)
    
    if not file_path.exists():
        print(f"❌ File not found: {file_path}")
        return
    
    print("="*60)
    print(f"INGESTING: {file_path.name}")
    print("="*60)
    
    # Read file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    print(f"✓ Read file ({len(content)} chars)")
    
    # Chunk
    print("✓ Chunking...")
    chunk_dicts = text_processor.chunk_text(
        text=content,
        chunk_size=1000,
        overlap=200
    )
    
    chunks = [c['content'] for c in chunk_dicts[:30]]  # Max 30 chunks
    print(f"✓ Created {len(chunks)} chunks")
    
    # Get metadata
    chapter = file_path.stem.replace('-', ' ').title()
    
    # Connect to Qdrant
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )
    
    # Process chunks
    print("✓ Generating embeddings...")
    points = []
    
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}/{len(chunks)}...", end='\r')
        
        embedding = await embedding_service.generate_embedding(chunk)
        
        chunk_id = hashlib.md5(f"{file_path}_{i}".encode()).hexdigest()
        
        point = PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                'content': chunk,
                'source_file': str(file_path),
                'module': 'manual',
                'chapter': chapter,
                'chunk_index': i,
                'total_chunks': len(chunks)
            }
        )
        points.append(point)
        
        await asyncio.sleep(0.3)  # Rate limiting
    
    # Upload
    print("\n✓ Uploading to Qdrant...")
    client.upsert(
        collection_name=settings.qdrant_collection_name,
        points=points
    )
    
    print(f"\n✅ SUCCESS! Ingested {len(points)} chunks")
    print("="*60)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python single_file_ingest.py <file_path>")
        print("\nExample:")
        print('  python single_file_ingest.py "C:/docs/chapter1.md"')
    else:
        asyncio.run(ingest_single_file(sys.argv[1]))