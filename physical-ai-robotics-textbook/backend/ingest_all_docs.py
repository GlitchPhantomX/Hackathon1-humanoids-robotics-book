"""
Document Ingestion Script
Ingests all markdown files from docusaurus/docs folder into Qdrant
Save as: ingest_all_docs.py
Run: python ingest_all_docs.py
"""

import asyncio
import os
import sys
from pathlib import Path
from typing import List, Dict, Any

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.config import settings
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from app.utils.text_processing import text_processor  # Fixed import
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import hashlib

async def read_markdown_file(file_path: Path) -> Dict[str, Any]:
    """Read and parse a markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract metadata from file path
        relative_path = file_path.relative_to(DOCS_DIR)
        parts = list(relative_path.parts)
        
        # Get chapter name (filename without .md)
        chapter = file_path.stem.replace('-', ' ').title()
        
        # Get module (folder structure)
        if len(parts) > 1:
            module = '/'.join(parts[:-1])
        else:
            module = 'root'
        
        return {
            'content': content,
            'source_file': str(file_path),
            'module': module,
            'chapter': chapter,
            'relative_path': str(relative_path)
        }
    except Exception as e:
        print(f"❌ Error reading {file_path}: {e}")
        return None

async def ingest_document(doc: Dict[str, Any], doc_index: int):
    """Ingest a single document with chunking"""
    try:
        content = doc['content']
        
        # Skip if content is too short
        if len(content.strip()) < 50:
            print(f"⚠️  Skipping {doc['chapter']} (too short)")
            return 0
        
        # Skip if content is too large (> 500KB)
        if len(content) > 500000:
            print(f"⚠️  Skipping {doc['chapter']} (too large: {len(content)} chars)")
            return 0
        
        print(f"\n📄 Processing: {doc['chapter']}")
        print(f"   Module: {doc['module']}")
        print(f"   Content size: {len(content)} chars")
        
        # Chunk the content with error handling
        try:
            chunk_dicts = text_processor.chunk_text(
                text=content,
                chunk_size=min(settings.chunk_size, 2000),  # Limit chunk size
                overlap=min(settings.chunk_overlap, 300)     # Limit overlap
            )
            
            # Limit number of chunks per document
            if len(chunk_dicts) > 50:
                print(f"   ⚠️  Too many chunks ({len(chunk_dicts)}), limiting to 50")
                chunk_dicts = chunk_dicts[:50]
            
            chunks = [chunk['content'] for chunk in chunk_dicts]
            print(f"   Chunks: {len(chunks)}")
            
        except MemoryError:
            print(f"   ⚠️  Memory error during chunking, using simple split")
            # Fallback to simple chunking
            chunk_size = 1000
            chunks = []
            for i in range(0, len(content), chunk_size):
                chunk = content[i:i+chunk_size]
                if chunk.strip():
                    chunks.append(chunk.strip())
                if len(chunks) >= 50:  # Limit to 50 chunks
                    break
            print(f"   Chunks (fallback): {len(chunks)}")
        
        # Process each chunk with rate limiting
        points = []
        for i, chunk in enumerate(chunks):
            try:
                # Generate embedding
                embedding = await embedding_service.generate_embedding(chunk)
                
                # Create unique ID
                chunk_id = hashlib.md5(
                    f"{doc['source_file']}_{i}".encode()
                ).hexdigest()
                
                # Create point
                point = PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        'content': chunk,
                        'source_file': doc['source_file'],
                        'module': doc['module'],
                        'chapter': doc['chapter'],
                        'chunk_index': i,
                        'total_chunks': len(chunks)
                    }
                )
                points.append(point)
                
                print(f"   ✓ Chunk {i+1}/{len(chunks)}", end='\r')
                
                # Rate limiting - wait every 5 chunks
                if (i + 1) % 5 == 0:
                    await asyncio.sleep(1)
                    
            except Exception as e:
                print(f"\n   ⚠️  Error processing chunk {i+1}: {e}")
                continue
        
        # Batch upsert to Qdrant (only if we have points)
        if not points:
            print(f"   ⚠️  No valid chunks to upload")
            return 0
            
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        
        client.upsert(
            collection_name=settings.qdrant_collection_name,
            points=points
        )
        
        print(f"\n   ✓ Ingested {len(points)} chunks")
        return len(points)
        
    except Exception as e:
        print(f"❌ Error ingesting {doc['chapter']}: {e}")
        import traceback
        traceback.print_exc()
        return 0

async def create_collection_if_not_exists():
    """Create Qdrant collection if it doesn't exist"""
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        
        collections = client.get_collections().collections
        collection_names = [c.name for c in collections]
        
        if settings.qdrant_collection_name not in collection_names:
            print(f"Creating collection: {settings.qdrant_collection_name}")
            
            client.create_collection(
                collection_name=settings.qdrant_collection_name,
                vectors_config=VectorParams(
                    size=768,  # text-embedding-004 dimension
                    distance=Distance.COSINE
                )
            )
            print("✓ Collection created")
        else:
            print(f"✓ Collection exists: {settings.qdrant_collection_name}")
            
    except Exception as e:
        print(f"❌ Error creating collection: {e}")
        raise

async def main():
    print("="*70)
    print("DOCUMENT INGESTION SCRIPT")
    print("="*70)
    
    # Find docs directory
    global DOCS_DIR
    current_dir = Path(__file__).parent
    
    # Try different possible locations
    possible_paths = [
        current_dir.parent / "docusaurus" / "docs",
        current_dir.parent.parent / "docusaurus" / "docs",
        Path("C:/new/physical-ai-robotics-textbook/docusaurus/docs"),
        Path("C:/new - Copy/physical-ai-robotics-textbook/docusaurus/docs"),
    ]
    
    DOCS_DIR = None
    for path in possible_paths:
        if path.exists():
            DOCS_DIR = path
            break
    
    if not DOCS_DIR:
        print("❌ Could not find docusaurus/docs directory!")
        print("Please update DOCS_DIR in the script manually")
        return
    
    print(f"\n📁 Docs directory: {DOCS_DIR}")
    
    # Create collection
    await create_collection_if_not_exists()
    
    # Find all markdown files
    md_files = list(DOCS_DIR.rglob("*.md"))
    print(f"\n📚 Found {len(md_files)} markdown files")
    
    if len(md_files) == 0:
        print("❌ No markdown files found!")
        return
    
    # Read all documents
    print("\n📖 Reading documents...")
    documents = []
    for file_path in md_files:
        doc = await read_markdown_file(file_path)
        if doc and doc['content'].strip():
            documents.append(doc)
    
    print(f"✓ Read {len(documents)} valid documents")
    
    # Ingest all documents
    print("\n🚀 Starting ingestion...")
    total_chunks = 0
    
    for i, doc in enumerate(documents, 1):
        chunks = await ingest_document(doc, i)
        total_chunks += chunks
        await asyncio.sleep(0.1)  # Rate limiting
    
    # Final stats
    print("\n" + "="*70)
    print("INGESTION COMPLETE!")
    print("="*70)
    print(f"📄 Documents processed: {len(documents)}")
    print(f"📦 Total chunks created: {total_chunks}")
    print(f"✓ Collection: {settings.qdrant_collection_name}")
    print("="*70)
    
    # Verify
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )
    info = client.get_collection(settings.qdrant_collection_name)
    print(f"\n✓ Final count in Qdrant: {info.points_count} points")

if __name__ == "__main__":
    asyncio.run(main())