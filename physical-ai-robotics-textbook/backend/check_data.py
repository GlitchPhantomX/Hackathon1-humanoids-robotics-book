"""
Quick check script - Save as: check_data.py
Run: python check_data.py
"""

import asyncio
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.config import settings
from qdrant_client import QdrantClient

async def main():
    print("="*60)
    print("CHECKING QDRANT DATA")
    print("="*60)
    
    try:
        # Connect to Qdrant
        print("\n1. Connecting to Qdrant...")
        print(f"   URL: {settings.qdrant_url}")
        
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        print("   ✓ Connected!")
        
        # Check collection
        collection_name = settings.qdrant_collection_name  # Fixed!
        print(f"\n2. Checking collection: {collection_name}")
        
        try:
            info = client.get_collection(collection_name)
            print(f"   ✓ Collection found!")
            print(f"   Total documents: {info.points_count}")
            print(f"   Vector size: {info.config.params.vectors.size}")
            
            if info.points_count == 0:
                print("\n   ❌ NO DATA FOUND!")
                print("   You need to run ingestion script!")
                return
            
        except Exception as e:
            print(f"   ❌ Collection not found: {e}")
            print("   You need to create collection and add data!")
            return
        
        # Get sample data
        print("\n3. Getting sample documents...")
        result = client.scroll(
            collection_name=collection_name,
            limit=5,
            with_payload=True,
            with_vectors=False
        )
        
        points = result[0]
        print(f"   Found {len(points)} samples:\n")
        
        for i, point in enumerate(points, 1):
            p = point.payload
            print(f"   [{i}] Chapter: {p.get('chapter', 'N/A')}")
            print(f"       Module: {p.get('module', 'N/A')}")
            print(f"       File: {p.get('source_file', 'N/A')}")
            print(f"       Content: {p.get('content', '')[:80]}...")
            print()
        
        # Quick search test
        print("4. Testing search with 'physical AI'...")
        from app.services.embedding_service import embedding_service
        
        query_emb = await embedding_service.generate_query_embedding("physical AI")
        
        results = client.search(
            collection_name=collection_name,
            query_vector=query_emb,
            limit=3,
            score_threshold=0.0
        )
        
        print(f"   Found {len(results)} results:")
        for i, r in enumerate(results, 1):
            print(f"   [{i}] Score: {r.score:.3f}")
            print(f"       Chapter: {r.payload.get('chapter', 'N/A')}")
            print(f"       Content: {r.payload.get('content', '')[:60]}...")
            print()
        
        if len(results) == 0:
            print("   ❌ NO SEARCH RESULTS!")
        elif results[0].score < 0.3:
            print("   ⚠️  Low scores - might need better data")
        else:
            print("   ✓ Search working!")
        
        print("="*60)
        print("DONE!")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())