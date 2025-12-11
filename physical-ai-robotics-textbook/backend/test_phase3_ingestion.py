#!/usr/bin/env python3
"""
Test script to verify Phase 3 (Document Ingestion & Vectorization) functionality.
"""
import asyncio
import sys
from pathlib import Path
import os

# Add the backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent))

from app.services.embedding_service import embedding_service
from app.services.qdrant_service import qdrant_service
from app.utils.markdown_parser import markdown_parser
from app.utils.text_processing import text_processor
from app.config import settings


async def test_phase3_functionality():
    """Test all Phase 3 components for document ingestion and vectorization."""
    print("=" * 60)
    print("Testing Phase 3: Document Ingestion & Vectorization")
    print("=" * 60)

    # Test 1: Configuration Check
    print("\n1. Checking configuration...")
    print(f"   Google API Key: {'SET' if settings.google_api_key else 'NOT SET'}")
    print(f"   Qdrant URL: {'SET' if settings.qdrant_url else 'NOT SET'}")
    print(f"   Qdrant API Key: {'SET' if settings.qdrant_api_key else 'NOT SET'}")
    print(f"   Collection Name: {settings.qdrant_collection_name}")
    print(f"   Chunk Size: {settings.chunk_size} chars")
    print(f"   Chunk Overlap: {settings.chunk_overlap} chars")
    
    if not settings.google_api_key or not settings.qdrant_url or not settings.qdrant_api_key:
        print("   ❌ CRITICAL: Missing required configuration values")
        return False
    else:
        print("   [OK] Configuration check passed")

    # Test 2: Service Initialization
    print("\n2. Testing service initialization...")
    try:
        # Test embedding service
        test_text = "This is a test for embedding service."
        embedding = await embedding_service.generate_embedding(test_text)
        print(f"   [OK] Embedding service: Generated {len(embedding)}-dimensional embedding")

        # Test Qdrant service - initialize collection
        qdrant_service.init_collection()
        print("   [OK] Qdrant service: Collection initialized")

    except Exception as e:
        print(f"   [ERROR] Service initialization failed: {e}")
        return False

    # Test 3: Markdown Parsing
    print("\n3. Testing markdown parsing...")
    try:
        # Look for any markdown file to test with
        docusaurus_docs_path = Path("../docusaurus/docs")
        if docusaurus_docs_path.exists():
            sample_files = list(docusaurus_docs_path.rglob("*.md"))
            if sample_files:
                sample_file = sample_files[0]
                parsed_doc = markdown_parser.parse_markdown_file(str(sample_file))

                print(f"   [OK] Parsed sample file: {sample_file.name}")
                print(f"      Module: {parsed_doc.get('module', 'N/A')}")
                print(f"      Chapter: {parsed_doc.get('chapter', 'N/A')}")
                print(f"      Content length: {len(parsed_doc.get('content', ''))} chars")
            else:
                print("   [WARN] No markdown files found in docusaurus/docs for testing")
        else:
            print("   [WARN] docusaurus/docs directory not found")

    except Exception as e:
        print(f"   [ERROR] Markdown parsing failed: {e}")
        return False

    # Test 4: Text Chunking
    print("\n4. Testing text chunking functionality...")
    try:
        sample_text = "This is a sample text to test the chunking functionality. " * 100
        chunks = text_processor.chunk_text(sample_text)

        print(f"   [OK] Chunked text into {len(chunks)} chunks")
        if chunks:
            print(f"      First chunk: {len(chunks[0]['content'])} chars")
            print(f"      Chunk 0 index: {chunks[0]['chunk_index']}")
            print(f"      Chunk 0 length: {chunks[0]['length']}")

    except Exception as e:
        print(f"   [ERROR] Text chunking failed: {e}")
        return False

    # Test 5: Full Ingestion Pipeline
    print("\n5. Testing full ingestion pipeline...")
    try:
        # Create a temporary test document
        test_doc_path = Path("temp_test_doc.md")
        with open(test_doc_path, "w", encoding="utf-8") as f:
            f.write("# Test Document\n\nThis is a test document for the RAG chatbot.\n\n## Section 1\n\nHere's some content to test the ingestion pipeline. This content will be chunked, embedded, and stored in Qdrant.\n\n## Section 2\n\nMore content for testing purposes. The system should properly extract metadata, chunk the text, generate embeddings, and store everything in the vector database.")

        # Parse the document
        parsed_doc = markdown_parser.parse_markdown_file(str(test_doc_path))
        print(f"   [OK] Document parsed: {parsed_doc['module']}/{parsed_doc['chapter']}")

        # Chunk the content
        chunks = text_processor.chunk_text(parsed_doc['content'])
        print(f"   [OK] Content chunked: {len(chunks)} chunks created")

        # Process the first chunk to test embedding and storage
        if chunks:
            test_chunk = chunks[0]
            processed_content = text_processor.preprocess_for_embedding(test_chunk['content'])

            # Generate embedding
            embedding = await embedding_service.generate_embedding(processed_content)
            print(f"   [OK] Embedding generated: {len(embedding)}-dimensional vector")

            # Store in Qdrant
            doc_id = qdrant_service.store_embedding(
                content=processed_content,
                embedding=embedding,
                module=parsed_doc['module'],
                chapter=parsed_doc['chapter'],
                source_file=parsed_doc['file_path'],
                chunk_index=test_chunk['chunk_index']
            )
            print(f"   [OK] Document stored in Qdrant with ID: {doc_id[:8]}...")

        # Clean up temporary file
        if test_doc_path.exists():
            test_doc_path.unlink()

    except Exception as e:
        print(f"   [ERROR] Full ingestion pipeline failed: {e}")
        return False

    # Test 6: Collection Validation
    print("\n6. Testing collection validation...")
    try:
        # Check collection info
        collection_info = qdrant_service.get_collection_info()
        if "error" not in collection_info:
            doc_count = qdrant_service.count_documents()
            print(f"   [OK] Collection: {collection_info['name']}")
            print(f"   [OK] Document count: {doc_count}")
        else:
            print(f"   [ERROR] Collection validation failed: {collection_info.get('error')}")

    except Exception as e:
        print(f"   [ERROR] Collection validation failed: {e}")
        return False

    # Test 7: Search Capability (to verify embeddings are working)
    print("\n7. Testing search capability...")
    try:
        # Generate a query embedding
        query_text = "test document content"
        query_embedding = await embedding_service.generate_query_embedding(query_text)

        # Search for similar documents
        results = qdrant_service.search_similar(query_embedding, top_k=3)
        print(f"   [OK] Search successful: found {len(results)} results")

        if results:
            print(f"      Top result: {results[0]['module']}/{results[0]['chapter']}")
            print(f"      Relevance score: {results[0]['relevance_score']:.3f}")

    except Exception as e:
        print(f"   [ERROR] Search capability test failed: {e}")
        return False

    print("\n" + "=" * 60)
    print("Phase 3 Testing Complete: All components working correctly!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    success = asyncio.run(test_phase3_functionality())
    if success:
        print("\n[SUCCESS] Phase 3 implementation verification: PASSED")
        sys.exit(0)
    else:
        print("\n[FAILED] Phase 3 implementation verification: FAILED")
        sys.exit(1)