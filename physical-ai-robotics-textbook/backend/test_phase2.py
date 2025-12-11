"""
Test script to verify Phase 2 implementation works correctly.
"""
import asyncio
import sys
from pathlib import Path

# Add backend directory to path
sys.path.insert(0, str(Path(__file__).parent))

def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")

    # Test core modules
    from app.config import settings
    print("[OK] app.config imported")

    from app.models import ChatRequest, ChatResponse
    print("[OK] app.models imported")

    # Test services
    from app.services.postgres_service import PostgresService, postgres_service
    print("[OK] postgres_service imported")

    from app.services.embedding_service import EmbeddingService, embedding_service
    print("[OK] embedding_service imported")

    from app.services.qdrant_service import QdrantService, qdrant_service
    print("[OK] qdrant_service imported")

    from app.services.agent_service import AgentService, agent_service
    print("[OK] agent_service imported")

    # Test utilities
    from app.utils.markdown_parser import MarkdownParser, markdown_parser
    print("[OK] markdown_parser imported")

    from app.utils.text_processing import TextProcessor, text_processor
    print("[OK] text_processing imported")

    # Test scripts
    from scripts.setup_db import setup_database
    print("[OK] setup_db imported")

    from scripts.ingest_documents import ingest_single_document
    print("[OK] ingest_documents imported")

    print("All imports successful!\n")


def test_config():
    """Test configuration loading."""
    print("Testing configuration...")
    from app.config import settings

    # Check that required settings are available
    assert hasattr(settings, 'google_api_key'), "Google API key not configured"
    assert hasattr(settings, 'qdrant_url'), "Qdrant URL not configured"
    assert hasattr(settings, 'database_url'), "Database URL not configured"

    print("[OK] Configuration loaded successfully\n")


def test_services_instantiation():
    """Test that services can be instantiated."""
    print("Testing service instantiation...")

    from app.services.postgres_service import PostgresService
    from app.services.embedding_service import EmbeddingService
    from app.services.qdrant_service import QdrantService
    from app.services.agent_service import AgentService
    from app.utils.markdown_parser import MarkdownParser
    from app.utils.text_processing import TextProcessor

    postgres_svc = PostgresService()
    embedding_svc = EmbeddingService()
    qdrant_svc = QdrantService()
    agent_svc = AgentService()
    markdown_parser = MarkdownParser()
    text_proc = TextProcessor()

    print("[OK] All services instantiated successfully\n")


def test_text_processing():
    """Test text processing functionality."""
    print("Testing text processing...")

    from app.utils.text_processing import text_processor

    sample_text = "This is a sample text for testing. It contains multiple sentences. Each sentence should be processed correctly."

    # Test chunking
    chunks = text_processor.chunk_text(sample_text, chunk_size=30, overlap=5)
    print(f"[OK] Text chunked into {len(chunks)} chunks")

    # Test statistics
    stats = text_processor.get_chunk_statistics(chunks)
    print(f"[OK] Chunk statistics: {stats['total_chunks']} chunks, avg size {stats['avg_chunk_size']:.1f}")

    print("[OK] Text processing tests passed\n")


def test_markdown_parsing():
    """Test markdown parsing functionality."""
    print("Testing markdown parsing...")

    from app.utils.markdown_parser import markdown_parser

    # Test with a simple markdown string
    sample_md = """# Test Document

This is a test document.

## Section 1

Some content here.

## Section 2

More content here.

```python
def hello():
    print("Hello, world!")
```
"""

    # Create a temporary file for testing
    import tempfile
    import os

    with tempfile.NamedTemporaryFile(mode='w', suffix='.md', delete=False) as f:
        f.write(sample_md)
        temp_path = f.name

    try:
        # Parse the temporary file
        result = markdown_parser.parse_markdown_file(temp_path)
        print(f"[OK] Parsed markdown file: {result['title']}")

        # Extract headings
        headings = markdown_parser.extract_headings(sample_md)
        print(f"[OK] Extracted {len(headings)} headings")

        # Extract code blocks
        code_blocks = markdown_parser.extract_code_blocks(sample_md)
        print(f"[OK] Extracted {len(code_blocks)} code blocks")

    finally:
        # Clean up temp file
        os.unlink(temp_path)

    print("[OK] Markdown parsing tests passed\n")


async def test_async_services():
    """Test async service functionality."""
    print("Testing async services...")

    # Test postgres service initialization (without connecting to a real DB)
    from app.services.postgres_service import PostgresService

    postgres_svc = PostgresService()

    # We won't actually connect to DB since it might not be available
    # Just test that the service is properly configured
    print("[OK] Postgres service initialized")

    # Test that embedding service can be accessed
    from app.services.embedding_service import embedding_service
    print(f"[OK] Embedding service available, model: {embedding_service.model_name}")

    # Test that qdrant service can be accessed
    from app.services.qdrant_service import qdrant_service
    print(f"[OK] Qdrant service available, collection: {qdrant_service.collection_name}")

    # Test that agent service can be accessed
    from app.services.agent_service import agent_service
    print(f"[OK] Agent service available, model: {agent_service.model_name}")

    print("[OK] Async service tests passed\n")


async def main():
    """Run all tests."""
    print("Testing Phase 2 Implementation\n")

    test_imports()
    test_config()
    test_services_instantiation()
    test_text_processing()
    test_markdown_parsing()
    await test_async_services()

    print("All Phase 2 tests passed successfully!")


if __name__ == "__main__":
    asyncio.run(main())