import os
import sys
from pathlib import Path
import asyncio
import unittest
from unittest.mock import patch, MagicMock, AsyncMock

# Set mock environment variables BEFORE other imports
os.environ["QDRANT_URL"] = "http://localhost:6333"
os.environ["QDRANT_API_KEY"] = "test_key"
os.environ["GOOGLE_API_KEY"] = "test_key"
os.environ["DATABASE_URL"] = "postgresql+asyncpg://test:test@localhost/test"


# Add the backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Now we can import the script, as the environment is set
from scripts import ingest_documents


class TestIngestionScript(unittest.IsolatedAsyncioTestCase):

    def setUp(self):
        """Set up a temporary directory and mock files for testing."""
        self.test_dir = Path("test_docs")
        self.test_dir.mkdir(exist_ok=True)

        self.mock_files = {
            "doc1.md": "## Module 1\n\n### Chapter 1\n\nContent of document 1.",
            "doc2.md": "## Module 1\n\n### Chapter 2\n\nContent of document 2, which is a bit longer.",
            "not_a_doc.txt": "This file should be ignored.",
        }

        for filename, content in self.mock_files.items():
            with open(self.test_dir / filename, "w") as f:
                f.write(content)

    def tearDown(self):
        """Clean up the temporary directory and files."""
        for filename in self.mock_files.keys():
            (self.test_dir / filename).unlink()
        self.test_dir.rmdir()

    @patch("qdrant_client.AsyncQdrantClient", autospec=True)
    @patch("scripts.ingest_documents.embedding_service")
    @patch("scripts.ingest_documents.markdown_parser", autospec=True)
    @patch("scripts.ingest_documents.text_processor")
    async def test_ingest_directory_flow(
        self,
        mock_text_processor,
        mock_markdown_parser,
        mock_embedding_service,
        MockAsyncQdrantClientClass,
    ):
        """Test the main directory ingestion flow with mocked services."""
        mock_client_instance = MockAsyncQdrantClientClass.return_value
        mock_client_instance.upsert = AsyncMock(return_value="mock-uuid")
        mock_client_instance.get_collections = AsyncMock(
            return_value=MagicMock(collections=[])
        )  # Mock empty collections
        mock_client_instance.create_collection = AsyncMock(return_value=True)
        mock_client_instance.create_payload_index = AsyncMock(return_value=True)
        mock_embedding_service.generate_embedding = AsyncMock(
            return_value=[0.1, 0.2, 0.3]
        )

        # Call init_collection before ingestion to ensure mocks are ready
        await qdrant_service.init_collection()

        # --- Run the function to be tested ---
        summary = await ingest_documents.ingest_directory(str(self.test_dir))

        # --- Assertions ---

        # Verify it found and processed only the markdown files
        self.assertEqual(summary["files_found"], 2)
        self.assertEqual(summary["files_processed"], 2)
        self.assertEqual(summary["successful"], 2)
        self.assertEqual(summary["failed"], 0)

        # Check that init_collection was called
        mock_client_instance.get_collections.assert_awaited_once()
        mock_client_instance.create_collection.assert_awaited_once()
        self.assertEqual(mock_client_instance.create_payload_index.call_count, 3)

        # Check calls to markdown_parser
        self.assertEqual(mock_markdown_parser.parse_markdown_file.call_count, 2)
        mock_markdown_parser.parse_markdown_file.assert_any_call(
            str(self.test_dir / "doc1.md")
        )
        mock_markdown_parser.parse_markdown_file.assert_any_call(
            str(self.test_dir / "doc2.md")
        )

        # Check calls to text_processor
        self.assertEqual(mock_text_processor.chunk_text.call_count, 2)
        self.assertEqual(
            mock_text_processor.preprocess_for_embedding.call_count, 2
        )  # 2 files * 1 chunk each

        # Check calls to embedding_service
        self.assertEqual(mock_embedding_service.generate_embedding.call_count, 2)
        mock_embedding_service.generate_embedding.assert_called_with("processed chunk")

        # Check calls to qdrant_service client's upsert method
        self.assertEqual(mock_client_instance.upsert.call_count, 2)

        # Inspect the first call to upsert
        first_call_args, first_call_kwargs = mock_client_instance.upsert.call_args_list[
            0
        ]

        # The first positional argument of upsert is a list of PointStructs
        first_point_struct = first_call_args[0][0]

        self.assertEqual(len(first_call_args[0]), 1)  # Only one point in the list
        self.assertIsInstance(first_point_struct, MagicMock)  # It's a mock

        self.assertEqual(first_point_struct.payload["content"], "processed chunk")
        self.assertEqual(first_point_struct.vector, [0.1, 0.2, 0.3])
        self.assertEqual(first_point_struct.payload["module"], "Module 1")
        self.assertEqual(first_point_struct.payload["chapter"], "Chapter 1")
        self.assertEqual(
            first_point_struct.payload["source_file"], str(self.test_dir / "doc1.md")
        )
        self.assertEqual(first_point_struct.payload["chunk_index"], 0)
        self.assertEqual(
            first_point_struct.payload["metadata"]["original_length"], len("chunk 1")
        )
        self.assertEqual(
            first_point_struct.payload["metadata"]["processed_length"],
            len("processed chunk"),
        )
        self.assertEqual(first_point_struct.payload["metadata"]["start_pos"], 0)
        self.assertEqual(first_point_struct.payload["metadata"]["end_pos"], 10)

    @patch("qdrant_client.AsyncQdrantClient", autospec=True)
    async def test_ingest_directory_force_refresh(self, MockAsyncQdrantClientClass):
        """Test the force_refresh logic."""
        # Mock other services to isolate the test to qdrant collection logic
        with patch("scripts.ingest_documents.embedding_service"), patch(
            "scripts.ingest_documents.markdown_parser"
        ), patch("scripts.ingest_documents.text_processor"):

            mock_client_instance = MockAsyncQdrantClientClass.return_value

            mock_client_instance.delete_collection = AsyncMock(return_value=True)
            mock_client_instance.get_collections = AsyncMock(
                return_value=MagicMock(collections=[])
            )
            mock_client_instance.create_collection = AsyncMock(return_value=True)
            mock_client_instance.create_payload_index = AsyncMock(return_value=True)

            await ingest_documents.ingest_directory(
                str(self.test_dir), force_refresh=True
            )

            # Verify that the collection is deleted and then initialized
            mock_client_instance.delete_collection.assert_awaited_once()
            mock_client_instance.get_collections.assert_awaited_once()
            mock_client_instance.create_collection.assert_awaited_once()
            self.assertEqual(mock_client_instance.create_payload_index.call_count, 3)

    @patch("scripts.ingest_documents.ingest_single_document", new_callable=AsyncMock)
    async def test_ingest_directory_error_handling(self, mock_ingest_single):
        """Test that the script handles and reports failures correctly."""

        mock_ingest_single.side_effect = [
            {"status": "success", "chunks_stored": 1},
            {"status": "error", "error": "Test error"},
        ]

        summary = await ingest_documents.ingest_directory(str(self.test_dir))

        self.assertEqual(summary["files_found"], 2)
        self.assertEqual(summary["successful"], 1)
        self.assertEqual(summary["failed"], 1)
        self.assertEqual(summary["total_chunks_created"], 1)
        self.assertEqual(len(summary["results"]), 2)
        self.assertEqual(summary["results"][1]["status"], "error")


if __name__ == "__main__":
    unittest.main(argv=["first-arg-is-ignored"], exit=False)
