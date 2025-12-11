"""
Tests for the vector search service functionality.
"""

import pytest
from unittest.mock import AsyncMock, patch
from app.services.vector_search_service import vector_search_service
from app.services.embedding_service import embedding_service
from app.services.qdrant_service import qdrant_service


class TestVectorSearchService:

    @pytest.mark.asyncio
    async def test_search_valid_query(self):
        """Test that the search function works with a valid query."""
        # Mock the embedding service to return a sample embedding
        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):  # 768-dim vector
            with patch.object(
                qdrant_service,
                "search_similar",
                return_value=[
                    {
                        "id": "test_id",
                        "content": "Test content",
                        "module": "Test Module",
                        "chapter": "Test Chapter",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.8,
                        "metadata": {},
                    }
                ],
            ):

                results = await vector_search_service.search("What is test?")

                assert len(results) == 1
                assert results[0]["content"] == "Test content"
                assert results[0]["relevance_score"] == 0.8
                assert results[0]["module"] == "Test Module"

    @pytest.mark.asyncio
    async def test_search_with_selected_text(self):
        """Test that the search function works with selected text."""
        # Mock the services
        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):
            with patch.object(
                qdrant_service,
                "search_similar",
                return_value=[
                    {
                        "id": "test_id",
                        "content": "Test content with selected text",
                        "module": "Test Module",
                        "chapter": "Test Chapter",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.85,
                        "metadata": {},
                    }
                ],
            ):

                results = await vector_search_service.search_with_selected_text(
                    query="What is this?", selected_text="This is selected text"
                )

                assert len(results) == 1
                assert results[0]["relevance_score"] == 0.85
                assert "selected" in results[0]["content"].lower()

    @pytest.mark.asyncio
    async def test_search_configurable_top_k(self):
        """Test that the search function respects the top_k parameter."""
        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):
            # Mock search_similar to return more results than top_k
            mock_results = []
            for i in range(10):
                mock_results.append(
                    {
                        "id": f"test_id_{i}",
                        "content": f"Test content {i}",
                        "module": "Test Module",
                        "chapter": "Test Chapter",
                        "source_file": "test.md",
                        "chunk_index": i,
                        "relevance_score": 0.9 - (i * 0.05),  # Decreasing scores
                        "metadata": {},
                    }
                )

            with patch.object(
                qdrant_service, "search_similar", return_value=mock_results
            ):

                # Request only 3 results
                results = await vector_search_service.search("What is test?", top_k=3)

                assert len(results) == 3
                # Ensure the results are sorted by relevance score (highest first)
                scores = [r["relevance_score"] for r in results]
                assert scores == sorted(scores, reverse=True)

    @pytest.mark.asyncio
    async def test_search_with_filters(self):
        """Test that the search function applies filters correctly."""
        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):
            with patch.object(
                qdrant_service,
                "search_similar",
                return_value=[
                    {
                        "id": "test_id",
                        "content": "Test content",
                        "module": "Robotics",
                        "chapter": "Kinematics",
                        "source_file": "robotics/kinematics.md",
                        "chunk_index": 1,
                        "relevance_score": 0.8,
                        "metadata": {},
                    }
                ],
            ):

                results = await vector_search_service.search_by_module_or_chapter(
                    "What is kinematics?", module="Robotics", chapter="Kinematics"
                )

                assert len(results) == 1
                assert results[0]["module"] == "Robotics"
                assert results[0]["chapter"] == "Kinematics"

    @pytest.mark.asyncio
    async def test_validate_search_query_short(self):
        """Test that short queries are rejected."""
        assert await vector_search_service.validate_search_query("") == False
        assert await vector_search_service.validate_search_query("a") == False
        assert await vector_search_service.validate_search_query("ab") == False
        assert (
            await vector_search_service.validate_search_query("abc") == True
        )  # Minimum length

    @pytest.mark.asyncio
    async def test_validate_search_query_long(self):
        """Test that very long queries are rejected."""
        long_query = "a" * 501  # More than 500 characters
        assert await vector_search_service.validate_search_query(long_query) == False

        valid_long_query = "a" * 500  # Exactly 500 characters
        assert (
            await vector_search_service.validate_search_query(valid_long_query) == True
        )

    @pytest.mark.asyncio
    async def test_validate_search_query_nonsensical(self):
        """Test that nonsensical queries are rejected."""
        # Mostly special characters with very little text
        nonsensical = "!@#$%^&*()_+=-{}[]|\\:;\"'<>?,./" * 5
        assert await vector_search_service.validate_search_query(nonsensical) == False

        # Reasonable query with some special characters is allowed
        reasonable = "What is ROS 2? Explain the benefits & drawbacks."
        assert await vector_search_service.validate_search_query(reasonable) == True

    @pytest.mark.asyncio
    async def test_search_empty_results(self):
        """Test behavior when no results are found."""
        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):
            with patch.object(
                qdrant_service, "search_similar", return_value=[]
            ):  # No results

                results = await vector_search_service.search("Unfindable query")

                assert len(results) == 0

    @pytest.mark.asyncio
    async def test_relevance_score_range(self):
        """Test that relevance scores are properly normalized to 0-1 range."""
        test_scores = [-0.5, 0.0, 0.5, 1.0, 1.5]
        expected_range = (0.0, 1.0)

        with patch.object(
            embedding_service,
            "generate_query_embedding",
            return_value=[0.1, 0.2, 0.3] * 256,
        ):
            # Create mock results with various scores
            mock_results = []
            for i, score in enumerate(test_scores):
                mock_results.append(
                    {
                        "id": f"test_id_{i}",
                        "content": f"Test content {i}",
                        "module": "Test Module",
                        "chapter": "Test Chapter",
                        "source_file": "test.md",
                        "chunk_index": i,
                        "relevance_score": score,
                        "metadata": {},
                    }
                )

            with patch.object(
                qdrant_service, "search_similar", return_value=mock_results
            ):

                results = await vector_search_service.search("Test query")

                for result in results:
                    score = result["relevance_score"]
                    assert (
                        expected_range[0] <= score <= expected_range[1]
                    ), f"Score {score} is not in range {expected_range}"

    @pytest.mark.asyncio
    async def test_get_search_statistics(self):
        """Test that search statistics are returned correctly."""
        mock_collection_info = {
            "point_count": 100,
            "vector_size": 768,
            "name": "physical_ai_textbook",
            "status": "healthy",
        }

        with patch.object(
            qdrant_service, "get_collection_info", return_value=mock_collection_info
        ):

            stats = await vector_search_service.get_search_statistics()

            assert stats["total_documents"] == 100
            assert stats["vector_size"] == 768
            assert stats["collection_name"] == "physical_ai_textbook"
            assert stats["status"] == "healthy"
