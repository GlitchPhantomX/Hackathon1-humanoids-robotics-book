"""
Tests for the conversation management functionality.
"""

import pytest
from unittest.mock import AsyncMock, patch
from fastapi.testclient import TestClient
from main import app
from app.models import ChatRequest, ChatResponse
from app.services.postgres_service import postgres_service
from app.services.agent_service import agent_service
from app.services.vector_search_service import vector_search_service
from app.services.qdrant_service import qdrant_service
from app.services.embedding_service import embedding_service
from datetime import datetime


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


class TestConversationManagement:

    @pytest.mark.asyncio
    async def test_create_conversation_and_chat(self):
        """Test creating a new conversation and exchanging messages."""
        # Mock the services to avoid external dependencies
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
                        "content": "Test content about robotics",
                        "module": "Module 1",
                        "chapter": "Chapter 1",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.85,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="This is a test response based on the context.",
                ):

                    client = TestClient(app)

                    # Send a chat request (this should create a new conversation)
                    response = client.post(
                        "/api/chat",
                        json={
                            "message": "What is ROS 2?",
                            "selected_text": None,
                            "user_id": "test_user_123",
                        },
                    )

                    assert response.status_code == 200
                    data = response.json()

                    # Check that the response has the expected structure
                    assert "response" in data
                    assert "conversation_id" in data
                    assert "sources" in data
                    assert len(data["sources"]) >= 0  # May have sources or not

                    # Store conversation ID for later use
                    conversation_id = data["conversation_id"]
                    assert conversation_id is not None
                    assert len(conversation_id) > 0

    @pytest.mark.asyncio
    async def test_conversation_persistence(self):
        """Test that conversations persist across requests."""
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
                        "content": "Test content about robotics",
                        "module": "Module 1",
                        "chapter": "Chapter 1",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.85,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="This is a test response based on the context.",
                ):

                    client = TestClient(app)

                    # Create first message in conversation
                    response1 = client.post(
                        "/api/chat",
                        json={"message": "What is ROS 2?", "selected_text": None},
                    )

                    assert response1.status_code == 200
                    data1 = response1.json()
                    conversation_id = data1["conversation_id"]

                    # Send follow-up message in same conversation
                    response2 = client.post(
                        "/api/chat",
                        json={
                            "message": "Can you explain more about it?",
                            "conversation_id": conversation_id,
                        },
                    )

                    assert response2.status_code == 200
                    data2 = response2.json()

                    # Verify conversation ID is the same
                    assert data2["conversation_id"] == conversation_id

    def test_get_conversation(self):
        """Test retrieving a specific conversation."""
        # Use TestClient to make a request to get a conversation
        client = TestClient(app)

        # First create a conversation to retrieve
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
                        "content": "Test content about robotics",
                        "module": "Module 1",
                        "chapter": "Chapter 1",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.85,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="This is a test response based on the context.",
                ):

                    response = client.post(
                        "/api/chat",
                        json={
                            "message": "What is machine learning?",
                            "selected_text": None,
                        },
                    )

                    assert response.status_code == 200
                    data = response.json()
                    conversation_id = data["conversation_id"]

                    # Now retrieve the conversation
                    get_response = client.get(f"/api/conversations/{conversation_id}")
                    assert get_response.status_code == 200

                    conv_data = get_response.json()
                    assert conv_data["id"] == conversation_id
                    assert "messages" in conv_data
                    assert len(conv_data["messages"]) >= 2  # user + assistant message

    def test_get_conversations_list(self):
        """Test retrieving a list of conversations."""
        client = TestClient(app)

        # Create a conversation first
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
                        "content": "Test content about robotics",
                        "module": "Module 1",
                        "chapter": "Chapter 1",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.85,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="This is a test response based on the context.",
                ):

                    response = client.post(
                        "/api/chat",
                        json={"message": "What is AI?", "selected_text": None},
                    )

                    assert response.status_code == 200

        # Get list of conversations
        get_response = client.get("/api/conversations")
        assert get_response.status_code == 200

        conversations = get_response.json()
        assert isinstance(conversations, list)
        assert len(conversations) >= 1

    @pytest.mark.asyncio
    async def test_store_source_citations(self):
        """Test that source citations are properly stored with assistant messages."""
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
                        "content": "ROS 2 is a robotics framework",
                        "module": "ROS 2 Fundamentals",
                        "chapter": "Introduction",
                        "source_file": "docs/ros2/intro.md",
                        "chunk_index": 1,
                        "relevance_score": 0.92,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="ROS 2 is a flexible framework for robotics development.",
                ):

                    client = TestClient(app)

                    response = client.post(
                        "/api/chat",
                        json={"message": "What is ROS 2?", "selected_text": None},
                    )

                    assert response.status_code == 200
                    data = response.json()

                    # Check that sources are returned in the response
                    assert len(data["sources"]) == 1
                    source = data["sources"][0]
                    assert source["source_file"] == "docs/ros2/intro.md"
                    assert source["module"] == "ROS 2 Fundamentals"
                    assert source["chapter"] == "Introduction"
                    assert source["relevance_score"] == 0.92

    def test_multiple_concurrent_conversations(self):
        """Test that multiple concurrent conversations work properly."""
        client = TestClient(app)

        conversations = []

        # Create multiple conversations
        for i in range(3):
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
                            "id": f"test_id_{i}",
                            "content": f"Test content {i}",
                            "module": f"Module {i}",
                            "chapter": f"Chapter {i}",
                            "source_file": f"test{i}.md",
                            "chunk_index": i,
                            "relevance_score": 0.8,
                            "metadata": {},
                        }
                    ],
                ):
                    with patch.object(
                        agent_service,
                        "generate_response",
                        return_value=f"This is response {i}",
                    ):

                        response = client.post(
                            "/api/chat",
                            json={
                                "message": f"Test message {i}",
                                "selected_text": None,
                            },
                        )

                        assert response.status_code == 200
                        data = response.json()
                        conversations.append(data["conversation_id"])

        # Verify all conversations have unique IDs
        assert len(set(conversations)) == 3  # All IDs should be unique

    def test_update_conversation_timestamp(self):
        """Test that conversation timestamp is updated on each new message."""
        client = TestClient(app)

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
                        "module": "Module 1",
                        "chapter": "Chapter 1",
                        "source_file": "test.md",
                        "chunk_index": 1,
                        "relevance_score": 0.8,
                        "metadata": {},
                    }
                ],
            ):
                with patch.object(
                    agent_service,
                    "generate_response",
                    return_value="This is a test response",
                ):

                    # Create initial conversation
                    response1 = client.post(
                        "/api/chat",
                        json={"message": "First message", "selected_text": None},
                    )
                    assert response1.status_code == 200
                    data1 = response1.json()
                    conversation_id = data1["conversation_id"]

                    # Check initial conversation details
                    conv_response = client.get(f"/api/conversations/{conversation_id}")
                    assert conv_response.status_code == 200
                    initial_conv = conv_response.json()
                    initial_updated_at = initial_conv["updated_at"]

                    # Add another message to update the timestamp
                    response2 = client.post(
                        "/api/chat",
                        json={
                            "message": "Second message",
                            "conversation_id": conversation_id,
                        },
                    )
                    assert response2.status_code == 200

                    # Check updated conversation details
                    conv_response2 = client.get(f"/api/conversations/{conversation_id}")
                    assert conv_response2.status_code == 200
                    updated_conv = conv_response2.json()
                    updated_at = updated_conv["updated_at"]

                    # The updated timestamp should be different from the original
                    assert updated_at != initial_updated_at
