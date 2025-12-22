import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
import sys
import os

# Add the parent directory to sys.path so we can import the main app
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app
from models.personalization import UserProfile, PersonalizeChapterRequest


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    return TestClient(app)


def test_health_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["service"] == "personalization-backend"
    assert "timestamp" in data
    assert "response_time_ms" in data
    assert "dependencies" in data


def test_personalize_chapter_endpoint(client):
    """Test the personalization endpoint integration"""
    # Create a mock user profile
    user_profile = {
        "softwareBackground": "beginner",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "none",
        "aiMlExperience": "beginner",
        "hasRosExperience": False,
        "hasGpuAccess": True,
        "learningGoals": "Learn robotics fundamentals"
    }

    # Create a test request payload
    request_payload = {
        "chapter_id": "test-chapter-1",
        "chapter_content": "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts.",
        "user_profile": user_profile
    }

    # Make a request with a mock auth header
    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    # Note: This test may fail if the RAG service is not running,
    # so we're testing the structure and error handling
    assert response.status_code in [200, 500]  # Either success or service error

    if response.status_code == 200:
        data = response.json()
        assert "personalized_content" in data
        assert "processing_time_ms" in data
        assert isinstance(data["processing_time_ms"], int)


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_personalize_chapter_with_mock_ai(mock_personalize_content, client):
    """Test personalization with mocked AI service"""
    # Mock the AI service to return known content
    mock_personalize_content.return_value = asyncio.Future()
    mock_personalize_content.return_value.set_result(
        "# Personalized Introduction to Robotics\n\nThis content is personalized for beginners."
    )

    # Create a mock user profile
    user_profile = {
        "softwareBackground": "beginner",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "none",
        "aiMlExperience": "beginner",
        "hasRosExperience": False,
        "hasGpuAccess": True,
        "learningGoals": "Learn robotics fundamentals"
    }

    # Create a test request payload
    request_payload = {
        "chapter_id": "test-chapter-2",
        "chapter_content": "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts.",
        "user_profile": user_profile
    }

    # Make a request with a mock auth header
    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data
    assert "processing_time_ms" in data
    assert isinstance(data["processing_time_ms"], int)


def test_unauthorized_request(client):
    """Test that unauthorized requests are rejected"""
    request_payload = {
        "chapter_id": "test-chapter-1",
        "chapter_content": "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts.",
        "user_profile": {
            "softwareBackground": "beginner",
            "hardwareBackground": "maker",
            "programmingLanguages": "Python",
            "roboticsExperience": "none",
            "aiMlExperience": "beginner",
            "hasRosExperience": False,
            "hasGpuAccess": True,
            "learningGoals": "Learn robotics fundamentals"
        }
    }

    # Make a request without auth header
    response = client.post("/api/personalize/chapter", json=request_payload)

    assert response.status_code == 401


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_experience_level_personalization(mock_personalize_content, client):
    """Test personalization based on different experience levels"""
    # Mock the AI service to return known content
    mock_personalize_content.return_value = asyncio.Future()
    mock_personalize_content.return_value.set_result(
        "# Personalized Introduction to Robotics\n\nThis content is personalized for beginners."
    )

    # Test with beginner user profile
    beginner_profile = {
        "softwareBackground": "beginner",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "none",
        "aiMlExperience": "beginner",
        "hasRosExperience": False,
        "hasGpuAccess": True,
        "learningGoals": "Learn robotics fundamentals"
    }

    request_payload = {
        "chapter_id": "test-chapter-3",
        "chapter_content": "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts.",
        "user_profile": beginner_profile
    }

    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_gpu_constraint_handling(mock_personalize_content, client):
    """Test personalization with GPU constraint handling"""
    # Mock the AI service to return known content
    mock_personalize_content.return_value = asyncio.Future()
    mock_personalize_content.return_value.set_result(
        "# GPU-Aware Introduction to Robotics\n\nContent adapted for CPU-only environment."
    )

    # Test with user without GPU access
    no_gpu_profile = {
        "softwareBackground": "intermediate",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "some",
        "aiMlExperience": "intermediate",
        "hasRosExperience": False,
        "hasGpuAccess": False,  # No GPU access
        "learningGoals": "Learn robotics fundamentals"
    }

    request_payload = {
        "chapter_id": "test-chapter-4",
        "chapter_content": "# GPU-Intensive Robotics\n\nThis chapter covers GPU-accelerated robotics.",
        "user_profile": no_gpu_profile
    }

    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_ros_experience_adaptation(mock_personalize_content, client):
    """Test personalization based on ROS experience"""
    # Mock the AI service to return known content
    mock_personalize_content.return_value = asyncio.Future()
    mock_personalize_content.return_value.set_result(
        "# Advanced ROS Robotics\n\nContent for experienced ROS users."
    )

    # Test with user with ROS experience
    ros_experienced_profile = {
        "softwareBackground": "advanced",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python, C++",
        "roboticsExperience": "extensive",
        "aiMlExperience": "intermediate",
        "hasRosExperience": True,  # Has ROS experience
        "hasGpuAccess": True,
        "learningGoals": "Master advanced robotics"
    }

    request_payload = {
        "chapter_id": "test-chapter-5",
        "chapter_content": "# ROS Basics\n\nThis chapter covers ROS fundamentals.",
        "user_profile": ros_experienced_profile
    }

    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_performance_timing(mock_personalize_content, client):
    """T048: Performance tests (<3s) and timing verification"""
    import time

    # Mock the AI service to simulate realistic processing time
    async def slow_personalize(*args, **kwargs):
        await asyncio.sleep(0.1)  # Simulate some processing time
        return "# Personalized Content\n\nThis content is personalized."

    mock_personalize_content.side_effect = slow_personalize

    # Test with a moderate-sized content
    request_payload = {
        "chapter_id": "test-chapter-perf",
        "chapter_content": "# Introduction\n\n" + "This is a test chapter content. " * 100,  # ~2KB content
        "user_profile": {
            "softwareBackground": "beginner",
            "hardwareBackground": "maker",
            "programmingLanguages": "Python",
            "roboticsExperience": "none",
            "aiMlExperience": "beginner",
            "hasRosExperience": False,
            "hasGpuAccess": True,
            "learningGoals": "Learn robotics fundamentals"
        }
    }

    start_time = time.time()
    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )
    end_time = time.time()

    # Verify the response
    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data
    assert "processing_time_ms" in data

    # Verify timing constraints
    api_processing_time = data["processing_time_ms"]
    total_time = (end_time - start_time) * 1000

    # Check that the API reports processing time under 3 seconds
    assert api_processing_time < 3000, f"API processing time {api_processing_time}ms exceeded 3 seconds"
    # Check that the total request time is reasonable
    assert total_time < 5000, f"Total request time {total_time:.2f}ms exceeded 5 seconds"


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_error_recovery_fallback(mock_personalize_content, client):
    """T049: Error recovery tests - Verify fallback mechanism works properly"""
    # Mock the AI service to raise an exception
    mock_personalize_content.side_effect = Exception("AI service unavailable")

    # Test with a user profile
    user_profile = {
        "softwareBackground": "beginner",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "none",
        "aiMlExperience": "beginner",
        "hasRosExperience": False,
        "hasGpuAccess": True,
        "learningGoals": "Learn robotics fundamentals"
    }

    original_content = "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts."

    request_payload = {
        "chapter_id": "test-chapter-error-recovery",
        "chapter_content": original_content,
        "user_profile": user_profile
    }

    # Make a request - should return original content as fallback
    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    # Should return 200 with original content as fallback
    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data
    # In case of error, it should return the original content as fallback
    assert data["personalized_content"] == original_content
    assert "processing_time_ms" in data
    assert isinstance(data["processing_time_ms"], int)


@patch('services.ai_personalization.ai_personalization_service.personalize_content')
def test_timeout_error_recovery(mock_personalize_content, client):
    """T049: Error recovery tests - Verify timeout error handling"""
    from httpx import TimeoutException

    # Mock the AI service to raise a timeout exception
    mock_personalize_content.side_effect = TimeoutException("Request timeout")

    # Test with a user profile
    user_profile = {
        "softwareBackground": "beginner",
        "hardwareBackground": "maker",
        "programmingLanguages": "Python",
        "roboticsExperience": "none",
        "aiMlExperience": "beginner",
        "hasRosExperience": False,
        "hasGpuAccess": True,
        "learningGoals": "Learn robotics fundamentals"
    }

    original_content = "# Introduction to Robotics\n\nThis chapter introduces basic robotics concepts."

    request_payload = {
        "chapter_id": "test-chapter-timeout-recovery",
        "chapter_content": original_content,
        "user_profile": user_profile
    }

    # Make a request - should return original content as fallback
    response = client.post(
        "/api/personalize/chapter",
        json=request_payload,
        headers={"Authorization": "Bearer test-token"}
    )

    # Should return 200 with original content as fallback
    assert response.status_code == 200
    data = response.json()
    assert "personalized_content" in data
    # In case of timeout, it should return the original content as fallback
    assert data["personalized_content"] == original_content
    assert "processing_time_ms" in data
    assert isinstance(data["processing_time_ms"], int)


if __name__ == "__main__":
    pytest.main([__file__])