"""
Test suite for authentication-based visibility and access control
"""
import pytest
import jwt
from datetime import datetime, timedelta
from .auth import SessionValidator
from .schemas import UserSession


class TestAuthBasedVisibilityAndAccess:
    """
    Test authentication-based visibility and access control
    """

    def setup_method(self):
        """Setup test fixtures"""
        self.validator = SessionValidator()
        self.test_user_id = "test_user_123"

    def test_valid_token_authentication(self):
        """Test that valid tokens are properly authenticated"""
        # Create a valid token
        token = self.validator.create_token(self.test_user_id)

        # Validate the session
        user_session = self.validator.validate_session(token)

        # Assert the session is valid
        assert user_session is not None
        assert user_session.authenticated is True
        assert user_session.user_id == self.test_user_id

    def test_invalid_token_rejection(self):
        """Test that invalid tokens are rejected"""
        # Test with an invalid token
        invalid_token = "invalid.token.here"

        # Validate the session
        user_session = self.validator.validate_session(invalid_token)

        # Assert the session is not valid
        assert user_session is None

    def test_expired_token_rejection(self):
        """Test that expired tokens are rejected"""
        # Create an expired token manually
        expired_payload = {
            "user_id": self.test_user_id,
            "exp": datetime.utcnow().timestamp() - 3600,  # Expired 1 hour ago
            "iat": datetime.utcnow().timestamp()
        }
        expired_token = jwt.encode(
            expired_payload,
            self.validator.secret,
            algorithm=self.validator.algorithm
        )

        # Validate the session
        user_session = self.validator.validate_session(expired_token)

        # Assert the session is not valid
        assert user_session is None

    def test_user_session_validation(self):
        """Test that user session validation works correctly"""
        # Create a valid token
        token = self.validator.create_token(self.test_user_id)

        # Validate the specific user session
        is_valid = self.validator.validate_user_session(self.test_user_id, token)

        # Assert the session is valid
        assert is_valid is True

    def test_user_session_validation_wrong_user(self):
        """Test that user session validation fails for wrong user"""
        # Create a valid token for test user
        token = self.validator.create_token(self.test_user_id)

        # Try to validate with different user ID
        is_valid = self.validator.validate_user_session("different_user", token)

        # Assert the session is not valid for different user
        assert is_valid is False

    def test_no_token_authentication_failure(self):
        """Test that empty/missing tokens are rejected"""
        # Test with empty token
        user_session = self.validator.validate_session("")

        # Assert the session is not valid
        assert user_session is None

    def test_token_user_id_extraction(self):
        """Test that user ID can be extracted from token"""
        # Create a valid token
        token = self.validator.create_token(self.test_user_id)

        # Extract user ID from token
        extracted_user_id = self.validator.get_user_id_from_token(token)

        # Assert the user ID matches
        assert extracted_user_id == self.test_user_id


# Run tests if this file is executed directly
if __name__ == "__main__":
    test_instance = TestAuthBasedVisibilityAndAccess()
    test_instance.setup_method()

    # Run all tests
    test_instance.test_valid_token_authentication()
    test_instance.test_invalid_token_rejection()
    test_instance.test_expired_token_rejection()
    test_instance.test_user_session_validation()
    test_instance.test_user_session_validation_wrong_user()
    test_instance.test_no_token_authentication_failure()
    test_instance.test_token_user_id_extraction()

    print("All authentication tests passed!")