#!/usr/bin/env python3
"""
Database setup script for the RAG Chatbot.
Creates the necessary tables in PostgreSQL for conversation management.
"""
import asyncio
import sys
import os
from pathlib import Path

# Add the backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.postgres_service import PostgresService
from app.config import settings


async def setup_database():
    """
    Set up the database tables for the RAG Chatbot.
    """
    print("Setting up database tables...")

    # Initialize the Postgres service
    postgres_service = PostgresService()

    try:
        # Create the tables
        await postgres_service.init_db()
        print("✓ Database tables created successfully!")

        # Verify tables were created by trying to create a test conversation
        print("\nTesting database connection...")
        test_conv = await postgres_service.create_conversation(
            title="Test Conversation"
        )
        print(f"✓ Test conversation created with ID: {test_conv['id']}")

        # Clean up test conversation
        await postgres_service.delete_conversation(test_conv["id"])
        print("✓ Test conversation cleaned up")

        print("\n🎉 Database setup completed successfully!")
        return True

    except Exception as e:
        print(f"❌ Error setting up database: {str(e)}")
        return False
    finally:
        await postgres_service.close()


async def verify_database():
    """
    Verify that the database is properly set up.
    """
    print("Verifying database setup...")

    postgres_service = PostgresService()

    try:
        # Try to get conversations (should work if tables exist)
        conversations = await postgres_service.get_conversations(limit=1)
        print("✓ Database connection verified")
        print(f"✓ Found {len(conversations)} existing conversations")
        return True

    except Exception as e:
        print(f"❌ Database verification failed: {str(e)}")
        return False
    finally:
        await postgres_service.close()


async def main():
    """
    Main function to run the database setup.
    """
    print("🚀 RAG Chatbot Database Setup Script")
    print("=" * 40)

    # Check if database URL is configured
    if not settings.database_url or "localhost" in settings.database_url:
        print("⚠️  WARNING: Using default/localhost database URL")
        print("   Please update your .env file with the correct NEON_DATABASE_URL")
        print(f"   Current URL: {settings.database_url}")
        print()

    action = (
        input("Do you want to (s)etup or (v)erify the database? (s/v): ")
        .lower()
        .strip()
    )

    if action == "s":
        success = await setup_database()
    elif action == "v":
        success = await verify_database()
    else:
        print("Invalid option. Use 's' for setup or 'v' for verify.")
        return 1

    if success:
        print("\n✅ Database setup/verification completed successfully!")
        return 0
    else:
        print("\n❌ Database setup/verification failed!")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
