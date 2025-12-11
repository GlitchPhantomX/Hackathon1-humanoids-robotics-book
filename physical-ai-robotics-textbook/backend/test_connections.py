import os
import asyncio
import google.generativeai as genai
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
import asyncpg  # pyright: ignore[reportMissingImports]
import ssl
import sys

# Load environment variables
load_dotenv()

async def test_postgres_connection():
    """Test Neon Postgres connection"""
    print("Testing Neon Postgres connection...")
    try:
        database_url = os.getenv("NEON_DATABASE_URL")
        if not database_url:
            print("[FAILED] NEON_DATABASE_URL not found in environment")
            return False

        # Remove quotes if present
        database_url = database_url.strip("'\"")

        # Test connection using asyncpg
        try:
            # Parse the connection string
            conn = await asyncpg.connect(dsn=database_url)
            await conn.fetchval("SELECT 1")  # Simple test query
            await conn.close()
            print("[OK] Neon Postgres connection successful")
            return True
        except Exception as e:
            print(f"[FAILED] Neon Postgres connection failed: {str(e)}")
            return False
    except Exception as e:
        print(f"[FAILED] Error testing Postgres connection: {str(e)}")
        return False


def test_qdrant_connection():
    """Test Qdrant Cloud connection"""
    print("Testing Qdrant Cloud connection...")
    try:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            print("[FAILED] QDRANT_URL or QDRANT_API_KEY not found in environment")
            return False

        # Create Qdrant client
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10
        )

        # Test connection by trying to list collections
        try:
            collections = client.get_collections()
            print("[OK] Qdrant Cloud connection successful")
            print(f"  Available collections: {[col.name for col in collections.collections]}")
            return True
        except Exception as e:
            print(f"[FAILED] Qdrant connection failed: {str(e)}")
            return False
    except Exception as e:
        print(f"[FAILED] Error testing Qdrant connection: {str(e)}")
        return False


def test_gemini_connection():
    """Test Google Gemini API connection"""
    print("Testing Google Gemini API connection...")
    try:
        api_key = os.getenv("GOOGLE_API_KEY")

        if not api_key:
            print("[FAILED] GOOGLE_API_KEY not found in environment")
            return False

        # Configure the API key
        genai.configure(api_key=api_key)

        # List available models as a basic test
        try:
            all_models = list(genai.list_models())  # Convert generator to list to avoid consumption issues
            print("[OK] Google Gemini API connection successful")
            model_names = [model.name for model in all_models if 'gemini' in model.name.lower()]
            print(f"  Available Gemini models: {model_names[:3]}")  # Show first 3

            # Test with a simple model - use available model
            available_models = [model.name for model in all_models if 'gemini' in model.name.lower()]
            if available_models:
                # Use the first available gemini model
                model_name = available_models[0].replace('models/', '')
                model = genai.GenerativeModel(model_name)
                response = model.generate_content("Hello, world! Just say hello back.")
                print(f"  Test response: {response.text[:50]}...")
                return True
            else:
                print("[FAILED] No available Gemini models found")
                return False
        except Exception as e:
            print(f"[FAILED] Google Gemini API test failed: {str(e)}")
            return False
    except Exception as e:
        print(f"[FAILED] Error testing Google Gemini API: {str(e)}")
        return False


def validate_env_variables():
    """Validate that all required environment variables are present"""
    print("Validating environment variables...")
    required_vars = ["QDRANT_URL", "QDRANT_API_KEY", "NEON_DATABASE_URL", "GOOGLE_API_KEY"]
    all_present = True

    for var in required_vars:
        value = os.getenv(var)
        if not value:
            print(f"[FAILED] Missing required environment variable: {var}")
            all_present = False
        else:
            # Show variable is present but mask sensitive parts
            if 'KEY' in var or 'PASSWORD' in var or 'TOKEN' in var:
                masked_value = value[:4] + '***' if len(value) > 4 else '***'
                print(f"[OK] {var}: {masked_value}")
            else:
                print(f"[OK] {var}: Present")

    if all_present:
        print("[OK] All required environment variables are present")
    return all_present


async def main():
    """Main function to run all tests"""
    print("Starting environment configuration tests...\n")

    # Validate environment variables
    env_valid = validate_env_variables()
    print()

    if not env_valid:
        print("❌ Environment validation failed. Please check your .env file.")
        return False

    # Run all connection tests
    results = {}

    # Test Qdrant
    results['qdrant'] = test_qdrant_connection()
    print()

    # Test Gemini
    results['gemini'] = test_gemini_connection()
    print()

    # Test Postgres
    results['postgres'] = await test_postgres_connection()
    print()

    # Summary
    print("Test Summary:")
    print(f"  Qdrant Cloud:     {'PASS' if results['qdrant'] else 'FAIL'}")
    print(f"  Google Gemini:    {'PASS' if results['gemini'] else 'FAIL'}")
    print(f"  Neon Postgres:    {'PASS' if results['postgres'] else 'FAIL'}")

    all_passed = all(results.values())
    print(f"\n{'All connections successful!' if all_passed else 'Some connections failed'}")

    return all_passed


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)