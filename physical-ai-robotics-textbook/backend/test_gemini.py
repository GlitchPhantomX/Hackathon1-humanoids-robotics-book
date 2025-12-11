import os
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_gemini_connection():
    """Test basic connection to Google Gemini API"""
    try:
        # Get API key from environment
        api_key = os.getenv("GOOGLE_API_KEY")

        if not api_key:
            print("ERROR: GOOGLE_API_KEY not found in environment variables")
            print("Please set your Gemini API key in the .env file")
            return False

        # Configure the API key
        genai.configure(api_key=api_key)

        # List available models as a basic test
        models = genai.list_models()
        print("Available models:")
        for model in models:
            print(f"  - {model.name}")

        # Test with a simple model
        model = genai.GenerativeModel('gemini-pro') if any('gemini-pro' in m.name for m in models) else genai.GenerativeModel('gemini-2.5-flash')

        # Generate a simple response
        response = model.generate_content("Hello, world! Just say hello back.")
        print(f"\nTest response: {response.text}")

        print("\n✓ Google Generative AI SDK is working correctly!")
        return True

    except Exception as e:
        print(f"✗ Error testing Google Generative AI SDK: {str(e)}")
        return False

if __name__ == "__main__":
    print("Testing Google Generative AI SDK...")
    success = test_gemini_connection()
    if not success:
        print("\nPlease make sure you have a valid Gemini API key in your .env file")