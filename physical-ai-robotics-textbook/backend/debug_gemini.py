import os
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def debug_gemini():
    """Debug Gemini models"""
    api_key = os.getenv("GOOGLE_API_KEY")
    if not api_key:
        print("GOOGLE_API_KEY not found")
        return

    genai.configure(api_key=api_key)

    try:
        models = list(genai.list_models())  # Convert generator to list
        print(f"Total models found: {len(models)}")

        for i, model in enumerate(models):
            print(f"Model {i+1}: {model.name}")
            has_gemini = 'gemini' in model.name.lower()
            print(f"  Contains 'gemini': {has_gemini}")

        gemini_models = [model.name for model in models if 'gemini' in model.name.lower()]
        print(f"\nFiltered gemini models: {gemini_models}")

        # Test with the first available model
        if gemini_models:
            model_name = gemini_models[0].replace('models/', '')
            print(f"Using model: {model_name}")

            model = genai.GenerativeModel(model_name)
            response = model.generate_content("Hello, world! Just say hello back.")
            print(f"Response: {response.text}")
        else:
            print("No gemini models found!")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    debug_gemini()