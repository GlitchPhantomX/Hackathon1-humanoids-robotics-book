from database import SessionLocal, create_conversation, create_message, create_feedback
from sqlalchemy.orm import Session
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def insert_sample_data():
    if not os.getenv("DATABASE_URL"):
        print("Error: DATABASE_URL environment variable is not set.")
        print("Please set it before running this script.")
        return

    db: Session = SessionLocal()
    try:
        print("Inserting sample data...")

        # Create a conversation
        user_id_1 = "test_user_1"
        conversation_1 = create_conversation(db, user_id_1)
        print(f"Created conversation: {conversation_1.id} for user {conversation_1.user_id}")

        # Add messages to the conversation
        message_1_user = create_message(
            db, conversation_1.id, "user", "Hello, how can you help me today?",
            selected_text="Hello"
        )
        print(f"Created message: {message_1_user.id} (user)")

        message_1_assistant = create_message(
            db, conversation_1.id, "assistant", "I can provide information and answer questions."
        )
        print(f"Created message: {message_1_assistant.id} (assistant)")

        # Add feedback to a message
        feedback_1 = create_feedback(db, message_1_assistant.id, 5, "Very helpful response!")
        print(f"Created feedback for message {feedback_1.message_id} with rating {feedback_1.rating}")

        # Create another conversation and messages
        user_id_2 = "test_user_2"
        conversation_2 = create_conversation(db, user_id_2)
        print(f"Created conversation: {conversation_2.id} for user {conversation_2.user_id}")

        message_2_user = create_message(
            db, conversation_2.id, "user", "What is the capital of France?"
        )
        print(f"Created message: {message_2_user.id} (user)")

        message_2_assistant = create_message(
            db, conversation_2.id, "assistant", "The capital of France is Paris."
        )
        print(f"Created message: {message_2_assistant.id} (assistant)")

        print("Sample data insertion complete.")

    except Exception as e:
        print(f"An error occurred: {e}")
        db.rollback()
    finally:
        db.close()

if __name__ == "__main__":
    insert_sample_data()
