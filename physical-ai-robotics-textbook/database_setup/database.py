from sqlalchemy import create_engine, Column, Integer, String, DateTime, ForeignKey, Text
from sqlalchemy.orm import sessionmaker, declarative_base, relationship
from sqlalchemy.sql import func
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

# Database connection string from environment variable
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://user:password@host/db?sslmode=require")

# Create a SQLAlchemy engine
engine = create_engine(DATABASE_URL)

# Create a declarative base for our models
Base = declarative_base()

# Define Database Models
class Conversation(Base):
    __tablename__ = "conversations"
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, index=True, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")

class Message(Base):
    __tablename__ = "messages"
    id = Column(Integer, primary_key=True, index=True)
    conversation_id = Column(Integer, ForeignKey("conversations.id"), nullable=False)
    role = Column(String, nullable=False)  # e.g., "user", "assistant"
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    selected_text = Column(Text, nullable=True) # Text selected by the user for context/feedback

    conversation = relationship("Conversation", back_populates="messages")
    feedback = relationship("Feedback", back_populates="message", uselist=False, cascade="all, delete-orphan")

class Feedback(Base):
    __tablename__ = "feedback"
    id = Column(Integer, primary_key=True, index=True)
    message_id = Column(Integer, ForeignKey("messages.id"), nullable=False, unique=True)
    rating = Column(Integer, nullable=False) # e.g., 1-5 stars
    comment = Column(Text, nullable=True)

    message = relationship("Message", back_populates="feedback")

# Session Local for database interactions
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Dependency to get the DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# CRUD Operations

# --- Conversations ---
def create_conversation(db_session, user_id: str):
    db_conversation = Conversation(user_id=user_id)
    db_session.add(db_conversation)
    db_session.commit()
    db_session.refresh(db_conversation)
    return db_conversation

def get_conversation(db_session, conversation_id: int):
    return db_session.query(Conversation).filter(Conversation.id == conversation_id).first()

def get_conversations_by_user(db_session, user_id: str):
    return db_session.query(Conversation).filter(Conversation.user_id == user_id).all()

# --- Messages ---
def create_message(db_session, conversation_id: int, role: str, content: str, selected_text: str = None):
    db_message = Message(
        conversation_id=conversation_id,
        role=role,
        content=content,
        selected_text=selected_text
    )
    db_session.add(db_message)
    db_session.commit()
    db_session.refresh(db_message)
    return db_message

def get_message(db_session, message_id: int):
    return db_session.query(Message).filter(Message.id == message_id).first()

def get_messages_by_conversation(db_session, conversation_id: int):
    return db_session.query(Message).filter(Message.conversation_id == conversation_id).all()

# --- Feedback ---
def create_feedback(db_session, message_id: int, rating: int, comment: str = None):
    db_feedback = Feedback(
        message_id=message_id,
        rating=rating,
        comment=comment
    )
    db_session.add(db_feedback)
    db_session.commit()
    db_session.refresh(db_feedback)
    return db_feedback

def get_feedback_by_message(db_session, message_id: int):
    return db_session.query(Feedback).filter(Feedback.message_id == message_id).first()

# You can add update and delete functions as needed following similar patterns
