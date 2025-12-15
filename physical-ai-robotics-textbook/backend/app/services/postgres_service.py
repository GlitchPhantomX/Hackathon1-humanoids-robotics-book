"""
PostgreSQL service for conversation management and message storage.
Uses asyncpg and SQLAlchemy for async database operations.
"""

from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4

# import asyncpg
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON, text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from app.config import settings
from app.models import (
    ConversationBase,
    MessageBase,
    ConversationCreate,
    MessageCreate,
    ConversationResponse,
)
import json


Base = declarative_base()


class ConversationDB(Base):
    __tablename__ = "conversations"

    id = Column(String, primary_key=True, default=lambda: str(uuid4()))
    user_id = Column(String, nullable=True)
    title = Column(String(500), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)


class MessageDB(Base):
    __tablename__ = "messages"

    id = Column(String, primary_key=True, default=lambda: str(uuid4()))
    conversation_id = Column(String, ForeignKey("conversations.id"), nullable=False)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    sources = Column(JSON, nullable=True)  # JSONB field for source citations

    # Relationship
    conversation = relationship("ConversationDB", back_populates="messages")


ConversationDB.messages = relationship(
    "MessageDB", order_by=MessageDB.timestamp, back_populates="conversation"
)


class PostgresService:
    """
    Service class for PostgreSQL database operations related to conversation management.
    """

    def __init__(self):
        # Configure SSL connection arguments for asyncpg
        connect_args = {
            "server_settings": {
                "application_name": "rag-chatbot",
            }
        }

        self.engine = create_async_engine(
            settings.database_url,
            echo=False,  # Set to True for SQL query logging during development
            connect_args=connect_args,
        )
        self.async_session = sessionmaker(
            self.engine, class_=AsyncSession, expire_on_commit=False
        )

    async def init_db(self):
        """Initialize database tables"""
        async with self.engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

    async def create_conversation(
        self, user_id: Optional[str] = None, title: Optional[str] = None
    ) -> Dict[str, Any]:
        """Create a new conversation"""
        conversation_id = str(uuid4())
        async with self.async_session() as session:
            async with session.begin():
                # Create conversation
                conversation = ConversationDB(
                    id=conversation_id, user_id=user_id, title=title
                )
                session.add(conversation)
                await session.commit()

                return {
                    "id": conversation_id,
                    "user_id": user_id,
                    "title": title,
                    "created_at": conversation.created_at,
                }

    async def get_conversation(self, conversation_id: str) -> Optional[Dict[str, Any]]:
        """Get a conversation by ID"""
        async with self.async_session() as session:
            conversation = await session.get(ConversationDB, conversation_id)
            if not conversation:
                return None

            return {
                "id": conversation.id,
                "user_id": conversation.user_id,
                "title": conversation.title,
                "created_at": conversation.created_at,
                "updated_at": conversation.updated_at,
            }

    async def get_conversations(
        self, user_id: Optional[str] = None, limit: int = 50
    ) -> List[Dict[str, Any]]:
        """Get all conversations for a user (or all conversations if user_id is None)"""
        async with self.async_session() as session:
            query = text(
                """
                SELECT id, user_id, title, created_at, updated_at
                FROM conversations
                WHERE (:user_id IS NULL OR user_id = :user_id)
                ORDER BY updated_at DESC
                LIMIT :limit
            """
            )
            result = await session.execute(query, {"user_id": user_id, "limit": limit})
            rows = result.fetchall()

            conversations = []
            for row in rows:
                conversations.append(
                    {
                        "id": row[0],
                        "user_id": row[1],
                        "title": row[2],
                        "created_at": row[3],
                        "updated_at": row[4],
                    }
                )

            return conversations

    async def update_conversation_title(self, conversation_id: str, title: str) -> bool:
        """Update conversation title"""
        async with self.async_session() as session:
            async with session.begin():
                conversation = await session.get(ConversationDB, conversation_id)
                if not conversation:
                    return False

                conversation.title = title
                conversation.updated_at = datetime.utcnow()
                await session.commit()
                return True

    async def create_message(
        self,
        conversation_id: str,
        role: str,
        content: str,
        sources: Optional[List[Dict]] = None,
    ) -> Dict[str, Any]:
        """Create a new message in a conversation"""
        message_id = str(uuid4())
        async with self.async_session() as session:
            async with session.begin():
                # Verify conversation exists
                conversation = await session.get(ConversationDB, conversation_id)
                if not conversation:
                    raise ValueError(f"Conversation {conversation_id} does not exist")

                # Create message
                message = MessageDB(
                    id=message_id,
                    conversation_id=conversation_id,
                    role=role,
                    content=content,
                    sources=sources,
                )
                session.add(message)

                # Update conversation timestamp
                conversation.updated_at = datetime.utcnow()

                await session.commit()

                return {
                    "id": message_id,
                    "conversation_id": conversation_id,
                    "role": role,
                    "content": content,
                    "sources": sources,
                    "timestamp": message.timestamp,
                }

    async def get_messages(
        self, conversation_id: str, limit: int = 50
    ) -> List[Dict[str, Any]]:
        """Get messages for a conversation"""
        async with self.async_session() as session:
            query = text(
                """
                SELECT id, conversation_id, role, content, sources, timestamp
                FROM messages
                WHERE conversation_id = :conversation_id
                ORDER BY timestamp ASC
                LIMIT :limit
            """
            )
            result = await session.execute(
                query, {"conversation_id": conversation_id, "limit": limit}
            )
            rows = result.fetchall()

            messages = []
            for row in rows:
                messages.append(
                    {
                        "id": row[0],
                        "conversation_id": row[1],
                        "role": row[2],
                        "content": row[3],
                        "sources": row[4],
                        "timestamp": row[5],
                    }
                )

            return messages

    async def get_recent_messages(
        self, conversation_id: str, limit: int = 10
    ) -> List[Dict[str, Any]]:
        """Get recent messages for a conversation (for context)"""
        async with self.async_session() as session:
            query = text(
                """
                SELECT id, conversation_id, role, content, sources, timestamp
                FROM messages
                WHERE conversation_id = :conversation_id
                ORDER BY timestamp DESC
                LIMIT :limit
            """
            )
            result = await session.execute(
                query, {"conversation_id": conversation_id, "limit": limit}
            )
            rows = result.fetchall()

            # Reverse the order to return in chronological order
            messages = []
            for row in reversed(rows):
                messages.append(
                    {
                        "id": row[0],
                        "conversation_id": row[1],
                        "role": row[2],
                        "content": row[3],
                        "sources": row[4],
                        "timestamp": row[5],
                    }
                )

            return messages

    async def delete_conversation(self, conversation_id: str) -> bool:
        """Delete a conversation and all its messages"""
        async with self.async_session() as session:
            async with session.begin():
                # Delete messages first due to foreign key constraint
                await session.execute(
                    text(
                        "DELETE FROM messages WHERE conversation_id = :conversation_id"
                    ),
                    {"conversation_id": conversation_id},
                )

                # Delete conversation
                result = await session.execute(
                    text("DELETE FROM conversations WHERE id = :conversation_id"),
                    {"conversation_id": conversation_id},
                )

                await session.commit()
                return result.rowcount > 0

    async def update_conversation(self, conversation_id: str, **kwargs) -> bool:
        """Update conversation fields"""
        valid_fields = {"user_id", "title"}
        update_fields = {k: v for k, v in kwargs.items() if k in valid_fields}

        if not update_fields:
            return False

        async with self.async_session() as session:
            async with session.begin():
                conversation = await session.get(ConversationDB, conversation_id)
                if not conversation:
                    return False

                for field, value in update_fields.items():
                    setattr(conversation, field, value)

                conversation.updated_at = datetime.utcnow()
                await session.commit()
                return True

    async def close(self):
        """Close the database engine"""
        await self.engine.dispose()


# Global service instance
postgres_service = PostgresService()
