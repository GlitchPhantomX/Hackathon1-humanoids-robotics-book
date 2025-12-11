# Physical AI & Robotics Textbook RAG Chatbot

An intelligent chatbot that allows users to ask questions about the Physical AI & Robotics textbook using Retrieval-Augmented Generation (RAG).

## Overview

This project implements a RAG-based chatbot that enables users to have conversations with the Physical AI & Robotics textbook. The system:

- Ingests textbook content into a vector database
- Performs semantic search to find relevant passages
- Uses AI to generate contextual, accurate responses
- Maintains conversation history
- Provides source citations for all answers

## Features

- **Intelligent Search**: Finds relevant content using vector similarity search
- **Context-Aware Responses**: Understands conversation context and follows up appropriately
- **Source Citations**: All answers include citations to specific textbook sections
- **Text Selection**: Users can select text on any page and ask questions about it
- **Persistent Conversations**: Conversation history maintained across sessions
- **Responsive Design**: Works on desktop and mobile devices

## Architecture

The system consists of two main components:

### Backend (FastAPI)
- Handles document ingestion and vectorization
- Manages conversation state in PostgreSQL
- Performs semantic search in Qdrant
- Generates responses using Google Generative AI
- Provides REST API endpoints

### Frontend (Docusaurus + React)
- Integrated chat widget on all documentation pages
- Text selection popup for asking questions about selected text
- Conversation history persistence via sessionStorage
- Responsive design for desktop and mobile

## Setup & Installation

### Backend
1. Navigate to the `backend` directory
2. Follow the setup instructions in [backend/README.md](backend/README.md)

### Frontend
1. The chat widget is already integrated with the Docusaurus documentation site
2. The widget will appear on all pages automatically

## Usage

1. **Document Ingestion**: Run the ingestion script to load textbook content into the vector database
2. **Start the Backend**: Run the FastAPI server
3. **Browse Documentation**: Visit any documentation page
4. **Ask Questions**: Use the floating chat button or select text to ask questions about the content

## Technologies Used

- **Backend**: Python, FastAPI, PostgreSQL, Qdrant, Google Generative AI
- **Frontend**: React, Docusaurus, JavaScript/TypeScript
- **Vector Storage**: Qdrant Cloud
- **Database**: Neon Postgres
- **AI Model**: Google Gemini Pro

## Development

See individual README files for development setup:
- [backend/README.md](backend/README.md) for backend development setup

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

[Specify license here]