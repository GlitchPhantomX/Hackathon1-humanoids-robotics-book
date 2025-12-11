# Quickstart for RAG Chatbot

This document provides a brief guide to setting up and running the RAG Chatbot project.

## Prerequisites

- Python 3.11+
- Node.js and npm (for Docusaurus)
- Docker (optional, for running databases locally)

## Backend Setup

1.  **Create a virtual environment**:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

2.  **Install dependencies**:
    ```bash
    pip install -r backend/requirements.txt
    ```

3.  **Set up environment variables**:
    Copy `backend/.env.example` to `backend/.env` and fill in the required values for the Gemini API, Qdrant, and Neon.

4.  **Run the database setup script**:
    ```bash
    python backend/scripts/setup_db.py
    ```

5.  **Run the ingestion script**:
    ```bash
    python backend/scripts/ingest_documents.py
    ```

6.  **Start the backend server**:
    ```bash
    uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
    ```

## Frontend Setup

1.  **Install dependencies**:
    ```bash
    cd physical-ai-robotics-textbook/docusaurus
    npm install
    ```

2.  **Start the development server**:
    ```bash
    npm start
    ```

The Docusaurus site will be available at `http://localhost:3000`.

## Running Tests

To run the backend tests:
```bash
pytest backend/tests
```