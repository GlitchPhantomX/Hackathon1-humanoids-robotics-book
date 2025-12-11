# RAG Chatbot Setup Guide

This guide provides comprehensive instructions for setting up and running the RAG (Retrieval Augmented Generation) Chatbot, translation services, personalized recommendations, and an admin dashboard for the Physical AI & Humanoid Robotics textbook.

## Overview

The system consists of two main parts:

1.  **Backend (FastAPI)**: Handles API requests, RAG pipeline logic (Qdrant, Gemini API), user authentication and profiles, translation management, content recommendations, and admin functionalities.
2.  **Frontend (Docusaurus/React)**: The Docusaurus site with integrated components for user interaction, including authentication, language toggling, translation triggering, points display, recommendations, and error notifications.

## Prerequisites

Before you begin, ensure you have the following installed:

*   **Python 3.11+** (recommended)
*   **Node.js and npm** (LTS version recommended)
*   **Git**
*   **Docker** (for running Redis locally for rate limiting, optional but recommended)
*   **PostgreSQL database** (e.g., Neon Serverless Postgres)
*   **Qdrant instance** (e.g., Qdrant Cloud)
*   **Google Gemini API Key**

## Quick Start (Refer to `specs/1-rag-chatbot-spec/quickstart.md` for detailed commands)

For detailed step-by-step commands to set up the backend and frontend, including virtual environment activation, dependency installation, and running servers, please refer to the `quickstart.md` located in `specs/1-rag-chatbot-spec/quickstart.md`.

## 1. Backend Setup

### 1.1. Environment Variables

Create a `.env` file in the `backend/` directory based on the following example. You **must** fill in your API keys, database, and Redis connection strings.

```ini
# .env file in backend/
GEMINI_API_KEY="YOUR_GOOGLE_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_CLUSTER_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
DATABASE_URL="postgresql://user:password@host:port/database"
REDIS_URL="redis://localhost:6379/0" # Or your Redis instance URL
SECRET_KEY="YOUR_RANDOM_SECRET_KEY_FOR_JWT" # Generate a strong, random key
FASTAPI_ENV="development" # or "production"
```

### 1.2. Database Migrations (Alembic)

After creating your `.env` file and ensuring `DATABASE_URL` is set, you will need to initialize and run database migrations using Alembic.

1.  Initialize Alembic (if not already done):
    ```bash
    cd backend
    venv312/Scripts/alembic.exe init alembic # Adjust path for your venv
    ```
2.  Configure `alembic.ini` and `alembic/env.py` (as done during development).
3.  Generate a new migration after changes to `app/models/database.py`:
    ```bash
    cd backend
    venv312/Scripts/alembic.exe revision --autogenerate -m "Add User and Translation models, roles, points"
    ```
4.  Apply migrations:
    ```bash
    cd backend
    venv312/Scripts/alembic.exe upgrade head
    ```

### 1.3. Indexing Textbook Content (for RAG and Recommendations)

To enable RAG functionality and personalized recommendations, index your Docusaurus markdown content into Qdrant.

1.  Ensure your Docusaurus docs are located (e.g., `physical-ai-robotics-textbook/docusaurus/docs/`).
2.  Run the ingestion script:
    ```bash
    cd backend
    python physical-ai-robotics-textbook/docusaurus/backend/scripts/ingest_site_content.py
    ```

## 2. Frontend Setup (Docusaurus/React)

The frontend integrates various components for enhanced user experience.

### 2.1. Key Components:

*   **Authentication**: `NavbarButtons.tsx`, `AuthModal.tsx` for user login/signup.
*   **Translation**: `TranslateButton.tsx`, `LanguageToggle.tsx` for chapter translation.
*   **User Dashboard**: `UserDashboard.tsx` displaying points, progress, and recommendations.
*   **Admin Dashboard**: `AdminDashboard.tsx` for monitoring users and translations.
*   **Error Handling**: `ErrorModal.tsx` for user-friendly error notifications.

## 3. Core Features

### 3.1. User Authentication & Profiles (Phase 3)

*   Users can sign up and log in securely.
*   User profiles store basic information and `total_points`.
*   Role-based access control (`user` vs `admin`) implemented.

### 3.2. Translate Chapters to Urdu (Phase 4)

*   Logged-in users can trigger translations of chapters to Urdu.
*   Translation status can be tracked.
*   Translated content is stored and can be displayed.

### 3.3. Earn Bonus Points (Phase 5)

*   Users receive 50 bonus points for each unique, full chapter translation they complete.
*   Points are tracked in the user's profile and duplicate awards are prevented.
*   Notifications are displayed via `PointsModal`.

### 3.4. Personalized Content Recommendations (Phase 6)

*   Users receive chapter recommendations based on their translation activity and other factors.
*   Displayed on the `UserDashboard`.

### 3.5. Admin Dashboard (Phase 7)

*   Admin users can access a dedicated dashboard.
*   View lists of all users and their details.
*   View all translation requests and their statuses.
*   Access protected by admin role.

## 4. Observability & Security (Phase 8)

*   **Logging**: Structured logging for all backend services.
*   **Metrics & Tracing**: Placeholders for future integration.
*   **Rate Limiting**: Implemented on key API endpoints (e.g., `/api/auth/signup`, `/api/auth/login`, `/api/translation/trigger`) using `fastapi-limiter`.
*   **Input Validation**: Handled by Pydantic schemas and FastAPI's automatic validation.

## 5. Deployment

*   **Backend**: Can be deployed to platforms like Railway (refer to `quickstart.md`).
*   **Frontend**: Can be deployed to platforms like Vercel or GitHub Pages (refer to `quickstart.md`).

## Troubleshooting

*   **"pg_config not found" error**: Ensure Python development headers and/or PostgreSQL client libraries are installed if `psycopg2-binary` fails.
*   **Redis Connection Issues**: Ensure your Redis server is running and `REDIS_URL` in `.env` is correct.
*   **JWT Errors**: Check `SECRET_KEY` in `.env` is set and matches.
*   **Admin Access Denied**: Verify the logged-in user's email matches the placeholder admin email (`admin@example.com`) or has the "admin" role if RBAC is fully implemented.
*   **Frontend Errors**: Check browser console for network errors (API calls) or React component issues. Ensure backend is running.

---