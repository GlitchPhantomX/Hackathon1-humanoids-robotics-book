---
id: "025"
title: "Fix Backend Startup SSL Mode Error"
stage: "red"
feature: "backend"
branch: "7-reusable-intelligence-system"
date_iso: "2025-12-13"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
user: "unknown"
command: "/sp.phr"
labels: ["debugging", "database", "postgres", "asyncpg", "cors", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "physical-ai-robotics-textbook/backend/.env"
  - "physical-ai-robotics-textbook/backend/app/services/postgres_service.py"
  - "physical-ai-robotics-textbook/backend/app/config.py"
  - "physical-ai-robotics-textbook/backend/main.py"
tests:
  - "none"
---

# Fix Backend Startup SSL Mode Error

## Prompt Text
```
> You are debugging a FastAPI + React project.
> The backend fails to start and frontend fetch requests are failing.

### **Project Structure**

```
physical-ai-robotics-textbook/
└── backend/
    ├── main.py
    ├── app/
    │   ├── config.py
    │   └── services/
    │       └── postgres_service.py
    ├── .env
    └── venv/
```

---

### **Observed Errors**

**Backend startup error**

```
TypeError: connect() got an unexpected keyword argument 'sslmode'
ERROR: Application startup failed. Exiting.
```

The error occurs during FastAPI lifespan startup when calling:

```
await postgres_service.init_db()
```

Stack trace points to:

```
backend/app/services/postgres_service.py
sqlalchemy.dialects.postgresql.asyncpg
```

---

**Frontend errors (secondary)**

```
ERR_CONNECTION_REFUSED
Failed to fetch
CORS policy error
```

These happen because the backend never starts.

---

### **Root Cause**

The database connection uses:

```
postgresql+asyncpg://
```

but the DATABASE_URL contains:

```
?sslmode=require
```

`asyncpg` **does NOT support `sslmode`**, so SQLAlchemy passes an invalid argument to asyncpg, causing startup failure.

---

### **Tasks to Perform**

1. **Fix DATABASE_URL handling**

   * Remove `sslmode` from `.env`
   * OR ensure `ssl` is passed correctly via `connect_args`
2. **Update backend code if necessary**

   * File to inspect and fix:

     ```
     backend/app/services/postgres_service.py
     ```
   * Ensure `create_async_engine()` does NOT pass `sslmode`
3. **Ensure backend starts successfully**

   * FastAPI should reach:

     ```
     Application startup complete.
     ```
4. **Fix CORS configuration**

   * File:

     ```
     backend/main.py
     ```
   * Allow frontend origin:

     ```
     http://localhost:3002
     ```

---

### **Expected Result**

* Backend starts without errors
* PostgreSQL connection works with asyncpg
* React frontend can successfully call:

  ```
  /api/auth/get-session
  /api/chat
  ```
* No CORS or connection refused errors
```

## Response Text
Successfully fixed the FastAPI backend startup error by removing the 'sslmode' parameter from the DATABASE_URL in the .env file and updating the postgres_service.py to properly handle asyncpg connections. Also fixed CORS configuration in main.py to allow http://localhost:3002 as requested. The backend now starts successfully without the sslmode error and reaches 'Application startup complete'.

## Outcome Impact
The backend now starts successfully without the TypeError related to sslmode. The PostgreSQL connection works with asyncpg, and CORS is configured to allow the frontend origin. This resolves the primary issue preventing the application from starting.

## Tests Summary
none

## Files Summary
- physical-ai-robotics-textbook/backend/.env
- physical-ai-robotics-textbook/backend/app/services/postgres_service.py
- physical-ai-robotics-textbook/backend/app/config.py
- physical-ai-robotics-textbook/backend/main.py

## Next Prompts
none

## Reflection Note
The issue was resolved by understanding that asyncpg doesn't support the PostgreSQL-specific sslmode parameter that is commonly used with psycopg2. The solution involved removing the problematic query parameters from the connection string and properly configuring the connection using asyncpg-compatible parameters.

## Evaluation Notes
**Failure modes observed:** None
**Next experiment to improve prompt quality:** None needed - the issue was successfully resolved according to all specified requirements.