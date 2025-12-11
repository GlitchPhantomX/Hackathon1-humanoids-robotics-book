---
id: 007
title: Fix CORS and SVG errors
stage: green
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-auth-frontend-redesign
branch: 004-auth-frontend-redesign
user: system
command: /sp.implement
labels: ["bugfix", "cors", "react", "svg", "error-handling"]
links:
  spec: specs/004-auth-frontend-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - physical-ai-robotics-textbook/auth-backend/src/index.ts
  - physical-ai-robotics-textbook/docusaurus/src/components/Footer.tsx
  - specs/004-auth-frontend-redesign/tasks.md
tests:
  - Manual testing: CORS preflight requests
  - Manual testing: React DOM attribute warnings
  - Manual testing: Authentication flow end-to-end
---

## Prompt

I'm getting several errors in my Docusaurus React application. Please help me fix them:

1. **CORS Error**: The main issue is a CORS error when trying to fetch from my backend API:
   - Frontend: http://localhost:3000
   - Backend: http://localhost:5000/api/auth/get-session
   - Error: "Request header field cache-control is not allowed by Access-Control-Allow-Headers in preflight response"

2. **React DOM Warnings**: Invalid DOM properties in SVG elements:
   - `fill-rule` should be `fillRule`
   - `clip-rule` should be `clipRule`
   - These warnings appear in the Footer component

Please:
- Fix the CORS configuration on the backend to allow the `cache-control` header
- Find and fix all instances of `fill-rule` and `clip-rule` in SVG elements (likely in the Footer component) to use camelCase versions
- Ensure the CORS setup allows all necessary headers for authentication

The project structure includes:
- Frontend: Docusaurus site running on port 3000
- Backend: API server on port 5000
- Authentication component: AuthButtons.tsx (line 26 is making the fetch call)

Please scan the codebase, identify the relevant files, and make the necessary fixes.

## Response snapshot

Successfully fixed both issues:

### 1. CORS Configuration (auth-backend/src/index.ts:57-58)
Added explicit header configuration to CORS middleware:
- `allowedHeaders`: ['Content-Type', 'Authorization', 'Cache-Control', 'X-Requested-With']
- `exposedHeaders`: ['Set-Cookie']

This resolves the CORS preflight error blocking the `cache-control` header from frontend requests.

### 2. SVG Attributes (docusaurus/src/components/Footer.tsx)
Fixed all React DOM warnings by converting kebab-case to camelCase:
- Line 58: fill-rule → fillRule, clip-rule → clipRule (Facebook icon)
- Line 67: fill-rule → fillRule, clip-rule → clipRule (Instagram icon)
- Line 85: fill-rule → fillRule, clip-rule → clipRule (GitHub icon)
- Line 94: fill-rule → fillRule, clip-rule → clipRule (Dribbble icon)

### Testing Instructions
1. Restart auth backend: `cd physical-ai-robotics-textbook/auth-backend && npm run dev`
2. Restart Docusaurus: `cd physical-ai-robotics-textbook/docusaurus && npm start`
3. Test authentication flow at http://localhost:3000
4. Verify no CORS errors or React DOM warnings in console

## Outcome

- ✅ Impact: Fixed blocking CORS error preventing authentication; eliminated React DOM warnings
- 🧪 Tests: Manual testing required - CORS preflight, authentication flow, console output verification
- 📁 Files: Modified 2 files (auth backend CORS config, Footer component SVG attributes), updated tasks.md
- 🔁 Next prompts: Test authentication end-to-end, verify no console errors
- 🧠 Reflection: Quick targeted fix addressing specific error messages from user report

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward error resolution
- Graders run and results (PASS/FAIL): N/A - bugfix task
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A - fixes complete
