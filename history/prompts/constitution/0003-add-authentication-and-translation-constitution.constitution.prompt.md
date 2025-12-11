---
id: 0003
title: Add Authentication and Translation Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-cli-agent
feature: none
branch: main
user: user
command: "/sp.constitution now add this constitution into my C:\new\.specify\memory this folder create new file named authentication-translation-constitution.md "
labels: ["constitution", "authentication", "translation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - C:\new\.specify\memory\authentication-translation-constitution.md
tests:
 - null
---

## Prompt

/sp.constitution now add this constitution into my C:\new\.specify\memory this folder create new file named authentication-translation-constitution.md 


# Detailed Constitution for “Physical AI and Humanoids Robotics” Features

**Project Folder:** `C:\new\physical-ai-robotics-textbook\docusaurus`
**Frameworks / Tools:** Docusaurus, Gemini CLI, Speckit Plus, Better-Auth, React

---

## 1. Project Objectives

* Enable users to **translate chapters into Urdu** via Gemini CLI.
* Award **up to 50 bonus points** for full chapter translations.
* Implement **signup/signin** with Better-Auth.
* Collect **user background** to personalize content.
* Ensure **security, data integrity, and seamless user experience**.

---

## 2. Scope

### 2.1 Chapters

| Folder          | Chapter      |
| --------------- | ------------ |
| 00-introduction | Introduction |
| 01-foundations  | Foundations  |
| 02-ros2         | ROS2         |
| 03-simulation   | Simulation   |
| 04-isaac        | Isaac        |
| 05-vla          | VLA          |
| 06-capstone     | Capstone     |

### 2.2 Features

1. User authentication and profile management via Better-Auth.
2. Background questionnaire for personalization.
3. Chapter translation to Urdu.
4. Bonus points system for full translations.
5. Toggle between English and Urdu content.
6. Admin dashboard for monitoring points and translations.

---

## 3. User Roles

| Role        | Permissions                                                                        |
| ----------- | ---------------------------------------------------------------------------------- |
| Participant | Signup/Signin, Translate chapters, Earn bonus points, View personal dashboard      |
| Admin       | View user translation activity, Monitor points, Moderate content, Generate reports |

---

## 4. User Flow

**Step 1: Authentication**

1. User clicks **Signup / Login**.
2. Better-Auth handles registration/login.
3. During signup, user fills **background questionnaire**:

   * Programming experience
   * Robotics experience
   * Hardware familiarity
4. User profile created with unique `user_id`.

**Step 2: Translation Feature**

1. User navigates to a chapter.
2. If **not logged in**, show login/signup prompt.
3. If logged in, show **“Translate to Urdu”** button at the top.
4. Clicking the button triggers Gemini CLI translation request.
5. Store translation in `docs-translations/ur/{chapter-folder}`.
6. Users can **toggle** between English and Urdu without affecting formatting.

**Step 3: Bonus Points Assignment**

1. Track completion status of translation for the chapter.
2. Award **50 bonus points** only when full translation is completed.
3. Update user profile with points and timestamp.
4. Notify user with a modal: *“Congratulations! You earned 50 bonus points.”* 

**Step 4: Personalization**

1. Use background info to recommend chapters or modules.
2. Content adaptation can consider user expertise level.

---

## 5. Technical Architecture

### 5.1 Frontend

* **React Components:**

  * `TranslateButton` – triggers translation.
  * `LanguageToggle` – switches English ↔ Urdu.
  * `PointsModal` – shows bonus points earned.
  * `UserDashboard` – displays points and progress.

* **Docusaurus Integration:**

  * Each chapter remains in `docs/`.
  * Urdu translations stored in `docs-translations/ur/`.
  * Ensure **code blocks, tables, images, and formulas remain intact**.

### 5.2 Backend

* **Authentication:**

  * Integrate Better-Auth for signup/signin.
  * Store `user_id`, `email`, `name`, `background` in database.

* **Translation API:**

  * Trigger Gemini CLI to translate chapter content.
  * Validate translation completion.
  * Store Urdu translation separately.

* **Points System:**

  * API endpoint verifies full translation completion.
  * Update `points_awarded` in database.
  * Prevent duplicate points for the same chapter.

### 5.3 Database Schema

**User Table:**

| Column       | Type       | Description              |
| ------------ | ---------- | ------------------------ |
| user_id      | INT / UUID | Primary key              |
| name         | VARCHAR    | User full name           |
| email        | VARCHAR    | User email               |
| background   | JSON       | Questionnaire answers    |
| signup_date  | DATETIME   | Timestamp of signup      |
| total_points | INT        | Accumulated bonus points |

**Translation Table:**

| Column         | Type       | Description          |
| -------------- | ---------- | -------------------- |
| translation_id | INT / UUID | Primary key          |
| user_id        | FK         | Links to user        |
| chapter_id     | VARCHAR    | Chapter folder name  |
| status         | ENUM       | Pending / Completed  |
| points_awarded | INT        | Bonus points granted |
| timestamp      | DATETIME   | Completion time      |

---

## 6. Security Considerations

* Translation bonus requires **authenticated user**.
* Prevent points manipulation.
* Proper UTF-8 encoding for Urdu content.
* Only allow **one full translation per user per chapter** for points.

---

## 7. Acceptance Criteria

1. Signup/Signin works via Better-Auth.
2. Background questionnaire collected and stored.
3. Translation button appears **only for logged-in users**.
4. Gemini CLI translates chapters accurately to Urdu.
5. Users can toggle between English and Urdu.
6. Full translation awards **50 bonus points**.
7. Admin can monitor user translation activity and points.
8. Docusaurus folder structure remains unchanged.
9. Personalization recommendations reflect user background.

---

## 8. Implementation Notes

* Keep Gemini CLI calls **modular and isolated**.
* Translation files should **never overwrite English originals**.
* UI should gracefully handle translation progress and errors.
* Personalization logic is extensible for future content adaptation.
## Response snapshot

Successfully created and wrote to new file: C:\new\.specify\memory\authentication-translation-constitution.md.

## Outcome

- ✅ Impact: Created a new constitution file for authentication and translation features.
- 🧪 Tests: No tests were run.
- 📁 Files: 1 file created.
- 🔁 Next prompts: Continue with implementation based on the constitution.
- 🧠 Reflection: The PHR was created manually due to the shell script failure. All placeholders were filled according to the template.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed to execute.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` is executable in the current shell environment, or provide a robust agent-native fallback that mimics its functionality.
