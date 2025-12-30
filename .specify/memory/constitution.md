# AI-Native Textbook: Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Spec-Driven Development (Non-Negotiable)
All work must follow the Spec-Kit Plus lifecycle: /sp.specify → /sp.plan → /sp.tasks → implementation. No direct or ad-hoc coding, content writing, or UI changes outside an approved spec.

### II. Incremental & Phase-Based Development
The project MUST be developed in clearly separated phases. A phase must be fully completed, reviewed, and stabilized before starting the next phase. Future phases must NOT modify or break completed phases unless explicitly specified.

### III. Single Constitution Rule
This constitution is written ONCE and must remain stable. All future changes are handled through new specifications, not by rewriting this constitution.

### IV. Academic Rigor & Technical Accuracy
All content must be technically accurate, concept-first, and follow APA citation style where references are required. Content must be structured for beginner-to-advanced learning progression.

### V. Phase Boundary Integrity
Each phase has clearly defined boundaries and constraints. Work in one phase must not cross into future phases. UI customization, RAG, authentication, or personalization are locked to their respective phases.

### VI. Quality & Safety Guarantees
Each phase must be independently buildable and deployable. No phase may introduce breaking changes to previous phases. Architecture decisions must be documented in plans. The final system must be suitable for hackathon judging, live demo, and GitHub Pages deployment.

## Development Phases

### PHASE 1 — Core Book Content (Modules & Chapters)
Goal: Produce high-quality, academically structured course content.
- Modules are written sequentially, one at a time.
- Each module contains clearly defined chapters.
- Content must be technically accurate, concept-first, and beginner-to-advanced.
- APA citation style must be followed where references are required.
- Constraint: UI customization, RAG, authentication, or personalization is NOT allowed in this phase.

### PHASE 2 — Custom Book UI (Frontend System)
Goal: Replace default Docusaurus UI with a sleek, user-friendly, modern reading experience.
- All UI work must live inside a dedicated /frontend folder.
- Design must be token-driven (colors, fonts, spacing, typography).
- The UI must enhance readability, navigation, and learning focus.
- No content rewriting is allowed in this phase.
- Constraint: Backend systems (RAG, Auth) are NOT implemented yet.

### PHASE 3 — Integrated RAG Chatbot
Goal: Embed an AI assistant that answers questions strictly from book content.
- Use OpenAI Agents / ChatKit SDKs.
- Use FastAPI for backend.
- Use Qdrant (Free Tier) for vector storage.
- Use Neon Serverless Postgres for metadata.
- The chatbot must support: Full-book Q&A, Q&A based only on user-selected text
- Constraint: No authentication or personalization logic in this phase.

### PHASE 4 — Authentication & Personalization
Goal: Personalize the learning experience per user.
- Implement signup/signin using Better-Auth.
- Collect user background (software, hardware, robotics level).
- Enable: Chapter-level personalization (button-based), Urdu translation toggle per chapter
- Constraint: Personalization must not alter original canonical content. All transformations must be reversible and user-controlled.

## Governance

This constitution supersedes all other practices and development approaches. All implementations must verify compliance with phase boundaries and spec-driven development. Any deviation from this constitution requires explicit amendment with proper documentation and approval.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
