---
id: 1
title: "Constitution Creation"
stage: "constitution"
date_iso: "2025-12-25"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "UNI-TECH"
command: "/sp.constitution"
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - ".specify/memory/constitution.md"
  - "history/prompts/constitution/1-constitution-creation.constitution.prompt.md"
tests_yaml: []
---

## Prompt Text

Requirements

You are required to complete a unified book project using Claude Code and Spec-Kit Plus. The core deliverables are:

1. AI/Spec-Driven Book Creation: Write a book using Docusaurus and deploy it to GitHub Pages. You will use Spec-Kit Plus ( https://github.com/panaversity/spec-kit-plus/ ) and Claude Code ( https://www.claude.com/product/claude-code ) to write the book.

2. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

reference constitution file prompt:

/sp.constitution

Project: Research paper on AI-native software development

Core principles:
- Accuracy through primary source verification
- Clarity for academic audience (computer science background)
- Reproducibility (all claims traceable)
- Rigor (peer-reviewed sources preferred)

Key standards:
- All factual claims must be traceable to sources
- Citation format: APA style
- Source types: minimum 50% peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 5,000-7,000 words
- Minimum 15 sources
- Format: PDF with embedded citations

Success criteria:
- All claims verified against sources
- Zero plagiarism detected
- Passes fact-checking review

## Response Text

I've successfully created a comprehensive constitution file for your unified book project with RAG chatbot. The constitution includes:

1. Core principles aligned with your project requirements
2. Technology stack requirements covering Docusaurus, OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud
3. Development workflow following Spec-Kit Plus methodology
4. Success criteria for both the book and chatbot functionality
5. Constraints and risk management considerations
6. Governance guidelines for the project

The constitution is now in place at `.specify/memory/constitution.md` and follows the Spec-Kit Plus approach you requested. This will serve as the foundational document guiding your project development.

Since this was a constitution creation task, I'm also creating a PHR (Prompt History Record) as required by the Claude Code rules. The constitution follows your requirements for a unified book project with an integrated RAG chatbot using the specified technology stack.

No architectural decision records (ADRs) were needed for this task as we were creating the foundational constitution rather than making specific architectural decisions that would require separate documentation.

The project constitution is now ready for use with your hackathon project!

## Outcome

Successfully created project constitution file at `.specify/memory/constitution.md` with appropriate governance, principles, and technology stack requirements for the unified book project with RAG chatbot.

## Evaluation

- Constitution addresses all project requirements
- File created at correct location
- Appropriate structure and content for project governance
- PHR created as required by Claude Code rules