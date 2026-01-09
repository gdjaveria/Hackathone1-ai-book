#!/usr/bin/env python3
"""
FastAPI server for the RAG Agent
This script starts the FastAPI server for the RAG agent API
"""

import uvicorn
import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

def main():
    """
    Main function to start the FastAPI server
    """
    print("Starting RAG Agent API server...")
    print("Loading environment variables...")

    # Run the FastAPI app with uvicorn
    uvicorn.run(
        "backend.rag_api:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable auto-reload during development
        log_level="info"
    )

if __name__ == "__main__":
    main()