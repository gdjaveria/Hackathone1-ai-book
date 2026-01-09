#!/usr/bin/env python3
"""
FastAPI server for the RAG Agent API
This script starts the FastAPI server for the RAG agent API
"""

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, field_validator
from typing import Optional
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import logging
import os
from dotenv import load_dotenv
import sys
import asyncio
from contextlib import asynccontextmanager
from datetime import datetime

# Add the project root directory to the path so we can import the agent
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Load environment variables
load_dotenv()

# Import the agent from the main agent.py file
from agent import AIAgentRetrieval

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Request model
class QueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None  # For maintaining conversation context if needed

    @field_validator('query')
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query cannot be empty or whitespace only')
        if len(v) > 1000:
            raise ValueError('Query must be between 1 and 1000 characters')
        return v.strip()

    @field_validator('session_id', mode='before')
    def validate_session_id(cls, v):
        if v is not None and len(str(v)) > 36:
            raise ValueError('Session ID must be 36 characters or less if provided')
        return v

# Response model
class QueryResponse(BaseModel):
    response: str
    session_id: Optional[str] = None
    success: bool
    error: Optional[str] = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifecycle events for the FastAPI application
    """
    logger.info("Starting RAG Agent API...")
    # Initialize any resources here if needed
    yield
    # Cleanup resources here if needed
    logger.info("Shutting down RAG Agent API...")

app = FastAPI(
    title="RAG Agent API",
    description="API for interacting with the RAG agent that retrieves book content",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",  # Swagger UI
    redoc_url="/redoc"  # ReDoc
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware to allow frontend connections
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URLs
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global agent instance
agent_instance = None

def get_agent():
    """
    Get or create the agent instance
    """
    global agent_instance
    if agent_instance is None:
        try:
            agent_instance = AIAgentRetrieval()
            logger.info("RAG Agent initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG Agent: {str(e)}")
            raise
    return agent_instance

@app.get("/")
async def read_root():
    """
    Root endpoint to verify the API is running
    """
    return {
        "message": "RAG Agent API is running",
        "status": "healthy"
    }

@app.post("/query", response_model=QueryResponse)
async def query_agent_endpoint(request: QueryRequest):
    """
    Process a user query using the RAG agent with retrieval capabilities
    """
    import time
    start_time = time.time()

    try:
        logger.info(f"Processing query: {request.query[:50]}...")

        # Get the agent instance
        agent = get_agent()

        # Process the query with the agent using asyncio run to handle event loop conflicts
        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(None, agent.query_agent, request.query)

        # Calculate response time
        response_time = round(time.time() - start_time, 3)
        logger.info(f"Query processed in {response_time}s")

        # Return the response
        return QueryResponse(
            response=response,
            session_id=request.session_id,
            success=True
        )

    except Exception as e:
        # Calculate response time for error case
        response_time = round(time.time() - start_time, 3)
        logger.error(f"Error processing query after {response_time}s: {str(e)}")
        return QueryResponse(
            response="",
            success=False,
            error=f"Error processing query: {str(e)}"
        )

@app.get("/health")
async def health_check(request: Request):
    """
    Health check endpoint to verify the API and agent are accessible
    """
    try:
        # Test basic agent functionality
        agent = get_agent()
        return {
            "status": "healthy",
            "agent": "available",
            "message": "RAG Agent is ready to process queries"
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "agent": "unavailable",
            "error": str(e)
        }


@app.get("/status")
async def status_check(request: Request):
    """
    Status endpoint to report agent status and system information
    """
    try:
        agent = get_agent()
        return {
            "status": "available",
            "agent_status": "ready",
            "api_version": "1.0.0",
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "message": "RAG Agent is operational and ready to process queries"
        }
    except Exception as e:
        logger.error(f"Status check failed: {str(e)}")
        return {
            "status": "unavailable",
            "agent_status": "error",
            "api_version": "1.0.0",
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "error": str(e)
        }

@app.get("/test")
async def test_agent(request: Request):
    """
    Test endpoint to verify the agent is working correctly
    """
    try:
        agent = get_agent()
        # Simple test query
        response = agent.query_agent("What is the purpose of this system?")
        return {
            "status": "success",
            "response": response[:200] + "..." if len(response) > 200 else response,
            "message": "Agent responded successfully"
        }
    except Exception as e:
        logger.error(f"Test query failed: {str(e)}")
        return {
            "status": "error",
            "error": str(e)
        }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)