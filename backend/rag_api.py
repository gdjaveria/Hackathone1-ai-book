"""
RAG API - Backend API for the Retrieval-Augmented Generation system
"""
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import logging
import os
import asyncio
from contextlib import asynccontextmanager
from datetime import datetime
import sys
import time

# Add the project root to the path to import agent module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import from the main directory since agent.py is there
from agent import AIAgentRetrieval

# Import backend models and config
from backend.models import QueryRequest, QueryResponse, RetrievedTextChunk
from backend.config import Config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Request model specific to this API
class AgentQueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None

# Response model specific to this API
class AgentQueryResponse(BaseModel):
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
        "status": "healthy",
        "backend": "available"
    }

@app.post("/query", response_model=AgentQueryResponse)
async def query_agent_endpoint(request: AgentQueryRequest):
    """
    Process a user query using the RAG agent with retrieval capabilities
    """
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
        return AgentQueryResponse(
            response=response,
            session_id=request.session_id,
            success=True
        )

    except Exception as e:
        # Calculate response time for error case
        response_time = round(time.time() - start_time, 3)
        logger.error(f"Error processing query after {response_time}s: {str(e)}")
        return AgentQueryResponse(
            response="",
            success=False,
            error=f"Error processing query: {str(e)}"
        )

@app.get("/health")
async def health_check():
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
async def status_check():
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
async def test_agent():
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
    uvicorn.run(app, host=Config.API_HOST, port=Config.API_PORT)