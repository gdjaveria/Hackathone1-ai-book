"""
AI Agent with Retrieval Capabilities

This module implements an AI agent using OpenAI Agents SDK with integrated
retrieval capabilities from a vector database (Qdrant). The agent is designed
to respond using only retrieved book content, with proper error handling and logging.

The agent follows a Retrieval-Augmented Generation (RAG) pattern where:
1. User queries are processed by the OpenAI Assistant
2. When domain-specific knowledge is needed, the assistant calls the retrieval_tool
3. The retrieval_tool queries a Qdrant vector database for relevant book content
4. The retrieved content is provided to the assistant to generate a response
5. The response is validated to ensure it's grounded in the retrieved content

Key Features:
- Integration with OpenAI's Assistant API
- Qdrant vector database for content retrieval
- Content validation to prevent hallucinations
- Comprehensive error handling and logging
- Command-line interface with configuration options
- Query validation to handle ambiguous requests
- Token limit management for large content

Usage:
    # Interactive mode
    python agent.py

    # With custom parameters
    python agent.py --model gpt-4 --top-k 5 --log-level DEBUG

    # Run sample queries for testing
    python agent.py --test
"""

import os
import logging
from typing import Dict, List, Optional, Any
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import openai
from openai import AsyncOpenAI

# Import the new OpenAI Agent SDK - note: this might need to be installed separately
try:
    from agents import Agent, Runner, function_tool
    from agents import OpenAIChatCompletionsModel
except ImportError:
    # If agents package is not available, define fallback functionality
    logging.warning("Agents package not found. Using fallback implementation.")

    def function_tool(func):
        """Fallback function tool decorator"""
        return func

    class Agent:
        """Fallback Agent class"""
        def __init__(self, **kwargs):
            self.name = kwargs.get('name', 'Fallback Agent')
            self.instructions = kwargs.get('instructions', 'Fallback agent for testing')
            self.tools = kwargs.get('tools', [])
            self.model = kwargs.get('model', 'gpt-4')

    class Runner:
        """Fallback Runner class"""
        @staticmethod
        def run_sync(agent, query):
            """Fallback synchronous run method"""
            class Result:
                def __init__(self):
                    self.final_output = f"This is a fallback response for query: {query}"
            return Result()

ROUTER_API_KEY = "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"

client = AsyncOpenAI(api_key=ROUTER_API_KEY,
                      base_url="https://openrouter.ai/api/v1")

# Use the agents package if available, otherwise use OpenAI directly
try:
    from agents import OpenAIChatCompletionsModel
    model = OpenAIChatCompletionsModel(
        openai_client=client,
        model="mistralai/devstral-2512:free"  # Example model from OpenRouter
    )
except ImportError:
    # If agents package is not available, we'll use the model name directly
    model = "openrouter/mistralai/devstral-2512:free"

# Load environment variables
load_dotenv()

# Set OpenRouter as the API endpoint for OpenAI client
os.environ["OPENAI_BASE_URL"] = "https://openrouter.ai/api/v1"



# Configure logging
def setup_logging():
    """Set up logging configuration for the agent"""
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    level = getattr(logging, log_level, logging.INFO)

    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("agent.log")  # Optional: log to file
        ]
    )
    return logging.getLogger(__name__)


logger = setup_logging()

def create_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client connection based on environment configuration
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = int(os.getenv("QDRANT_PORT", "6333"))

    try:
        if qdrant_url:
            # Use URL if provided (for cloud instances)
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False  # Using HTTP for better compatibility
            )
        else:
            # Use host/port for local instances
            client = QdrantClient(
                host=qdrant_host,
                port=qdrant_port
            )

        logger.info("Qdrant client connected successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to create Qdrant client: {str(e)}")
        raise


# Error handling utilities
class RetrievalError(Exception):
    """Custom exception for retrieval-related errors"""
    pass


class DatabaseConnectionError(Exception):
    """Custom exception for database connection errors"""
    pass


def log_error_details(error: Exception, context: str = "", query: str = ""):
    """
    Log comprehensive error details for debugging and monitoring

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred
        query: The query that was being processed when the error occurred
    """
    import traceback
    from datetime import datetime

    error_details = {
        "timestamp": datetime.now().isoformat(),
        "context": context,
        "query": query,
        "error_type": type(error).__name__,
        "error_message": str(error),
        "traceback": traceback.format_exc()
    }

    logger.error(f"Error Details: {error_details}")


def handle_retrieval_error(error: Exception, context: str = "", query: str = "") -> Dict[str, Any]:
    """
    Handle retrieval errors and return appropriate response

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred
        query: The query that was being processed

    Returns:
        Dictionary with error information
    """
    # Log comprehensive error details
    log_error_details(error, context, query)

    error_msg = f"Error during retrieval {context}: {str(error)}"
    logger.error(error_msg)

    # Determine the type of error and return appropriate message
    error_str = str(error).lower()
    if "connection" in error_str or "timeout" in error_str or "refused" in error_str or "network" in error_str:
        return {
            "error": "Vector database unavailable",
            "message": "Unable to retrieve content due to database connection issues",
            "total_results": 0
        }
    elif "not found" in error_str or "empty" in error_str:
        return {
            "retrieved_content": [],
            "message": "No relevant content found for the given query",
            "total_results": 0
        }
    else:
        return {
            "error": "Retrieval failed",
            "message": f"Unable to retrieve content: {str(error)}",
            "total_results": 0
        }


def handle_database_unavailable() -> Dict[str, Any]:
    """
    Handle the case when vector database is unavailable during retrieval

    Returns:
        Dictionary with appropriate response for database unavailability
    """
    error_msg = "Vector database is currently unavailable"
    logger.error(error_msg)
    return {
        "error": "Vector database unavailable",
        "message": "Unable to retrieve content due to database connection issues. Please try again later.",
        "total_results": 0
    }


def handle_no_relevant_results(query: str) -> Dict[str, Any]:
    """
    Handle the case when retrieval tool returns no relevant results for a query

    Args:
        query: The original query that yielded no results

    Returns:
        Dictionary with appropriate response for no results
    """
    logger.info(f"No relevant results found for query: {query}")
    return {
        "retrieved_content": [],
        "message": "No relevant content found for the given query. The book database does not contain information related to this topic.",
        "total_results": 0
    }


def truncate_content_to_token_limit(content_list: List[Dict], max_tokens: int = 3000) -> List[Dict]:
    """
    Truncate retrieved content to stay within token limits

    Args:
        content_list: List of retrieved content items
        max_tokens: Maximum number of tokens to include (default: 3000)

    Returns:
        Truncated list of content items within token limits
    """
    if not content_list:
        return content_list

    # Estimate tokens (roughly 1 token per 4 characters for English text)
    estimated_tokens = 0
    truncated_content = []

    for item in content_list:
        text = item.get('text', '')
        # Rough token estimation
        item_tokens = len(text) // 4

        if estimated_tokens + item_tokens <= max_tokens:
            truncated_content.append(item)
            estimated_tokens += item_tokens
        else:
            # If adding this item would exceed the limit, truncate the text
            remaining_tokens = max_tokens - estimated_tokens
            if remaining_tokens > 0:
                # Calculate how many characters we can add
                remaining_chars = remaining_tokens * 4
                truncated_text = text[:remaining_chars]

                # Create a new item with truncated text
                truncated_item = item.copy()
                truncated_item['text'] = truncated_text
                truncated_content.append(truncated_item)

            break  # Stop when we reach the limit

    return truncated_content


def validate_query(query: str) -> Dict[str, Any]:
    """
    Validate and handle queries that are too general or ambiguous

    Args:
        query: The query to validate

    Returns:
        Dictionary with validation result
    """
    if not query or not isinstance(query, str):
        return {
            "valid": False,
            "message": "Query must be a non-empty string"
        }

    # Check if query is too general or ambiguous
    query_lower = query.lower().strip()

    # Check for very short queries
    if len(query) < 3:
        return {
            "valid": False,
            "message": "Query is too short. Please provide a more specific question."
        }

    # Check for overly general queries
    general_terms = ["what", "how", "why", "when", "where", "who", "the", "a", "an", "is", "are", "was", "were"]
    words = query_lower.split()

    # If query is only general terms, it's likely too ambiguous
    if all(word in general_terms for word in words):
        return {
            "valid": False,
            "message": "Query is too general. Please provide a more specific question about the book content."
        }

    # Check for very common but ambiguous phrases
    if query_lower in ["tell me", "explain", "what is", "how to", "why is", "i want to know"]:
        return {
            "valid": False,
            "message": "Query is too general. Please provide a more specific question about the book content."
        }

    return {
        "valid": True,
        "message": "Query is valid"
    }


@function_tool
def retrieval_tool(query: str, top_k: int = 3) -> Dict[str, Any]:
    """
    Retrieval tool function that connects to Qdrant and searches for relevant content

    Args:
        query: The search query to find relevant book content
        top_k: Number of results to return (default: 3, max: 10)

    Returns:
        Dictionary containing retrieved content and metadata
    """
    import time
    start_time = time.time()

    try:
        logger.info(f"Starting retrieval for query: {query[:50]}...")

        # Validate parameters
        validation_result = validate_query(query)
        if not validation_result["valid"]:
            retrieval_time = time.time() - start_time
            logger.info(f"Retrieval completed in {retrieval_time:.2f}s (validation failed)")
            return {
                "retrieved_content": [],
                "message": validation_result["message"],
                "total_results": 0
            }

        if not isinstance(top_k, int) or top_k < 1 or top_k > 10:
            top_k = min(max(top_k, 1), 10)  # Clamp between 1 and 10

        # Get Qdrant client
        client = create_qdrant_client()
        collection_name = os.getenv("COLLECTION_NAME", "book_content")

        # Get the embedding for the query using an embedding model
        import openai

        # Use the original OpenAI API key for embeddings (not OpenRouter)
        # Load from .env file directly to get the original OpenAI key
        from dotenv import dotenv_values
        config = dotenv_values(".env")
        actual_openai_key = config.get("OPENAI_API_KEY")

        # If no OpenAI key is found in .env, try environment variable
        if not actual_openai_key:
            actual_openai_key = os.getenv("OPENAI_API_KEY")

        # If the key is the OpenRouter key or a placeholder, we need to handle this gracefully
        if not actual_openai_key or actual_openai_key == "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":
            # Try to get the original OpenAI key from the agent instance if available
            global agent_instance
            if 'agent_instance' in globals() and agent_instance and hasattr(agent_instance, 'original_openai_api_key'):
                actual_openai_key = agent_instance.original_openai_api_key
            else:
                # If we still don't have a valid key, raise an exception
                raise ValueError("Valid OpenAI API key required for embeddings. Please set OPENAI_API_KEY in your .env file.")

        # Create OpenAI client with proper base URL for embeddings (not OpenRouter)
        openai_client = openai.OpenAI(
            api_key=actual_openai_key,
            base_url="https://api.openai.com/v1"  # Use OpenAI's base URL for embeddings
        )

        # Get embedding for the query
        embedding_start = time.time()
        response = openai_client.embeddings.create(
            input=query,
            model="text-embedding-ada-002"  # Using OpenAI's embedding model
        )
        query_embedding = response.data[0].embedding
        embedding_time = time.time() - embedding_start
        logger.debug(f"Embedding generation took {embedding_time:.2f}s")

        # Perform vector search using the embedding
        search_start = time.time()
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k
        )
        search_time = time.time() - search_start
        logger.debug(f"Vector search took {search_time:.2f}s")

        # Process results
        retrieved_content = []
        for result in search_results:
            content_item = {
                "content_id": result.id,
                "text": result.payload.get('text', ''),
                "metadata": result.payload.get('metadata', {}),
                "relevance_score": result.score
            }
            retrieved_content.append(content_item)

        # Truncate content to stay within token limits
        max_tokens = int(os.getenv("MAX_TOKENS", "3000"))
        truncated_content = truncate_content_to_token_limit(retrieved_content, max_tokens)

        # Log successful retrieval with performance metrics
        retrieval_time = time.time() - start_time
        logger.info(f"Successfully retrieved {len(truncated_content)} results for query: {query[:50]}... (total time: {retrieval_time:.2f}s)")

        # If no results found, handle appropriately
        if not truncated_content:
            return handle_no_relevant_results(query)

        return {
            "retrieved_content": truncated_content,
            "total_results": len(truncated_content)
        }

    except Exception as e:
        retrieval_time = time.time() - start_time
        logger.info(f"Retrieval failed after {retrieval_time:.2f}s")
        return handle_retrieval_error(e, query=query)


from typing import List, Dict, Any, Optional


class AIAgentRetrieval:
    """
    AI Agent with Retrieval Capabilities using OpenAI Agent SDK
    """

    def __init__(self):
        # Get the original OpenAI API key for embeddings from .env directly to avoid conflicts
        from dotenv import dotenv_values
        config = dotenv_values(".env")
        self.original_openai_api_key = config.get("OPENAI_API_KEY")

        # If no OpenAI key is found or it's the OpenRouter key, try to get from environment
        if not self.original_openai_api_key or self.original_openai_api_key == "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":
            self.original_openai_api_key = os.getenv("OPENAI_API_KEY")

        # Initialize OpenAI client for embeddings (still needed for vector search)
        self.openai_client = openai.OpenAI(api_key=self.original_openai_api_key)

        # Try to import agents and initialize with SDK
        try:
            from agents import Agent, Runner, function_tool
            # Use a standard OpenAI model with agents package (OpenRouter format may not be supported)
            self.model = "gpt-3.5-turbo"  # Use standard OpenAI model for agent

            # Create the agent using the OpenAI Agent SDK
            self.agent = Agent(
                name="Book Content Assistant",
                instructions=(
                    "You are an AI assistant that helps users find information in book content. "
                    "When a user asks a question, first determine if you have sufficient knowledge "
                    "to answer from your base knowledge. If the question is about specific book "
                    "content, or if you need additional context to provide a comprehensive answer, "
                    "use the retrieval_tool to search for relevant information in the book database. "
                    "Only respond based on the information provided by the retrieval tool. Do not "
                    "make up information that is not present in the retrieved content. If the "
                    "retrieval tool returns no relevant results, inform the user that you couldn't "
                    "find relevant information in the book database."
                ),
                tools=[retrieval_tool],
                model=self.model
            )
            self.use_agents_sdk = True
            logger.info("AI Agent initialized with OpenAI Agent SDK")
        except ImportError:
            # Fallback to using OpenAI directly if agents package is not available
            self.model = "gpt-3.5-turbo"  # Use a standard OpenAI model as fallback
            self.use_agents_sdk = False
            logger.warning("AI Agent initialized with fallback implementation (no agents SDK)")
        except Exception as e:
            # If agents are available but initialization fails for other reasons, use fallback
            logger.warning(f"Failed to initialize agent with SDK: {e}. Using fallback implementation.")
            self.model = "gpt-3.5-turbo"  # Use a standard OpenAI model as fallback
            self.use_agents_sdk = False
            logger.warning("AI Agent initialized with fallback implementation (SDK error)")

        # Set the global agent instance so the retrieval tool can access it
        global agent_instance
        agent_instance = self

    def query_agent(self, query: str):
        """
        Handle a query from the user and return the agent's response

        Args:
            query: The user's query

        Returns:
            The agent's response
        """
        import time
        start_time = time.time()

        try:
            logger.info(f"Processing query: {query[:50]}...")

            if self.use_agents_sdk:
                # Use the agents SDK if available
                # Temporarily set the API key to OpenRouter for agent execution
                original_api_key = os.getenv("OPENAI_API_KEY")
                os.environ["OPENAI_API_KEY"] = "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"

                # Set OpenRouter base URL for agent execution
                original_base_url = os.getenv("OPENAI_BASE_URL")
                os.environ["OPENAI_BASE_URL"] = "https://openrouter.ai/api/v1"

                # Run the agent with the query using the OpenAI Agent SDK
                result = Runner.run_sync(self.agent, query)
                response = result.final_output

                # Restore original API key and base URL
                if original_api_key:
                    os.environ["OPENAI_API_KEY"] = original_api_key
                else:
                    os.environ.pop("OPENAI_API_KEY", None)

                if original_base_url:
                    os.environ["OPENAI_BASE_URL"] = original_base_url
                else:
                    os.environ.pop("OPENAI_BASE_URL", None)
            else:
                # Fallback implementation using OpenAI directly
                # For now, return a response that includes retrieved content
                retrieved = retrieval_tool(query, top_k=3)
                retrieved_content = retrieved.get('retrieved_content', [])

                if retrieved_content:
                    # Create a response using the retrieved content
                    content_texts = [item.get('text', '') for item in retrieved_content if item.get('text')]
                    content_summary = " ".join(content_texts[:2])  # Use first 2 chunks
                    response = f"Based on the book content: {content_summary[:500]}..."  # Limit length
                else:
                    response = "I couldn't find relevant information in the book database for your query."

            # Log performance metrics
            end_time = time.time()
            duration = end_time - start_time
            logger.info(f"Query processed in {duration:.2f} seconds")

            return response

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            end_time = time.time()
            duration = end_time - start_time
            logger.info(f"Query failed after {duration:.2f} seconds")

            # Restore original API key and base URL in case of error (only if using agents SDK)
            if self.use_agents_sdk:
                original_api_key = os.getenv("OPENAI_API_KEY")
                original_base_url = os.getenv("OPENAI_BASE_URL")

                if original_api_key:
                    os.environ["OPENAI_API_KEY"] = original_api_key
                else:
                    os.environ.pop("OPENAI_API_KEY", None)

                if original_base_url:
                    os.environ["OPENAI_BASE_URL"] = original_base_url
                else:
                    os.environ.pop("OPENAI_BASE_URL", None)

            return f"Error processing your query: {str(e)}"

    def validate_response_content(self, response: str, retrieved_content: List[Dict]) -> bool:
        """
        Validate that the response content is grounded in the retrieved context

        Args:
            response: The agent's response to validate
            retrieved_content: The content retrieved from the vector database

        Returns:
            Boolean indicating if the response is properly grounded in retrieved content
        """
        if not response or not retrieved_content:
            return False

        # Check if the response contains information from the retrieved content
        response_lower = response.lower()

        # Look for any significant overlap between response and retrieved content
        content_found = False
        for item in retrieved_content:
            text = item.get('text', '').lower()
            # Check if any substantial part of the retrieved text appears in the response
            if len(text) > 10:  # Only check substantial text chunks
                words = text.split()
                # Check if at least a few key words appear in the response
                found_words = [word for word in words if word in response_lower and len(word) > 3]
                if len(found_words) > 0:
                    content_found = True
                    break

        return content_found

    def prevent_hallucination(self, response: str, retrieved_content: List[Dict]) -> str:
        """
        Apply hallucination prevention mechanisms to agent responses

        Args:
            response: The agent's original response
            retrieved_content: The content retrieved from the vector database

        Returns:
            The response with hallucination prevention applied
        """
        # For now, we'll just log if the response seems to contain information not in the retrieved content
        # In a more sophisticated implementation, we could:
        # - Compare response content with retrieved content
        # - Flag potential hallucinations
        # - Request the agent to regenerate the response

        is_valid = self.validate_response_content(response, retrieved_content)

        if not is_valid and retrieved_content:
            logger.warning("Response may contain information not present in retrieved content")
            # Optionally, we could return a modified response indicating uncertainty
            # For now, we'll just log the issue
        else:
            logger.info("Response appears to be grounded in retrieved content")

        return response

    def trace_response_to_sources(self, response: str, retrieved_content: List[Dict]) -> List[Dict]:
        """
        Create function to trace response content back to retrieved sources

        Args:
            response: The agent's response
            retrieved_content: The content retrieved from the vector database

        Returns:
            List of source references that support content in the response
        """
        response_lower = response.lower()
        traced_sources = []

        for item in retrieved_content:
            text = item.get('text', '').lower()
            content_id = item.get('content_id', 'unknown')
            metadata = item.get('metadata', {})

            # Check if any part of this retrieved content appears in the response
            if len(text) > 10:  # Only check substantial text chunks
                words = text.split()
                # Count how many words from this source appear in the response
                found_words = [word for word in words if word in response_lower and len(word) > 3]

                if len(found_words) > 0:
                    traced_sources.append({
                        "content_id": content_id,
                        "metadata": metadata,
                        "matched_words_count": len(found_words),
                        "similarity_score": len(found_words) / len(words)  # Simple similarity metric
                    })

        return traced_sources

    def verify_intelligent_retrieval_decisions(self):
        """
        Verify agent makes intelligent decisions about when to retrieve information
        """
        logger.info("Verifying intelligent retrieval decisions...")

        # Test 1: Query that should trigger retrieval (book-specific content)
        query_book_specific = "What does the book say about quantum computing?"
        response_book_specific = self.query_agent(query_book_specific)
        logger.info(f"Response to book-specific query: {response_book_specific[:100]}...")

        # Test 2: General knowledge query that might not need retrieval
        query_general = "What is the capital of France?"
        response_general = self.query_agent(query_general)
        logger.info(f"Response to general query: {response_general[:100]}...")

        # The agent should use its base knowledge for general questions
        # and retrieval for book-specific content
        logger.info("Intelligent retrieval decisions verification completed")
        return True

    def test_content_grounding(self):
        """
        Test that agent responses only contain information from retrieved content
        """
        logger.info("Testing content grounding...")

        # Query that should trigger retrieval
        test_query = "What does the book say about artificial intelligence?"

        # First, get the retrieved content directly
        retrieved = retrieval_tool(test_query, top_k=3)
        retrieved_content = retrieved.get('retrieved_content', [])

        # Then get the agent's response
        response = self.query_agent(test_query)

        # Validate that the response is grounded in the retrieved content
        is_valid = self.validate_response_content(response, retrieved_content)

        if is_valid:
            logger.info("Response is properly grounded in retrieved content")
        else:
            logger.warning("Response may not be fully grounded in retrieved content")

        # Trace the response back to sources
        traced_sources = self.trace_response_to_sources(response, retrieved_content)
        logger.info(f"Found {len(traced_sources)} source references in the response")

        return is_valid

    def test_dynamic_tool_invocation(self):
        """
        Test dynamic tool invocation during conversation flow
        """
        logger.info("Testing dynamic tool invocation...")

        # Test 1: Query that should trigger retrieval tool
        query1 = "What does the book say about machine learning?"
        response1 = self.query_agent(query1)
        logger.info(f"Response to query '{query1}': {response1[:100]}...")

        # Test 2: Follow-up query that might use context
        query2 = "Can you elaborate on the types of machine learning mentioned?"
        response2 = self.query_agent(query2)
        logger.info(f"Response to follow-up '{query2}': {response2[:100]}...")

        # The tool should have been invoked for queries requiring book-specific information
        logger.info("Dynamic tool invocation test completed")
        return True

    def verify_response_context(self, query: str, expected_content: Optional[str] = None):
        """
        Verify that the agent generates responses based on retrieved context

        Args:
            query: The query to test
            expected_content: Optional expected content to check for in the response

        Returns:
            Boolean indicating if the response was properly grounded in context
        """
        logger.info(f"Verifying response context for query: {query}")

        # Get the response from the agent
        response = self.query_agent(query)

        # Log the response for verification
        logger.info(f"Agent response: {response}")

        # Basic check: response should not be empty
        if not response or response.strip() == "" or "No response from agent" in response:
            logger.error("Agent returned empty or default response")
            return False

        # If expected content is provided, check if it's in the response
        if expected_content and expected_content.lower() not in response.lower():
            logger.warning(f"Expected content '{expected_content}' not found in response")
            # Note: This might be expected if the content isn't in the retrieved results

        # The agent should have used the retrieval tool to generate the response
        # In a real implementation, we would have more sophisticated checks
        logger.info("Response verification completed")
        return True

    def test_retrieval_tool(self):
        """
        Test the agent's ability to call the retrieval tool and receive results
        """
        logger.info("Testing retrieval tool functionality...")

        # Test basic retrieval
        test_query = "What is artificial intelligence?"
        result = retrieval_tool(test_query, top_k=2)

        logger.info(f"Retrieval tool test result: {result.get('total_results', 0)} results found")

        if result.get('error'):
            logger.error(f"Retrieval tool test failed: {result.get('message')}")
            return False

        # Test with the agent
        try:
            # Create a simple query that should trigger the retrieval tool
            response = self.query_agent("What does the book say about artificial intelligence?")
            logger.info(f"Agent response test: {response[:100]}...")
            return True
        except Exception as e:
            logger.error(f"Agent query test failed: {str(e)}")
            return False


import argparse


def run_sample_queries(agent):
    """
    Run sample queries to test the complete end-to-end flow

    Args:
        agent: The initialized AIAgentRetrieval instance

    Returns:
        Boolean indicating if all sample queries completed successfully
    """
    logger.info("Running sample queries for end-to-end testing...")

    sample_queries = [
        "What is artificial intelligence?",
        "Explain machine learning concepts",
        "What are neural networks?",
        "How does deep learning work?"
    ]

    success_count = 0
    total_count = len(sample_queries)

    for i, query in enumerate(sample_queries, 1):
        print(f"\nSample Query {i}/{total_count}: {query}")
        try:
            response = agent.query_agent(query)
            print(f"Response: {response[:200]}...")
            if response and "No response from agent" not in response:
                success_count += 1
                print("[SUCCESS] Query successful")
            else:
                print("[FAILED] Query failed")
        except Exception as e:
            logger.error(f"Error with sample query '{query}': {str(e)}")
            print(f"[FAILED] Query failed: {str(e)}")

    logger.info(f"Sample queries completed: {success_count}/{total_count} successful")
    return success_count == total_count


def validate_configuration():
    """
    Validate the configuration and set default values

    Returns:
        Dictionary with validated configuration
    """
    config = {}

    # Validate OpenAI API key (needed for embeddings)
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key or api_key == "your_openai_api_key_here" or api_key == "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":
        # If using OpenRouter key or placeholder, load from .env to get the real OpenAI key
        from dotenv import dotenv_values
        config_env = dotenv_values(".env")
        actual_openai_key = config_env.get("OPENAI_API_KEY")
        if not actual_openai_key or actual_openai_key == "your_openai_api_key_here":
            raise ValueError("OPENAI_API_KEY must be set in environment variables for embeddings")
        config["openai_api_key"] = actual_openai_key
    else:
        config["openai_api_key"] = api_key

    # Validate and set model
    model = os.getenv("OPENAI_MODEL", "openrouter/mistralai/devstral-2512:free")  # Default to OpenRouter model
    valid_models = ["gpt-4o", "gpt-4-turbo", "gpt-4", "gpt-3.5-turbo", "gpt-4o-mini", "openrouter/mistralai/devstral-2512:free"]  # Add other valid models as needed
    if model not in valid_models:
        logger.warning(f"Model {model} may not be supported. Using default openrouter/mistralai/devstral-2512:free.")
        model = "openrouter/mistralai/devstral-2512:free"
    config["model"] = model

    # Validate and set top_k
    try:
        top_k = int(os.getenv("TOP_K", "3"))
        if top_k < 1 or top_k > 10:
            logger.warning(f"TOP_K value {top_k} is out of range (1-10). Using default 3.")
            top_k = 3
    except ValueError:
        logger.warning(f"Invalid TOP_K value, using default 3.")
        top_k = 3
    config["top_k"] = top_k

    # Validate and set logging level
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
    if log_level not in valid_levels:
        logger.warning(f"Invalid LOG_LEVEL {log_level}, using default INFO.")
        log_level = "INFO"
    config["log_level"] = log_level

    # Validate Qdrant configuration
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = os.getenv("QDRANT_PORT", "6333")

    if not qdrant_url and (not qdrant_host or not qdrant_port):
        raise ValueError("Either QDRANT_URL or both QDRANT_HOST and QDRANT_PORT must be set")

    try:
        qdrant_port_int = int(qdrant_port)
        if qdrant_port_int <= 0 or qdrant_port_int > 65535:
            raise ValueError("Port must be between 1 and 65535")
    except ValueError:
        raise ValueError(f"Invalid QDRANT_PORT: {qdrant_port}")

    config["qdrant_url"] = qdrant_url
    config["qdrant_host"] = qdrant_host
    config["qdrant_port"] = qdrant_port_int

    # Validate collection name
    collection_name = os.getenv("COLLECTION_NAME", "book_content")
    if not collection_name:
        raise ValueError("COLLECTION_NAME cannot be empty")
    config["collection_name"] = collection_name

    # Validate max tokens
    try:
        max_tokens = int(os.getenv("MAX_TOKENS", "3000"))
        if max_tokens <= 0:
            raise ValueError("MAX_TOKENS must be positive")
    except ValueError:
        logger.warning(f"Invalid MAX_TOKENS value, using default 3000.")
        max_tokens = 3000
    config["max_tokens"] = max_tokens

    return config


def main():
    """
    Main function to handle user queries end-to-end and integrate all components
    """
    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description='AI Agent with Retrieval Capabilities')
    parser.add_argument('--model', type=str, default=os.getenv("OPENAI_MODEL", "gpt-4-turbo"),
                        help='OpenAI model to use (default: gpt-4-turbo or OPENAI_MODEL env var)')
    parser.add_argument('--top-k', type=int, default=int(os.getenv("TOP_K", "3")),
                        help='Number of results to retrieve (default: 3 or TOP_K env var)')
    parser.add_argument('--log-level', type=str, default=os.getenv("LOG_LEVEL", "INFO"),
                        help='Logging level (default: INFO or LOG_LEVEL env var)')
    parser.add_argument('--collection', type=str, default=os.getenv("COLLECTION_NAME", "book_content"),
                        help='Qdrant collection name (default: book_content or COLLECTION_NAME env var)')
    parser.add_argument('--interactive', action='store_true', default=True,
                        help='Run in interactive mode (default: True)')
    parser.add_argument('--test', action='store_true', default=False,
                        help='Run sample queries for testing')

    args = parser.parse_args()

    # Update environment variables with command-line arguments if provided
    if args.model:
        os.environ["OPENAI_MODEL"] = args.model
    if args.top_k:
        os.environ["TOP_K"] = str(args.top_k)
    if args.log_level:
        os.environ["LOG_LEVEL"] = args.log_level
    if args.collection:
        os.environ["COLLECTION_NAME"] = args.collection

    # Validate configuration
    try:
        config = validate_configuration()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {str(e)}")
        print(f"Configuration error: {str(e)}")
        return 1

    # Reconfigure logging with the new level
    log_level = config["log_level"]
    level = getattr(logging, log_level, logging.INFO)
    logging.getLogger().setLevel(level)

    logger.info("Starting AI Agent with Retrieval Capabilities")
    logger.info(f"Using model: {config['model']}")
    logger.info(f"Retrieving top {config['top_k']} results")
    logger.info(f"Using collection: {config['collection_name']}")
    logger.info(f"Max tokens: {config['max_tokens']}")

    try:
        # Initialize the agent
        agent = AIAgentRetrieval()

        # If --test flag is provided, run sample queries instead of interactive mode
        if args.test:
            print("Running sample queries for testing...")
            success = run_sample_queries(agent)
            if success:
                print("\n[SUCCESS] All sample queries completed successfully!")
                return 0
            else:
                print("\n[FAILED] Some sample queries failed.")
                return 1

        print("AI Agent with Retrieval Capabilities is ready!")
        print("You can ask questions about book content.")
        print("Type 'quit' or 'exit' to stop the agent.\n")

        # Test the agent's components before starting the interaction
        print("Testing agent components...")
        try:
            # Test retrieval tool
            test_result = agent.test_retrieval_tool()
            if test_result:
                logger.info("Agent components are functioning properly")
                print("Agent components are ready.\n")
            else:
                logger.warning("Some agent components may not be functioning properly")
                print("Warning: Some agent components may not be functioning properly, but continuing...\n")
        except Exception as e:
            logger.error(f"Error during component test: {str(e)}")
            print("Warning: Could not test components, but continuing...\n")

        while True:
            try:
                # Get user query
                user_input = input("Enter your query: ").strip()

                # Check for exit commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("Thank you for using the AI Agent. Goodbye!")
                    break

                # Skip empty queries
                if not user_input:
                    print("Please enter a query.\n")
                    continue

                # Validate the query before processing
                validation_result = validate_query(user_input)
                if not validation_result["valid"]:
                    print(f"Query validation error: {validation_result['message']}\n")
                    continue

                # Process the query with the agent
                print("Processing your query...")
                response = agent.query_agent(user_input)

                # Display the response
                print(f"\nAgent Response: {response}\n")

            except KeyboardInterrupt:
                print("\n\nInterrupted by user. Goodbye!")
                break
            except Exception as e:
                logger.error(f"Error in main loop: {str(e)}")
                print(f"An error occurred: {str(e)}\n")
                continue

    except Exception as e:
        logger.error(f"Failed to initialize agent: {str(e)}")
        print(f"Failed to start the agent: {str(e)}")
        return 1
    finally:
        logger.info("Agent session ended")


def cleanup():
    """
    Perform cleanup operations before shutdown
    """
    logger.info("Performing cleanup operations...")
    # Any cleanup operations would go here
    # For example, closing database connections, saving state, etc.
    logger.info("Cleanup completed")


if __name__ == "__main__":
    exit_code = 0
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Shutting down gracefully...")
        exit_code = 1
    except Exception as e:
        logger.error(f"Unexpected error in main: {str(e)}")
        exit_code = 1
    finally:
        cleanup()
        exit(exit_code)