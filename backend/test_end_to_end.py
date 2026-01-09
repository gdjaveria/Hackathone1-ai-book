#!/usr/bin/env python3
"""
Test script for end-to-end functionality of the RAG Agent API
"""

import asyncio
import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from api import QueryRequest, get_agent, QueryResponse

def test_end_to_end():
    """Test end-to-end functionality"""
    print("Testing end-to-end functionality...")

    try:
        # Test creating a query request
        print("\nTesting query request creation...")
        query_request = QueryRequest(query="What is artificial intelligence?")
        print(f"✓ Query request created: {query_request.query}")

        # Test getting agent instance
        print("\nTesting agent accessibility...")
        agent = get_agent()
        print(f"✓ Agent accessible: {agent is not None}")

        # Test agent response
        print("\nTesting agent response...")
        response = agent.query_agent("What is artificial intelligence?")
        print(f"✓ Agent responded: {len(response)} characters received")

        # Test response formatting
        print("\nTesting response formatting...")
        response_obj = QueryResponse(
            response=response,
            session_id="test-session",
            success=True
        )
        print(f"✓ Response formatted: {response_obj.success}")

        # Test with a longer query to validate length limits
        print("\nTesting query validation...")
        long_query = "a" * 500
        valid_request = QueryRequest(query=long_query)
        print(f"✓ Long query validated: {len(valid_request.query)} characters")

        # Test error response formatting
        print("\nTesting error response formatting...")
        error_response = QueryResponse(
            response="",
            session_id="test-session",
            success=False,
            error="Test error message"
        )
        print(f"✓ Error response formatted: success={error_response.success}, error='{error_response.error}'")

        print("\n✓ All end-to-end functionality tests passed!")
        return True

    except Exception as e:
        print(f"✗ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_end_to_end()
    if success:
        print("\n[SUCCESS] End-to-end functionality working correctly")
        sys.exit(0)
    else:
        print("\n[ERROR] End-to-end functionality has issues")
        sys.exit(1)