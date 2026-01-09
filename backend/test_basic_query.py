#!/usr/bin/env python3
"""
Test script for basic query functionality of the RAG Agent API
"""

import asyncio
import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from api import QueryRequest, get_agent

def test_basic_query():
    """Test basic query functionality"""
    print("Testing basic query functionality...")

    try:
        # Test valid query
        valid_request = QueryRequest(query="What is physical AI?")
        print(f"✓ Valid query created: {valid_request.query}")

        # Test query validation
        try:
            empty_request = QueryRequest(query="   ")  # Should fail
            print("✗ Empty query validation failed")
        except Exception as e:
            print(f"✓ Empty query validation works: {str(e)}")

        try:
            long_request = QueryRequest(query="a" * 1001)  # Should fail
            print("✗ Long query validation failed")
        except Exception as e:
            print(f"✓ Long query validation works: {str(e)}")

        # Test with valid session_id
        valid_request_with_session = QueryRequest(query="What is AI?", session_id="test-session-123")
        print(f"✓ Valid query with session created: {valid_request_with_session.session_id}")

        # Test with too long session_id
        try:
            invalid_session_request = QueryRequest(query="What is AI?", session_id="a" * 40)  # Should fail
            print("✗ Session ID validation failed")
        except Exception as e:
            print(f"✓ Session ID validation works: {str(e)}")

        # Test agent integration
        print("\nTesting agent integration...")
        agent = get_agent()
        response = agent.query_agent("What is artificial intelligence?")
        print(f"✓ Agent responded successfully (first 100 chars): {response[:100]}...")

        print("\n✓ All basic query functionality tests passed!")
        return True

    except Exception as e:
        print(f"✗ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_basic_query()
    if success:
        print("\n[SUCCESS] Basic query functionality working correctly")
        sys.exit(0)
    else:
        print("\n[ERROR] Basic query functionality has issues")
        sys.exit(1)