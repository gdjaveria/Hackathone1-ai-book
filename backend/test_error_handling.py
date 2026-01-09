#!/usr/bin/env python3
"""
Test script for error handling in the RAG Agent API
"""

import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from api import QueryRequest
from pydantic import ValidationError

def test_error_handling():
    """Test error handling for invalid inputs"""
    print("Testing error handling...")

    try:
        # Test valid query
        valid_request = QueryRequest(query="What is AI?")
        print(f"✓ Valid query accepted: {valid_request.query}")

        # Test empty query error
        try:
            empty_request = QueryRequest(query="")
            print("✗ Empty query validation failed")
            return False
        except ValidationError as e:
            print(f"✓ Empty query correctly rejected: {e}")

        # Test whitespace-only query error
        try:
            whitespace_request = QueryRequest(query="   ")
            print("✗ Whitespace-only query validation failed")
            return False
        except ValidationError as e:
            print(f"✓ Whitespace-only query correctly rejected: {e}")

        # Test too-long query error
        try:
            long_request = QueryRequest(query="a" * 1001)
            print("✗ Long query validation failed")
            return False
        except ValidationError as e:
            print(f"✓ Long query correctly rejected: {e}")

        # Test valid session_id
        valid_with_session = QueryRequest(query="What is AI?", session_id="test-session")
        print(f"✓ Valid query with session accepted: {valid_with_session.session_id}")

        # Test too-long session_id error
        try:
            long_session_request = QueryRequest(query="What is AI?", session_id="a" * 40)
            print("✗ Long session ID validation failed")
            return False
        except ValidationError as e:
            print(f"✓ Long session ID correctly rejected: {e}")

        print("\n✓ All error handling tests passed!")
        return True

    except Exception as e:
        print(f"✗ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_error_handling()
    if success:
        print("\n[SUCCESS] Error handling working correctly")
        sys.exit(0)
    else:
        print("\n[ERROR] Error handling has issues")
        sys.exit(1)