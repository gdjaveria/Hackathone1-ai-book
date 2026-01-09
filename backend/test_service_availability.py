#!/usr/bin/env python3
"""
Test script for service availability in the RAG Agent API
"""

import asyncio
import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from api import health_check, status_check, get_agent

def test_service_availability():
    """Test service availability and error handling"""
    print("Testing service availability...")

    try:
        # Test that services are available
        print("\nTesting service availability...")

        # Test health check
        health_result = asyncio.run(health_check())
        print(f"✓ Health check: {health_result['status']}")

        # Test status check
        status_result = asyncio.run(status_check())
        print(f"✓ Status check: {status_result['status']}")

        # Test agent accessibility
        agent = get_agent()
        print(f"✓ Agent is accessible: {agent is not None}")

        # Test that the API can handle normal operations
        print("\nTesting normal operations...")

        # Verify health check returns proper structure
        required_health_fields = ['status', 'agent', 'message']
        for field in required_health_fields:
            assert field in health_result, f"Missing field {field} in health result"
        print("✓ Health check has all required fields")

        # Verify status check returns proper structure
        required_status_fields = ['status', 'agent_status', 'api_version', 'timestamp', 'message']
        for field in required_status_fields:
            assert field in status_result, f"Missing field {field} in status result"
        print("✓ Status check has all required fields")

        print("\n✓ All service availability tests passed!")
        return True

    except Exception as e:
        print(f"✗ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_service_availability()
    if success:
        print("\n[SUCCESS] Service availability working correctly")
        sys.exit(0)
    else:
        print("\n[ERROR] Service availability has issues")
        sys.exit(1)