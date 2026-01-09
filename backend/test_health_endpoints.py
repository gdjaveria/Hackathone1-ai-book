#!/usr/bin/env python3
"""
Test script for health and status endpoints of the RAG Agent API
"""

import asyncio
import sys
import os

# Add the project root to the path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from api import health_check, status_check, get_agent

def test_health_endpoints():
    """Test health and status endpoints"""
    print("Testing health and status endpoints...")

    try:
        # Test health check
        print("\nTesting health endpoint...")
        health_result = asyncio.run(health_check())
        print(f"✓ Health check result: {health_result}")

        # Verify health result structure
        assert 'status' in health_result
        assert 'agent' in health_result
        assert health_result['status'] in ['healthy', 'unhealthy']
        print("✓ Health result structure is correct")

        # Test status check
        print("\nTesting status endpoint...")
        status_result = asyncio.run(status_check())
        print(f"✓ Status check result: {status_result}")

        # Verify status result structure
        assert 'status' in status_result
        assert 'agent_status' in status_result
        assert 'api_version' in status_result
        assert 'timestamp' in status_result
        print("✓ Status result structure is correct")

        # Test agent connectivity through health check
        print("\nTesting agent connectivity...")
        agent = get_agent()
        print(f"✓ Agent is accessible: {agent is not None}")

        print("\n✓ All health and status endpoint tests passed!")
        return True

    except Exception as e:
        print(f"✗ Test failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_health_endpoints()
    if success:
        print("\n[SUCCESS] Health and status endpoints working correctly")
        sys.exit(0)
    else:
        print("\n[ERROR] Health and status endpoints have issues")
        sys.exit(1)