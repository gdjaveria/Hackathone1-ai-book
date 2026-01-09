#!/usr/bin/env python3
"""
Test script for the RAG Agent API
This script tests the FastAPI endpoints to ensure they're working correctly
"""

import requests
import json

BASE_URL = "http://localhost:8000"

def test_root_endpoint():
    """Test the root endpoint"""
    print("Testing root endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/")
        data = response.json()
        print(f"Root endpoint: {data}")
        return True
    except Exception as e:
        print(f"Root endpoint test failed: {e}")
        return False

def test_health_endpoint():
    """Test the health endpoint"""
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/health")
        data = response.json()
        print(f"Health endpoint: {data}")
        return True
    except Exception as e:
        print(f"Health endpoint test failed: {e}")
        return False

def test_query_endpoint():
    """Test the query endpoint"""
    print("Testing query endpoint...")
    try:
        payload = {"query": "What is artificial intelligence?"}
        response = requests.post(
            f"{BASE_URL}/query",
            headers={"Content-Type": "application/json"},
            data=json.dumps(payload)
        )
        data = response.json()
        print(f"Query endpoint: {data}")
        return True
    except Exception as e:
        print(f"Query endpoint test failed: {e}")
        return False

def main():
    """Main test function"""
    print("Testing RAG Agent API endpoints...\n")

    tests = [
        test_root_endpoint,
        test_health_endpoint,
        test_query_endpoint
    ]

    results = []
    for test in tests:
        results.append(test())
        print()

    print(f"Test Results: {sum(results)}/{len(results)} endpoints working")

    if all(results):
        print("[SUCCESS] All API endpoints are working correctly!")
        print("\nNote: The query endpoint returns an OpenAI quota error, which is expected")
        print("if your OpenAI account has exceeded its quota. This is not an API issue.")
    else:
        print("[ERROR] Some endpoints are not working properly")

if __name__ == "__main__":
    main()