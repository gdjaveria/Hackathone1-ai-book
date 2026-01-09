#!/usr/bin/env python3
"""
Debug script to check payload data in different Qdrant operations
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from backend.config import Config
from backend.storage import QdrantStorage
from backend.embedding import generate_single_embedding

def debug_payloads():
    config = Config()
    storage = QdrantStorage()

    # Get a sample point using scroll (which we know works)
    print("=== Testing scroll method ===")
    scroll_result = storage.client.scroll(
        collection_name=storage.collection_name,
        limit=1
    )
    points, _ = scroll_result
    if points:
        sample_point = points[0]
        print(f"Scroll - Point ID: {sample_point.id}")
        print(f"Scroll - Payload keys: {list(sample_point.payload.keys())}")
        print(f"Scroll - URL: {sample_point.payload.get('url', 'MISSING')}")
        print(f"Scroll - Title: {sample_point.payload.get('title', 'MISSING')}")
        print(f"Scroll - Source: {sample_point.payload.get('source', 'MISSING')}")
        print()

    # Now test query_points method
    print("=== Testing query_points method ===")
    test_embedding = generate_single_embedding("machine learning")
    query_results = storage.client.query_points(
        collection_name=storage.collection_name,
        query=test_embedding,
        limit=1,
        with_payload=True
    )

    if query_results.points:
        query_point = query_results.points[0]
        print(f"Query - Point ID: {query_point.id}")
        print(f"Query - Payload keys: {list(query_point.payload.keys())}")
        print(f"Query - URL: {query_point.payload.get('url', 'MISSING')}")
        print(f"Query - Title: {query_point.payload.get('title', 'MISSING')}")
        print(f"Query - Source: {query_point.payload.get('source', 'MISSING')}")
        print()

    # Also test retrieve method
    print("=== Testing retrieve method ===")
    if points:
        retrieved = storage.client.retrieve(
            collection_name=storage.collection_name,
            ids=[sample_point.id],
            with_payload=True
        )
        if retrieved:
            retrieved_point = retrieved[0]
            print(f"Retrieve - Point ID: {retrieved_point.id}")
            print(f"Retrieve - Payload keys: {list(retrieved_point.payload.keys())}")
            print(f"Retrieve - URL: {retrieved_point.payload.get('url', 'MISSING')}")
            print(f"Retrieve - Title: {retrieved_point.payload.get('title', 'MISSING')}")
            print(f"Retrieve - Source: {retrieved_point.payload.get('source', 'MISSING')}")

if __name__ == "__main__":
    debug_payloads()