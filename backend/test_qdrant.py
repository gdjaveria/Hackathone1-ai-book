#!/usr/bin/env python3
"""
Test script to check Qdrant client API
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from backend.config import Config
from backend.storage import QdrantStorage

def test_qdrant_api():
    config = Config()

    # Validate configuration
    config_errors = config.validate()
    if config_errors:
        print("Configuration validation failed:")
        for error in config_errors:
            print(f"  - {error}")
        return False

    storage = QdrantStorage()

    # Check available methods
    print("Available methods in QdrantClient:")
    methods = [method for method in dir(storage.client) if 'search' in method.lower()]
    print(methods)

    # Test if search method exists
    if hasattr(storage.client, 'search'):
        print("✓ search method exists")
    else:
        print("✗ search method does not exist")

    # Try a simple search
    try:
        # Get collection info first
        collection_info = storage.client.get_collection(storage.collection_name)
        print(f"Collection {storage.collection_name} has {collection_info.points_count} points")

        # Try to get a sample point to use for testing
        if collection_info.points_count > 0:
            scroll_result = storage.client.scroll(
                collection_name=storage.collection_name,
                limit=1
            )
            points, _ = scroll_result
            if points:
                sample_point = points[0]
                print(f"Sample point ID: {sample_point.id}")

                # Try searching with a random vector
                import random
                test_vector = [random.random() for _ in range(1024)]  # 1024-dim vector

                print("Attempting search with test vector...")
                search_results = storage.client.search(
                    collection_name=storage.collection_name,
                    query_vector=test_vector,
                    limit=1
                )

                print(f"Search returned {len(search_results)} results")
                if search_results:
                    result = search_results[0]
                    print(f"First result - ID: {result.id}, Score: {result.score}")
                    print(f"Payload keys: {list(result.payload.keys())}")

    except Exception as e:
        print(f"Error during search test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_qdrant_api()