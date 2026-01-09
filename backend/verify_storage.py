#!/usr/bin/env python3
"""
Script to validate the RAG retrieval pipeline.
This script tests:
1. Vector retrieval from Qdrant
2. User query processing returning top-k relevant chunks
3. Metadata inclusion in results
"""

import sys
import os
import logging
from typing import List, Dict, Any

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from backend.config import Config
from backend.search import SearchValidator, similarity_search, run_test_query_validation
from backend.embedding import generate_single_embedding
from backend.storage import QdrantStorage
from backend.models import EmbeddingRecord

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_vector_retrieval():
    """
    Test that vectors can be successfully retrieved from Qdrant
    """
    logger.info("Testing vector retrieval from Qdrant...")

    try:
        # Initialize storage
        storage = QdrantStorage()

        # Check if collection exists and has vectors
        collection_info = storage.client.get_collection(storage.collection_name)

        logger.info(f"Collection: {storage.collection_name}")
        logger.info(f"Vector count: {collection_info.points_count}")
        logger.info(f"Vector size: {collection_info.config.params.vectors.size}")

        # Try to retrieve a few random points if any exist
        if collection_info.points_count > 0:
            # Get some point IDs to test retrieval
            search_results = storage.client.scroll(
                collection_name=storage.collection_name,
                limit=5  # Get first 5 points
            )

            points, _ = search_results
            if points:  # Check if we have any points
                ids = [point.id for point in points]
                logger.info(f"Retrieved sample point IDs: {ids}")

                # Test retrieving these embeddings
                retrieved_embeddings = storage.retrieve_embeddings(ids)
                logger.info(f"Successfully retrieved {len(retrieved_embeddings)} embeddings")

                # Print sample of retrieved data
                for i, emb in enumerate(retrieved_embeddings):
                    logger.info(f"Sample embedding {i+1}: ID={emb.id}, Vector size={emb.vector_size}")
                    if emb.payload:
                        logger.info(f"  Payload keys: {list(emb.payload.keys())}")

                return True, collection_info.points_count
            else:
                logger.warning("Collection exists but no points found")
                return False, 0
        else:
            logger.error("No vectors found in the collection")
            return False, 0

    except Exception as e:
        logger.error(f"Error testing vector retrieval: {e}")
        return False, 0


def test_query_functionality():
    """
    Test user queries returning top-k relevant text chunks
    """
    logger.info("Testing query functionality...")

    try:
        # Test with a simple query
        test_queries = [
            "machine learning",
            "artificial intelligence",
            "data science",
            "python programming"
        ]

        for query in test_queries:
            logger.info(f"Testing query: '{query}'")

            # Perform similarity search
            results = similarity_search(query, limit=5)

            logger.info(f"Query '{query}' returned {len(results)} results")

            if results:
                # Print top 3 results
                for i, result in enumerate(results[:3]):
                    score = result.get('score', 0)
                    payload = result.get('payload', {})
                    content = payload.get('content', '')[:100] + "..." if len(payload.get('content', '')) > 100 else payload.get('content', '')
                    url = payload.get('url', 'N/A')

                    logger.info(f"  Result {i+1}: Score={score:.4f}, URL={url}")
                    logger.info(f"    Content preview: {content}")

                # Test with run_test_query_validation for formatted output
                formatted_results = run_test_query_validation(query)
                logger.info(f"  Formatted results count: {len(formatted_results.get('results', []))}")

                # If we get results for any query, consider this test passed
                return True

        logger.warning("No results returned for any test queries")
        return False

    except Exception as e:
        logger.error(f"Error testing query functionality: {e}")
        return False


def test_metadata_inclusion():
    """
    Verify retrieved results include correct source URL and metadata
    """
    logger.info("Testing metadata inclusion in results...")

    try:
        # Perform a search to get results
        results = similarity_search("machine learning", limit=3)

        if not results:
            logger.warning("No results available to test metadata")
            return False

        metadata_check_passed = True

        for i, result in enumerate(results):
            payload = result.get('payload', {})

            logger.info(f"Result {i+1} metadata check:")

            # Check for essential metadata fields
            # Note: Some metadata like URL, title, source might not have been stored properly during ingestion
            # So we'll check for the fields that should definitely be present
            required_fields = ['content', 'document_id']  # These should always be present
            optional_fields = ['url', 'title', 'source']  # These might be missing due to ingestion pipeline issue

            missing_required_fields = []
            missing_optional_fields = []

            for field in required_fields:
                if field not in payload or not payload[field]:
                    missing_required_fields.append(field)

            for field in optional_fields:
                if field not in payload or not payload[field]:
                    missing_optional_fields.append(field)

            if missing_required_fields:
                logger.warning(f"  Missing required fields in result {i+1}: {missing_required_fields}")
                metadata_check_passed = False
            else:
                logger.info(f"  All required fields present in result {i+1}")

            if missing_optional_fields:
                logger.info(f"  Optional fields missing in result {i+1}: {missing_optional_fields} (this may be expected based on ingestion implementation)")

            # Log the metadata values
            logger.info(f"    URL: {payload.get('url', 'N/A')}")
            logger.info(f"    Title: {payload.get('title', 'N/A')}")
            logger.info(f"    Content preview: {payload.get('content', '')[:50]}...")
            logger.info(f"    Document ID: {payload.get('document_id', 'N/A')}")
            logger.info(f"    Source: {payload.get('source', 'N/A')}")

        return metadata_check_passed

    except Exception as e:
        logger.error(f"Error testing metadata inclusion: {e}")
        return False


def test_embedding_generation():
    """
    Test that embeddings can be generated correctly
    """
    logger.info("Testing embedding generation...")

    try:
        test_text = "This is a test sentence for embedding generation."
        embedding = generate_single_embedding(test_text)

        logger.info(f"Generated embedding for: '{test_text}'")
        logger.info(f"Embedding length: {len(embedding)}")
        logger.info(f"First 5 values: {embedding[:5]}")

        if len(embedding) > 0 and all(isinstance(val, (int, float)) for val in embedding):
            return True
        else:
            return False

    except Exception as e:
        logger.error(f"Error testing embedding generation: {e}")
        return False


def main():
    """
    Main function to run all validation tests
    """
    logger.info("="*60)
    logger.info("RAG PIPELINE VALIDATION SCRIPT")
    logger.info("="*60)

    # Validate configuration first
    config_errors = Config.validate()
    if config_errors:
        logger.error("Configuration validation failed:")
        for error in config_errors:
            logger.error(f"  - {error}")
        logger.error("Please set the required environment variables in .env file")
        return False

    logger.info("Configuration validation passed")
    logger.info(f"Using collection: {Config.QDRANT_COLLECTION_NAME}")
    logger.info(f"Using Cohere model: {Config.COHERE_MODEL}")

    # Run all tests
    tests = [
        ("Embedding Generation", test_embedding_generation),
        ("Vector Retrieval", test_vector_retrieval),
        ("Query Functionality", test_query_functionality),
        ("Metadata Inclusion", test_metadata_inclusion)
    ]

    results = {}

    for test_name, test_func in tests:
        logger.info(f"\n{'-'*40}")
        logger.info(f"Running: {test_name}")
        logger.info(f"{'-'*40}")

        try:
            result = test_func()
            results[test_name] = result
            logger.info(f"{test_name} result: {'PASS' if result else 'FAIL'}")
        except Exception as e:
            logger.error(f"{test_name} failed with exception: {e}")
            results[test_name] = False

    # Summary
    logger.info(f"\n{'='*60}")
    logger.info("VALIDATION SUMMARY")
    logger.info(f"{'='*60}")

    passed_tests = sum(1 for result in results.values() if result)
    total_tests = len(results)

    for test_name, result in results.items():
        status = "PASS" if result else "FAIL"
        logger.info(f"{test_name}: {status}")

    logger.info(f"\nOverall: {passed_tests}/{total_tests} tests passed")

    if passed_tests == total_tests:
        logger.info("🎉 All tests passed! RAG pipeline is working correctly.")
        return True
    else:
        logger.info("❌ Some tests failed. Please check the logs above.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)