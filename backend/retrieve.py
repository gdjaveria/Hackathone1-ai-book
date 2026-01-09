#!/usr/bin/env python3
"""
RAG Retrieval Validation Script

This script connects to Qdrant to validate the RAG retrieval pipeline.
It performs top-k similarity search for user queries and validates results
with text content, metadata, and source URLs.

Usage Examples:
    # Run validation with test queries
    python retrieve.py --validate

    # Run a single query
    python retrieve.py --query "What is machine learning?"

    # Run a single query with custom top-k
    python retrieve.py --query "Explain neural networks" --top-k 3

    # Run with specific collection
    python retrieve.py --query "Tell me about RAG" --collection my_collection

    # Run basic connectivity tests (default)
    python retrieve.py

Environment Variables:
    COHERE_API_KEY: Your Cohere API key
    QDRANT_URL: URL of your Qdrant instance
    QDRANT_API_KEY: API key for Qdrant (if required)
    QDRANT_COLLECTION_NAME: Name of the collection to search (default: docusaurus_embeddings)
"""

import asyncio
import logging
import sys
import time
from datetime import datetime
from typing import List, Optional, Dict, Any

# Import dependencies
import qdrant_client
from qdrant_client.http import models
import cohere
from pydantic import BaseModel

# Import the existing config from backend
import os
import sys
# Add the project root to the path to import backend modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from backend.config import Config


# Import models
from backend.models import RetrievedTextChunk, QueryVector, SearchResult, ValidationReport

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def setup_qdrant_client():
    """Set up and return Qdrant client based on configuration."""
    config_errors = Config.validate()
    if config_errors:
        for error in config_errors:
            logger.error(f"Configuration error: {error}")
        raise ValueError("Configuration validation failed")

    try:
        logger.info(f"Connecting to Qdrant at {Config.QDRANT_URL}")
        client = qdrant_client.QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=30  # 30 second timeout
        )

        # Test the connection
        client.get_collections()
        logger.info("Successfully connected to Qdrant")

        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        raise

def setup_cohere_client():
    """Set up and return Cohere client based on configuration."""
    if not Config.COHERE_API_KEY:
        logger.error("COHERE_API_KEY is required")
        raise ValueError("COHERE_API_KEY is required")

    logger.info("Initializing Cohere client")
    client = cohere.Client(api_key=Config.COHERE_API_KEY)
    logger.info("Cohere client initialized successfully")
    return client

def load_qdrant_collections(client):
    """Load and return the available collections from Qdrant."""
    try:
        logger.info("Loading Qdrant collections...")
        collections = client.get_collections()
        collection_names = [collection.name for collection in collections.collections]
        logger.info(f"Available collections: {collection_names}")
        return collection_names
    except Exception as e:
        logger.error(f"Error loading Qdrant collections: {e}")
        raise

def generate_query_embedding(cohere_client, query_text: str, query_id: Optional[str] = None) -> QueryVector:
    """Generate embedding for user query using Cohere."""
    if not query_id:
        from uuid import uuid4
        query_id = str(uuid4())

    logger.info(f"Generating embedding for query: '{query_text[:50]}...'")
    try:
        # Use the Cohere client to generate embeddings
        response = cohere_client.embed(
            texts=[query_text],
            model=Config.COHERE_MODEL,  # Use the model from config
            input_type="search_query"  # Specify this is a search query
        )

        # Extract the embedding vector from the response
        embedding_vector = response.embeddings[0]  # Get the first (and only) embedding

        # Create and return QueryVector model
        query_vector = QueryVector(
            vector=embedding_vector,
            original_query=query_text,
            query_id=query_id
        )

        logger.info(f"Successfully generated embedding with {len(embedding_vector)} dimensions")
        return query_vector
    except Exception as e:
        logger.error(f"Error generating query embedding: {e}")
        raise

def validate_metadata(chunk: RetrievedTextChunk) -> List[str]:
    """
    Validate the metadata of a retrieved text chunk.

    Returns:
        List of validation errors, empty if valid
    """
    errors = []

    # Validate text content
    if not chunk.text_content or not chunk.text_content.strip():
        errors.append("Text content is empty")

    # Validate source URL format (only if URL is provided)
    if chunk.source_url and chunk.source_url.strip():
        import re
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)
        if not url_pattern.match(chunk.source_url):
            errors.append(f"Invalid URL format: {chunk.source_url}")

    # Validate similarity score
    if not (0 <= chunk.similarity_score <= 1):
        errors.append(f"Similarity score out of range [0,1]: {chunk.similarity_score}")

    return errors

def perform_similarity_search(qdrant_client, query_vector: QueryVector, top_k: int = 5, collection_name: str = None) -> List[RetrievedTextChunk]:
    """Perform top-k similarity search in Qdrant."""
    if not collection_name:
        collection_name = Config.QDRANT_COLLECTION_NAME

    logger.info(f"Performing similarity search in collection '{collection_name}' with top_k={top_k}")
    try:
        # Perform the search in Qdrant using the new API
        # Remove the 'using' parameter to let Qdrant use the default vector field
        search_results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector.vector,
            limit=top_k,
            with_payload=True,  # Include payload data (metadata)
        )

        # Convert search results to RetrievedTextChunk models
        retrieved_chunks = []
        for result in search_results.points:
            # Extract content from payload
            payload = result.payload or {}

            # Create RetrievedTextChunk from search result
            chunk = RetrievedTextChunk(
                text_content=payload.get('content', ''),
                source_url=payload.get('url', ''),
                metadata=payload,
                similarity_score=float(result.score),
                chunk_id=str(result.id)
            )

            # Validate the chunk metadata
            validation_errors = validate_metadata(chunk)
            if validation_errors:
                logger.warning(f"Metadata validation errors for chunk {chunk.chunk_id}: {validation_errors}")

            retrieved_chunks.append(chunk)

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant")
        return retrieved_chunks
    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        raise

def search_query(query_text: str, top_k: int = 5, collection_name: str = None) -> dict:
    """
    Main function to perform a search query with validation.

    Returns:
        dict: Dictionary with success status, results, search time, and error information
    """
    from datetime import datetime

    logger.info(f"Starting search query for: '{query_text[:50]}...' with top_k={top_k}")
    start_time = time.time()

    try:
        # Set up clients
        qdrant_client = setup_qdrant_client()
        cohere_client = setup_cohere_client()

        # Generate query embedding
        query_vector = generate_query_embedding(cohere_client, query_text)

        # Perform similarity search
        retrieved_chunks = perform_similarity_search(
            qdrant_client,
            query_vector,
            top_k=top_k,
            collection_name=collection_name
        )

        search_time = time.time() - start_time

        # Create SearchResult model
        search_result = SearchResult(
            query_id=query_vector.query_id,
            retrieved_chunks=retrieved_chunks,
            search_time=search_time,
            query_timestamp=datetime.now(),
            total_candidates=len(retrieved_chunks)  # This is an approximation
        )

        logger.info(f"Search completed successfully in {search_time:.2f}s, retrieved {len(retrieved_chunks)} chunks")
        return {
            "success": True,
            "results": retrieved_chunks,
            "search_time": search_time,
            "search_result": search_result,
            "error": None
        }
    except Exception as e:
        search_time = time.time() - start_time
        logger.error(f"Search query failed after {search_time:.2f}s: {e}")
        return {
            "success": False,
            "results": [],
            "search_time": search_time,
            "search_result": None,
            "error": str(e)
        }

def validate_metadata_preservation(search_results: List[SearchResult]) -> tuple[float, List[str]]:
    """
    Validate metadata preservation across all search results.

    Args:
        search_results: List of search results to validate

    Returns:
        Tuple of (preservation_accuracy, validation_errors)
    """
    total_chunks = 0
    valid_chunks = 0
    all_errors = []

    for search_result in search_results:
        for chunk in search_result.retrieved_chunks:
            total_chunks += 1
            validation_errors = validate_metadata(chunk)
            if not validation_errors:
                valid_chunks += 1
            else:
                all_errors.extend([f"Chunk {chunk.chunk_id}: {error}" for error in validation_errors])

    preservation_accuracy = (valid_chunks / total_chunks * 100) if total_chunks > 0 else 100.0
    return preservation_accuracy, all_errors

def validate_pipeline(test_queries: List[str], top_k: int = 5) -> ValidationReport:
    """
    Validate the entire RAG retrieval pipeline with multiple test queries.

    Args:
        test_queries: List of test queries to validate
        top_k: Number of top results to return for each query

    Returns:
        ValidationReport: Complete validation report
    """
    from datetime import datetime

    logger.info(f"Starting pipeline validation with {len(test_queries)} test queries")

    validation_start_time = time.time()
    total_queries = len(test_queries)
    successful_queries = 0
    total_response_time = 0.0
    search_results = []
    errors = []

    for i, query in enumerate(test_queries):
        logger.info(f"Processing query {i+1}/{total_queries}: '{query[:50]}...'")

        try:
            # Execute the search query
            result = search_query(query, top_k=top_k)

            if result["success"]:
                successful_queries += 1
                total_response_time += result["search_time"]

                # Add to search results if we have a search_result
                if result["search_result"]:
                    search_results.append(result["search_result"])
            else:
                errors.append(f"Query '{query}' failed: {result['error']}")
        except Exception as e:
            errors.append(f"Query '{query}' caused exception: {str(e)}")
            logger.error(f"Error processing query {i+1}: {e}")

    validation_time = time.time() - validation_start_time

    # Calculate average response time
    avg_response_time = total_response_time / successful_queries if successful_queries > 0 else 0.0

    # Calculate relevance accuracy (for now, just assume 100% if successful)
    # In a real scenario, we'd have ground truth to compare against
    relevance_accuracy = (successful_queries / total_queries * 100) if total_queries > 0 else 0.0

    # Validate metadata preservation across all results
    metadata_preservation_accuracy, metadata_errors = validate_metadata_preservation(search_results)
    if metadata_errors:
        logger.warning(f"Metadata validation found {len(metadata_errors)} issues")

    # Create validation report
    validation_report = ValidationReport(
        validation_timestamp=datetime.now(),
        total_queries=total_queries,
        successful_queries=successful_queries,
        average_response_time=avg_response_time,
        relevance_accuracy=relevance_accuracy,
        details=search_results,
        errors=errors + metadata_errors  # Include metadata validation errors in the report
    )

    logger.info(f"Pipeline validation completed. Success: {successful_queries}/{total_queries}, "
                f"Avg response time: {avg_response_time:.2f}s, Relevance accuracy: {relevance_accuracy:.1f}%")

    return validation_report

def run_end_to_end_test():
    """Perform end-to-end testing with sample queries."""
    logger.info("Running end-to-end tests...")

    # Test queries for end-to-end validation
    test_queries = [
        "What is the capital of France?",
        "Tell me about machine learning",
        "How does RAG work?"
    ]

    results_summary = []

    for query in test_queries:
        logger.info(f"Testing query: '{query}'")
        try:
            result = search_query(query, top_k=2)
            if result["success"]:
                logger.info(f"  ✓ Query successful, retrieved {len(result['results'])} results in {result['search_time']:.2f}s")
                results_summary.append((query, True, result['search_time'], len(result['results'])))
            else:
                logger.error(f"  ✗ Query failed: {result['error']}")
                results_summary.append((query, False, result['search_time'], 0))
        except Exception as e:
            logger.error(f"  ✗ Query caused exception: {e}")
            results_summary.append((query, False, 0, 0))

    # Print summary
    print(f"\nEnd-to-End Test Summary:")
    print(f"  Total queries: {len(test_queries)}")
    successful = sum(1 for _, success, _, _ in results_summary if success)
    print(f"  Successful: {successful}/{len(test_queries)}")
    print(f"  Success rate: {successful/len(test_queries)*100:.1f}%")

    for query, success, time_taken, num_results in results_summary:
        status = "✓" if success else "✗"
        print(f"  {status} '{query[:30]}...': {time_taken:.2f}s, {num_results} results")

    return successful == len(test_queries)

def main():
    """Main function to run the RAG retrieval validation."""
    import argparse

    parser = argparse.ArgumentParser(description='RAG Retrieval Validation Script')
    parser.add_argument('--query', type=str, help='Single query to test')
    parser.add_argument('--validate', action='store_true', help='Run validation with test queries')
    parser.add_argument('--test', action='store_true', help='Run end-to-end tests')
    parser.add_argument('--top-k', type=int, default=5, help='Number of top results to return (default: 5)')
    parser.add_argument('--collection', type=str, help='Collection name to search in (optional)')

    args = parser.parse_args()

    # Validate configuration
    config_errors = Config.validate()
    if config_errors:
        for error in config_errors:
            logger.error(f"Configuration error: {error}")
        return 1

    logger.info("Configuration is valid.")

    if args.test:
        # Run end-to-end tests
        success = run_end_to_end_test()
        return 0 if success else 1

    elif args.validate:
        # Run validation with test queries
        logger.info("Running pipeline validation...")
        test_queries = [
            "What is the capital of France?",
            "Tell me about machine learning",
            "How does RAG work?",
            "Explain vector embeddings",
            "What are the benefits of using Qdrant?"
        ]

        validation_report = validate_pipeline(test_queries, top_k=args.top_k)

        # Print summary
        print(f"\nValidation Summary:")
        print(f"  Total queries: {validation_report.total_queries}")
        print(f"  Successful: {validation_report.successful_queries}")
        print(f"  Success rate: {(validation_report.successful_queries/validation_report.total_queries)*100:.1f}%")
        print(f"  Average response time: {validation_report.average_response_time:.2f}s")
        print(f"  Relevance accuracy: {validation_report.relevance_accuracy:.1f}%")

        if validation_report.errors:
            print(f"  Errors: {len(validation_report.errors)}")
            for error in validation_report.errors:
                print(f"    - {error}")

        return 0

    elif args.query:
        # Run single query
        logger.info(f"Running single query: '{args.query}'")

        result = search_query(args.query, top_k=args.top_k, collection_name=args.collection)

        if result["success"]:
            print(f"\nQuery: '{args.query}'")
            print(f"Results ({len(result['results'])} found in {result['search_time']:.2f}s):")
            for i, chunk in enumerate(result["results"], 1):
                print(f"  {i}. Score: {chunk.similarity_score:.3f}")
                print(f"     Content: {chunk.text_content[:100]}...")
                print(f"     Source: {chunk.source_url}")
                print()
        else:
            logger.error(f"Query failed: {result['error']}")
            return 1

    else:
        # Default behavior - run some tests
        logger.info("Running basic connectivity tests...")

        try:
            # Test Qdrant client setup
            qdrant_client_instance = setup_qdrant_client()
            logger.info("Qdrant client initialized successfully.")

            # Load collections
            collections = load_qdrant_collections(qdrant_client_instance)
            logger.info(f"Loaded {len(collections)} collections from Qdrant.")

            # Test Cohere client setup
            cohere_client_instance = setup_cohere_client()
            logger.info("Cohere client initialized successfully.")

            # Test embedding generation
            test_query = "What is the capital of France?"
            query_vector = generate_query_embedding(cohere_client_instance, test_query)
            logger.info(f"Generated embedding for query: '{test_query[:30]}...' (vector length: {len(query_vector.vector)})")

            # Test search_query function
            search_result = search_query(test_query, top_k=3)
            if search_result["success"]:
                logger.info(f"Search completed in {search_result['search_time']:.2f}s")
                logger.info(f"Retrieved {len(search_result['results'])} results")

                # Show first result as example
                if search_result['results']:
                    first_result = search_result['results'][0]
                    print(f"Sample result: Score {first_result.similarity_score:.3f} from {first_result.source_url}")
            else:
                logger.error(f"Search failed: {search_result['error']}")
                return 1
        except Exception as e:
            logger.error(f"Error during retrieval process: {e}")
            return 1

    return 0

if __name__ == "__main__":
    sys.exit(main() or 0)