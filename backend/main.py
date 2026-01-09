"""
Main ingestion pipeline that orchestrates the entire process from URL crawling to embedding storage
"""
import argparse
import logging
import sys
import time
from typing import List, Dict, Any
from tqdm import tqdm
import os
import sys

# Load environment variables from .env file in the root directory
from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.crawler import DocusaurusCrawler, crawl_docusaurus_site
from backend.embedding import embed_documents
from backend.storage import store_text_chunks_with_embeddings
from backend.search import run_sample_validation
from backend.config import Config

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def run_ingestion_pipeline(base_url: str, chunk_size: int = 512, overlap: int = 50,
                          validate: bool = True) -> Dict[str, Any]:
    """
    Run the complete ingestion pipeline from URL crawling to embedding storage
    :param base_url: Base URL of the Docusaurus site to process
    :param chunk_size: Size of text chunks for embedding
    :param overlap: Overlap between text chunks
    :param validate: Whether to run validation after ingestion
    :return: Dictionary with pipeline results and statistics
    """
    start_time = time.time()

    # Validate configuration
    config_errors = Config.validate()
    if config_errors:
        logger.error(f"Configuration errors: {config_errors}")
        return {
            'success': False,
            'error': 'Configuration validation failed',
            'config_errors': config_errors
        }

    results = {
        'success': True,
        'steps': {},
        'total_time': 0
    }

    try:
        # Step 1: Crawl the Docusaurus site
        logger.info(f"Starting crawl of {base_url}")
        crawl_start = time.time()

        documents = crawl_docusaurus_site(base_url)

        crawl_time = time.time() - crawl_start
        results['steps']['crawling'] = {
            'time': crawl_time,
            'documents_found': len(documents),
            'status': 'completed' if documents else 'no_content'
        }

        logger.info(f"Crawling completed in {crawl_time:.2f}s. Found {len(documents)} documents")

        if not documents:
            logger.warning("No documents found during crawling")
            results['success'] = False
            results['error'] = 'No documents found during crawling'
            return results

        # Step 2: Generate embeddings
        logger.info(f"Starting embedding generation for {len(documents)} documents")
        embedding_start = time.time()

        # Process documents in batches to manage memory usage
        batch_size = 5  # Reduce batch size to avoid potential timeout issues
        all_embeddings = []

        for i in tqdm(range(0, len(documents), batch_size), desc="Processing document batches"):
            batch = documents[i:i + batch_size]
            try:
                batch_embeddings = embed_documents(batch, chunk_size=chunk_size, overlap=overlap)
                all_embeddings.extend(batch_embeddings)
                logger.info(f"Completed batch {i//batch_size + 1}/{(len(documents)-1)//batch_size + 1}, total embeddings: {len(all_embeddings)}")
            except Exception as e:
                logger.error(f"Error processing batch {i//batch_size + 1}: {e}")
                # Continue with next batch instead of failing completely
                continue

        embedding_time = time.time() - embedding_start
        results['steps']['embedding'] = {
            'time': embedding_time,
            'chunks_processed': len(all_embeddings),
            'status': 'completed'
        }

        logger.info(f"Embedding completed in {embedding_time:.2f}s. Processed {len(all_embeddings)} text chunks")

        # Step 3: Store embeddings in Qdrant
        logger.info(f"Starting storage of {len(all_embeddings)} embeddings")
        storage_start = time.time()

        storage_success = store_text_chunks_with_embeddings(all_embeddings)

        storage_time = time.time() - storage_start
        results['steps']['storage'] = {
            'time': storage_time,
            'chunks_stored': len(all_embeddings) if storage_success else 0,
            'status': 'completed' if storage_success else 'failed'
        }

        logger.info(f"Storage completed in {storage_time:.2f}s. Success: {storage_success}")

        if not storage_success:
            results['success'] = False
            results['error'] = 'Failed to store embeddings in Qdrant'
            return results

        # Step 4: Run validation if requested
        if validate:
            logger.info("Running validation tests")
            validation_start = time.time()

            validation_results = run_sample_validation()

            validation_time = time.time() - validation_start
            results['steps']['validation'] = {
                'time': validation_time,
                'status': 'completed',
                'results': validation_results
            }

            logger.info(f"Validation completed in {validation_time:.2f}s")

        # Calculate total time
        total_time = time.time() - start_time
        results['total_time'] = total_time

        logger.info(f"Pipeline completed successfully in {total_time:.2f}s")
        return results

    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        results['success'] = False
        results['error'] = str(e)
        results['total_time'] = time.time() - start_time
        return results


def main():
    """
    Main function with command-line interface
    """
    parser = argparse.ArgumentParser(description='Docusaurus URL Ingestion & Embedding Pipeline')
    parser.add_argument('--url', type=str, required=True, help='Base URL of the Docusaurus site to process')
    parser.add_argument('--chunk-size', type=int, default=512, help='Size of text chunks for embedding (default: 512)')
    parser.add_argument('--overlap', type=int, default=50, help='Overlap between text chunks (default: 50)')
    parser.add_argument('--no-validate', action='store_true', help='Skip validation after ingestion')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    # Set logging level based on verbose flag
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logger.info(f"Starting ingestion pipeline for URL: {args.url}")

    # Run the pipeline
    results = run_ingestion_pipeline(
        base_url=args.url,
        chunk_size=args.chunk_size,
        overlap=args.overlap,
        validate=not args.no_validate
    )

    # Print results
    if results['success']:
        print(f"\n[SUCCESS] Pipeline completed successfully!")
        print(f"[STATS] Results:")
        print(f"   Total time: {results['total_time']:.2f}s")
        if 'crawling' in results['steps']:
            crawl_info = results['steps']['crawling']
            print(f"   Documents crawled: {crawl_info['documents_found']} ({crawl_info['time']:.2f}s)")
        if 'embedding' in results['steps']:
            embed_info = results['steps']['embedding']
            print(f"   Text chunks processed: {embed_info['chunks_processed']} ({embed_info['time']:.2f}s)")
        if 'storage' in results['steps']:
            storage_info = results['steps']['storage']
            print(f"   Chunks stored: {storage_info['chunks_stored']} ({storage_info['time']:.2f}s)")
    else:
        print(f"\n[ERROR] Pipeline failed!")
        print(f"Error: {results.get('error', 'Unknown error')}")
        sys.exit(1)


if __name__ == "__main__":
    main()