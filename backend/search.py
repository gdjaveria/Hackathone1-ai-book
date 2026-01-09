"""
Similarity search and validation functions for the embedding pipeline
"""
from typing import List, Dict, Any, Optional
import logging
from backend.storage import QdrantStorage, retrieve_similar_texts
from backend.embedding import create_cohere_client
from backend.config import Config

logger = logging.getLogger(__name__)

def similarity_search(query: str, top_k: int = 5, api_key: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Perform similarity search for a text query
    :param query: The text query to search for
    :param top_k: Number of results to return
    :param api_key: Cohere API key (optional, will use config if not provided)
    :return: List of similar text chunks with metadata and similarity scores
    """
    try:
        # Generate embedding for the query
        embedding_generator = create_cohere_client(api_key)
        query_embedding = embedding_generator.embed_single_text(query, input_type="search_query")

        # Search for similar embeddings in Qdrant
        results = retrieve_similar_texts(query_embedding, top_k=top_k)

        return results
    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        return []


def validate_search_results(query: str, expected_keywords: List[str], top_k: int = 5) -> Dict[str, Any]:
    """
    Validate search results by checking if expected keywords appear in results
    :param query: The query that was used
    :param expected_keywords: List of keywords that should appear in relevant results
    :param top_k: Number of results to check
    :return: Validation report with metrics
    """
    try:
        results = similarity_search(query, top_k=top_k)

        # Count how many results contain expected keywords
        relevant_results = 0
        total_keywords_found = 0
        keyword_appearance = {}

        for result in results:
            text = result.get('text', '').lower()
            result_keywords = []

            for keyword in expected_keywords:
                if keyword.lower() in text:
                    result_keywords.append(keyword)
                    total_keywords_found += 1
                    keyword_appearance[keyword] = keyword_appearance.get(keyword, 0) + 1

            if result_keywords:
                relevant_results += 1

        # Calculate metrics
        relevance_score = relevant_results / len(results) if results else 0
        keyword_coverage = total_keywords_found / sum(1 for _ in expected_keywords) if expected_keywords else 0

        validation_report = {
            'query': query,
            'total_results': len(results),
            'relevant_results': relevant_results,
            'relevance_score': relevance_score,
            'expected_keywords': expected_keywords,
            'keywords_found': list(keyword_appearance.keys()),
            'keyword_appearance_count': keyword_appearance,
            'keyword_coverage': keyword_coverage,
            'results': results
        }

        return validation_report
    except Exception as e:
        logger.error(f"Error validating search results: {e}")
        return {
            'query': query,
            'error': str(e),
            'total_results': 0,
            'relevant_results': 0,
            'relevance_score': 0,
            'expected_keywords': expected_keywords,
            'keywords_found': [],
            'keyword_appearance_count': {},
            'keyword_coverage': 0,
            'results': []
        }


def rank_search_results(results: List[Dict[str, Any]], query: str) -> List[Dict[str, Any]]:
    """
    Rank search results based on relevance to the query
    :param results: List of search results from similarity_search
    :param query: Original query for context
    :return: Ranked list of results
    """
    # For now, results from Qdrant are already ranked by similarity score
    # In the future, we could implement additional ranking based on other factors
    return sorted(results, key=lambda x: x['score'], reverse=True)


def test_query_validation(queries: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Run validation on multiple test queries
    :param queries: List of dictionaries with 'query' and 'expected_keywords' keys
    :return: Comprehensive validation report
    """
    all_results = []
    total_queries = len(queries)
    successful_queries = 0
    total_relevance_score = 0
    errors = []

    for query_data in queries:
        query = query_data.get('query', '')
        expected_keywords = query_data.get('expected_keywords', [])

        try:
            validation_result = validate_search_results(query, expected_keywords)
            all_results.append(validation_result)

            if 'error' not in validation_result:
                successful_queries += 1
                total_relevance_score += validation_result.get('relevance_score', 0)
            else:
                errors.append(f"Query '{query}': {validation_result.get('error', 'Unknown error')}")
        except Exception as e:
            error_msg = f"Query '{query}': {str(e)}"
            errors.append(error_msg)
            logger.error(f"Error validating query: {error_msg}")

    # Calculate overall metrics
    average_relevance = total_relevance_score / successful_queries if successful_queries > 0 else 0
    success_rate = successful_queries / total_queries if total_queries > 0 else 0

    validation_report = {
        'total_queries': total_queries,
        'successful_queries': successful_queries,
        'success_rate': success_rate,
        'average_relevance_score': average_relevance,
        'detailed_results': all_results,
        'errors': errors
    }

    return validation_report


def calculate_relevance_score(result: Dict[str, Any], query: str) -> float:
    """
    Calculate a relevance score for a search result based on multiple factors
    :param result: A single search result
    :param query: The original query
    :return: Relevance score between 0 and 1
    """
    score = result.get('score', 0)  # Base score from vector similarity

    # Additional factors could be added here in the future:
    # - Keyword matching in text
    # - Text length appropriateness
    # - Source document quality
    # - etc.

    return min(score, 1.0)  # Ensure score doesn't exceed 1.0


def search_with_validation(query: str, top_k: int = 5, validate: bool = True,
                         expected_keywords: Optional[List[str]] = None) -> Dict[str, Any]:
    """
    Perform search with optional validation
    :param query: The search query
    :param top_k: Number of results to return
    :param validate: Whether to perform validation
    :param expected_keywords: Keywords to validate against (if validation enabled)
    :return: Search results with optional validation
    """
    try:
        # Perform the search
        results = similarity_search(query, top_k=top_k)

        response = {
            'query': query,
            'results': results,
            'result_count': len(results)
        }

        # Add validation if requested
        if validate:
            validation_query = {'query': query}
            if expected_keywords:
                validation_query['expected_keywords'] = expected_keywords
            else:
                # Extract keywords from query if not provided
                validation_query['expected_keywords'] = query.lower().split()[:5]  # Use first 5 words as keywords

            validation_result = validate_search_results(**validation_query)
            response['validation'] = validation_result

        return response
    except Exception as e:
        logger.error(f"Error in search with validation: {e}")
        return {
            'query': query,
            'results': [],
            'result_count': 0,
            'error': str(e)
        }


# Example usage function
def run_sample_validation() -> Dict[str, Any]:
    """
    Run a sample validation to test the search functionality
    """
    sample_queries = [
        {
            'query': 'How to set up the system?',
            'expected_keywords': ['setup', 'configuration', 'install', 'system']
        },
        {
            'query': 'What are the main features?',
            'expected_keywords': ['features', 'main', 'key', 'functionality']
        }
    ]

    return test_query_validation(sample_queries)