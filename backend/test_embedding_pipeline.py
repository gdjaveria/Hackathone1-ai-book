"""
Test suite for the embedding pipeline functionality
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.crawler import DocusaurusCrawler, crawl_docusaurus_site
from backend.embedding import CohereEmbeddingGenerator, embed_documents
from backend.storage import QdrantStorage, store_text_chunks_with_embeddings
from backend.search import similarity_search, validate_search_results
from backend.main import run_ingestion_pipeline
from backend.config import Config


def test_config_validation():
    """Test configuration validation"""
    errors = Config.validate()
    # Should have errors since API keys are not set in test environment
    assert isinstance(errors, list)


@patch('backend.crawler.requests.Session.get')
def test_crawler_basic_functionality(mock_get):
    """Test basic crawling functionality"""
    # Mock response
    mock_response = Mock()
    mock_response.text = '<html><head><title>Test</title></head><body><p>Test content</p></body></html>'
    mock_response.raise_for_status.return_value = None
    mock_get.return_value = mock_response

    crawler = DocusaurusCrawler("https://example.com")
    result = crawler.fetch_page_content("https://example.com")

    assert result is not None
    assert 'url' in result
    assert 'title' in result
    assert 'content' in result


def test_embed_single_text():
    """Test embedding generation (mocked since it requires API key)"""
    with patch.object(CohereEmbeddingGenerator, '__init__', return_value=None):
        with patch.object(CohereEmbeddingGenerator, 'embed_single_text', return_value=[0.1, 0.2, 0.3]):
            # This test would need to be expanded with proper mocking
            pass


def test_chunk_text_utility():
    """Test text chunking utility function"""
    from backend.utils import chunk_text

    text = "This is a test sentence. " * 100  # Create a long text
    chunks = chunk_text(text, chunk_size=50, overlap=10)

    assert len(chunks) > 0
    assert all(isinstance(chunk, str) for chunk in chunks)
    assert all(len(chunk) <= 50 for chunk in chunks)


def test_clean_html_content():
    """Test HTML cleaning utility function"""
    from backend.utils import clean_html_content

    html = "<html><head><title>Test</title></head><body><nav>Navigation</nav><p>Content</p></body></html>"
    cleaned = clean_html_content(html)

    assert "Navigation" not in cleaned
    assert "Content" in cleaned


@patch('backend.storage.QdrantClient')
def test_qdrant_storage_initialization(mock_client):
    """Test Qdrant storage initialization"""
    mock_instance = Mock()
    mock_client.return_value = mock_instance

    storage = QdrantStorage(collection_name="test_collection")

    # Verify client was initialized
    assert mock_client.called


def test_pipeline_with_mocked_components():
    """Test the main pipeline with mocked external dependencies"""
    with patch('backend.crawler.crawl_docusaurus_site') as mock_crawl, \
         patch('backend.embedding.embed_documents') as mock_embed, \
         patch('backend.storage.store_text_chunks_with_embeddings') as mock_store:

        # Mock return values
        mock_crawl.return_value = [{'url': 'https://example.com', 'content': 'test content'}]
        mock_embed.return_value = [{'embedding': [0.1, 0.2, 0.3], 'text': 'test', 'url': 'https://example.com'}]
        mock_store.return_value = True

        # Run pipeline with a fake URL
        results = run_ingestion_pipeline("https://example.com", validate=False)

        # Verify success
        assert results['success'] is True
        assert 'steps' in results


def test_similarity_search_with_mock():
    """Test similarity search with mocked dependencies"""
    with patch('backend.embedding.create_cohere_client') as mock_embed_client, \
         patch('backend.storage.retrieve_similar_texts') as mock_search:

        # Mock the embedding client
        mock_embed_client.return_value.embed_single_text.return_value = [0.1, 0.2, 0.3]

        # Mock the search results
        mock_search.return_value = [
            {'text': 'test result', 'score': 0.9, 'url': 'https://example.com'}
        ]

        results = similarity_search("test query")

        assert len(results) > 0
        assert 'text' in results[0]


if __name__ == "__main__":
    pytest.main([__file__])