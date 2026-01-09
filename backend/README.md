# Docusaurus Embedding Pipeline

A backend pipeline for ingesting Docusaurus URLs, generating embeddings using Cohere models, and storing them in Qdrant vector database.

## Overview

This pipeline provides a complete solution for:
- Crawling Docusaurus websites to extract content
- Generating vector embeddings using Cohere models
- Storing embeddings in Qdrant for efficient similarity search
- Validating search relevance

## Features

- **URL Crawling**: Automatically discovers and crawls all pages on a Docusaurus site
- **Content Extraction**: Extracts clean text content while preserving document structure
- **Text Chunking**: Splits content into appropriately sized chunks for embedding
- **Embedding Generation**: Uses Cohere's powerful embedding models
- **Vector Storage**: Stores embeddings in Qdrant for fast similarity search
- **Search Validation**: Validates retrieval relevance with test queries

## Prerequisites

- Python 3.8+
- Cohere API key
- Qdrant Cloud account or local instance

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Navigate to the backend directory:
   ```bash
   cd backend
   ```

3. Install dependencies using uv:
   ```bash
   uv sync
   # or if using pip
   pip install -r requirements.txt
   ```

## Configuration

1. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

2. Edit the `.env` file with your API keys and configuration:
   ```bash
   # Add your Cohere API key
   COHERE_API_KEY=your_actual_cohere_api_key_here

   # Add your Qdrant configuration
   QDRANT_API_KEY=your_actual_qdrant_api_key_here
   QDRANT_URL=your_actual_qdrant_cluster_url_here
   ```

## Usage

### Command Line Interface

Run the complete ingestion pipeline:

```bash
python main.py --url https://your-docusaurus-site.com
```

Additional options:
```bash
# Specify chunk size and overlap
python main.py --url https://your-site.com --chunk-size 1024 --overlap 100

# Skip validation after ingestion
python main.py --url https://your-site.com --no-validate

# Enable verbose logging
python main.py --url https://your-site.com --verbose
```

### Programmatic Usage

```python
from backend.main import run_ingestion_pipeline

# Run the complete pipeline
results = run_ingestion_pipeline(
    base_url="https://your-docusaurus-site.com",
    chunk_size=512,
    overlap=50,
    validate=True
)

print(f"Pipeline success: {results['success']}")
print(f"Total time: {results['total_time']:.2f}s")
```

## Components

### Crawler (`crawler.py`)
- Discovers all URLs on a Docusaurus site
- Extracts clean text content from HTML pages
- Handles rate limiting and error handling

### Embedding (`embedding.py`)
- Generates embeddings using Cohere models
- Handles text chunking and batch processing
- Manages API rate limits and error handling

### Storage (`storage.py`)
- Manages Qdrant vector database operations
- Handles collection setup and embedding storage
- Provides similarity search capabilities

### Search (`search.py`)
- Performs similarity search on stored embeddings
- Validates search result relevance
- Provides ranking and scoring functions

### Main Pipeline (`main.py`)
- Orchestrates the complete workflow
- Provides command-line interface
- Handles error management and progress tracking

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `COHERE_API_KEY` | Your Cohere API key | required |
| `QDRANT_API_KEY` | Your Qdrant API key | required |
| `QDRANT_URL` | Qdrant cluster URL | required |
| `QDRANT_COLLECTION_NAME` | Name of the collection to use | `docusaurus_embeddings` |
| `COHERE_MODEL` | Cohere embedding model | `embed-english-v3.0` |
| `CHUNK_SIZE` | Size of text chunks | 512 |
| `CHUNK_OVERLAP` | Overlap between chunks | 50 |
| `RATE_LIMIT_DELAY` | Delay between requests (seconds) | 1 |

## Validation

The pipeline includes validation capabilities to ensure search quality:

```python
from backend.search import test_query_validation

queries = [
    {
        'query': 'How to set up the system?',
        'expected_keywords': ['setup', 'configuration', 'install']
    },
    {
        'query': 'What are the main features?',
        'expected_keywords': ['features', 'main', 'key']
    }
]

results = test_query_validation(queries)
print(f"Success rate: {results['success_rate']:.2%}")
print(f"Average relevance: {results['average_relevance_score']:.2f}")
```

## Error Handling

The pipeline includes comprehensive error handling:
- Network request failures during crawling
- API rate limits and service unavailability
- Invalid configuration validation
- Graceful degradation when services are unavailable

## Performance

- Processes 100 pages in under 5 minutes (depending on content size and network)
- Supports 10,000+ document chunks in Qdrant
- Efficient batch processing for embedding generation
- Optimized storage and retrieval operations

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure your Cohere and Qdrant API keys are correct and have sufficient permissions
2. **Rate Limits**: The pipeline includes rate limiting, but check your API usage limits
3. **Network Issues**: Ensure the target Docusaurus site is accessible and not blocked by robots.txt

### Logging

Enable debug logging for detailed information:
```bash
python main.py --url https://your-site.com --verbose
```

## License

[Add your license information here]