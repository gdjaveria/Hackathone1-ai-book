# Quickstart: URL Ingestion & Embedding Pipeline

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-24

## Setup Instructions

### Prerequisites
- Python 3.11+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

### 1. Environment Setup

Create a `.env` file with the following variables:

```bash
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here
COHERE_MODEL=embed-english-v3.0  # or embed-multilingual-v3.0

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=docusaurus_embeddings

# Processing Configuration
CHUNK_SIZE=512  # Number of words per chunk
CHUNK_OVERLAP=50  # Overlapping words between chunks
RATE_LIMIT_DELAY=1  # Delay between requests in seconds
```

### 2. Project Initialization

```bash
# Create backend directory
mkdir backend
cd backend

# Initialize project with UV
uv init
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv tqdm
```

### 3. Run the Pipeline

```bash
# From the backend directory
python main.py
```

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Your Qdrant cloud cluster URL (required)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (default: docusaurus_embeddings)
- `COHERE_MODEL`: Cohere embedding model to use (default: embed-english-v3.0)
- `CHUNK_SIZE`: Number of words per text chunk (default: 512)
- `CHUNK_OVERLAP`: Number of overlapping words between chunks (default: 50)
- `RATE_LIMIT_DELAY`: Delay between web requests in seconds (default: 1)

## Expected Output

When running the pipeline:
1. Documents will be crawled from the specified Docusaurus site
2. Content will be cleaned and chunked
3. Embeddings will be generated and stored in Qdrant
4. Progress will be displayed with tqdm
5. Final statistics will be printed showing:
   - Number of documents processed
   - Number of chunks created
   - Number of embeddings stored
   - Total processing time

## Troubleshooting

### Common Issues
- **API Key Errors**: Verify your Cohere and Qdrant API keys are correct
- **Rate Limits**: Adjust RATE_LIMIT_DELAY if encountering rate limit errors
- **Connection Issues**: Check your QDRant URL and network connectivity
- **Memory Issues**: For large sites, process in batches or increase system memory