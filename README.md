# RAG Retrieval Validation

This project provides a validation script for RAG (Retrieval-Augmented Generation) retrieval pipelines, specifically designed to validate data stored in Qdrant vector databases.

## Features

- Connect to Qdrant vector database to retrieve embedded content
- Perform top-k similarity search for user queries
- Validate results include text content, metadata, and source URLs
- Comprehensive logging and performance metrics
- End-to-end pipeline validation

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   export QDRANT_URL="your-qdrant-instance-url"
   export QDRANT_API_KEY="your-qdrant-api-key"  # if required
   export QDRANT_COLLECTION_NAME="your-collection-name"  # default: docusaurus_embeddings
   ```

## Usage

### Run validation with test queries:
```bash
python retrieve.py --validate
```

### Run a single query:
```bash
python retrieve.py --query "What is machine learning?"
```

### Run a single query with custom top-k:
```bash
python retrieve.py --query "Explain neural networks" --top-k 3
```

### Run with specific collection:
```bash
python retrieve.py --query "Tell me about RAG" --collection my_collection
```

### Run end-to-end tests:
```bash
python retrieve.py --test
```

### Run basic connectivity tests (default):
```bash
python retrieve.py
```

## Configuration

The script uses configuration from `backend/config.py` which includes:
- Qdrant connection settings
- Cohere model selection
- Default top-k value (default: 5)
- Timeout settings

## Output

The script provides detailed output including:
- Individual search results with text chunks, metadata, and source URLs
- Performance metrics (response times)
- Validation reports with success rates and accuracy
- Any errors encountered during the process 
