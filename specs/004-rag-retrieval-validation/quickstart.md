# Quickstart: RAG Retrieval Validation

## Overview
The `retrieve.py` script validates the RAG retrieval pipeline by connecting to Qdrant, performing top-k similarity searches with test queries, and validating the results.

## Prerequisites
- Python 3.9+
- Qdrant server running and accessible
- Cohere API key for embedding generation
- Existing vector collections in Qdrant from Spec-1

## Setup
1. Install required dependencies:
```bash
pip install qdrant-client cohere pydantic python-dotenv
```

2. Set up environment variables:
```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_HOST="your-qdrant-host"
export QDRANT_PORT="6333"  # default port
export COLLECTION_NAME="your-collection-name"
```

## Usage
Run the validation script with test queries:
```bash
cd backend
python retrieve.py
```

Or run with specific query:
```bash
python retrieve.py --query "your test query here"
```

Or run batch validation:
```bash
python retrieve.py --validate
```

## Configuration
The script uses configuration from `config.py` which includes:
- Qdrant connection settings
- Default top-k value (default: 5)
- Cohere model selection
- Timeout settings

## Output
The script will output:
- Individual search results with text chunks, metadata, and source URLs
- Performance metrics (response times)
- Validation report with success rates and accuracy
- Any errors encountered during the process

## Example Output
```
Query: "What is the capital of France?"
Top 5 results:
1. Text: "Paris is the capital and most populous city of France..."
   Source: https://example.com/book/page123
   Score: 0.87
   Metadata: {"document_id": "doc123", "page": 45}

Validation Report:
- Total queries: 1
- Successful: 1
- Avg response time: 0.42s
- Relevance accuracy: 92%
```