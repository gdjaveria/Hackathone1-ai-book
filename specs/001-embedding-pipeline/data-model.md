# Data Model: URL Ingestion & Embedding Pipeline

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-24

## Core Data Structures

### Document
Represents a single page or content unit from a Docusaurus site

```python
{
  "id": str,                    # Unique identifier for the document
  "url": str,                   # Original URL of the document
  "title": str,                 # Title of the document
  "content": str,               # Cleaned text content
  "source": str,                # Source website/origin
  "created_at": str,            # Timestamp of ingestion
  "metadata": dict              # Additional metadata (tags, categories, etc.)
}
```

### TextChunk
A segment of text extracted from a document for embedding

```python
{
  "id": str,                    # Unique identifier for the chunk
  "document_id": str,           # Reference to parent document
  "content": str,               # Text content of the chunk
  "chunk_index": int,           # Position of chunk in original document
  "word_count": int,            # Number of words in chunk
  "token_count": int            # Estimated token count
}
```

### EmbeddingRecord
A vector representation of a text chunk with associated metadata

```python
{
  "id": str,                    # Unique identifier for the record
  "chunk_id": str,              # Reference to the text chunk
  "vector": list[float],        # The embedding vector
  "vector_size": int,           # Dimension of the vector
  "model": str,                 # Model used for embedding
  "created_at": str,            # Timestamp of embedding creation
  "payload": dict               # Additional metadata stored with vector
}
```

## Qdrant Collection Schema

### Collection: `docusaurus_embeddings`

**Vector Configuration**:
- Size: 1024 (for Cohere multilingual v3) or 768 (for English v3)
- Distance: Cosine

**Payload Fields**:
- `url`: Original document URL (keyword, indexed)
- `title`: Document title (text, indexed)
- `content`: Text content (text, stored)
- `document_id`: Reference to original document (keyword, indexed)
- `chunk_index`: Position of chunk in document (integer, indexed)
- `source`: Source website identifier (keyword, indexed)
- `created_at`: Timestamp (integer, indexed)

## Data Flow

1. **Ingestion**: URLs → Documents (with content extraction)
2. **Processing**: Documents → TextChunks (with chunking logic)
3. **Embedding**: TextChunks → EmbeddingRecords (with vector generation)
4. **Storage**: EmbeddingRecords → Qdrant Collection (with indexing)