# Data Model: RAG Retrieval Validation

## Entity: RetrievedTextChunk
**Description**: A segment of book content that matches the user query, including the text content, source URL, and metadata
**Fields**:
- `text_content` (str): The actual text content of the chunk
- `source_url` (str): URL indicating the source document
- `metadata` (dict): Additional metadata about the chunk (document_id, page_number, etc.)
- `similarity_score` (float): The similarity score from the vector search
- `chunk_id` (str): Unique identifier for this text chunk

**Validation rules**:
- `text_content` must not be empty
- `source_url` must be a valid URL format
- `similarity_score` must be between 0 and 1

## Entity: QueryVector
**Description**: The vector representation of the user's text query used for similarity search in Qdrant
**Fields**:
- `vector` (list[float]): The embedding vector representation of the query
- `original_query` (str): The original text query from the user
- `query_id` (str): Unique identifier for this query

**Validation rules**:
- `vector` must have the same dimension as stored embeddings
- `original_query` must not be empty

## Entity: SearchResult
**Description**: Container for the results of a single query search operation
**Fields**:
- `query_id` (str): Reference to the original query
- `retrieved_chunks` (list[RetrievedTextChunk]): List of top-k retrieved chunks
- `search_time` (float): Time taken to perform the search in seconds
- `query_timestamp` (datetime): When the query was executed
- `total_candidates` (int): Total number of candidates considered

**Validation rules**:
- `retrieved_chunks` must contain at least 1 and at most k items
- `search_time` must be positive

## Entity: ValidationReport
**Description**: Report of the validation results for a set of queries
**Fields**:
- `validation_timestamp` (datetime): When validation was performed
- `total_queries` (int): Number of queries tested
- `successful_queries` (int): Number of queries that completed successfully
- `average_response_time` (float): Average response time across all queries
- `relevance_accuracy` (float): Percentage of relevant results (0-100)
- `details` (list[SearchResult]): Individual results for each query
- `errors` (list[str]): Any errors encountered during validation

**Validation rules**:
- `relevance_accuracy` must be between 0 and 100
- `successful_queries` must not exceed `total_queries`