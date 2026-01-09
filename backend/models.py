"""
Data models for the backend API
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class RetrievedTextChunk(BaseModel):
    """
    Model representing a text chunk retrieved from the vector database
    """
    text_content: str = Field(..., description="The actual text content of the chunk")
    source_url: str = Field(default="", description="URL where the content was sourced from")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata about the chunk")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score between 0 and 1")
    chunk_id: str = Field(..., description="Unique identifier for this chunk")


class QueryVector(BaseModel):
    """
    Model representing a query vector with its original text
    """
    vector: List[float] = Field(..., description="The embedding vector")
    original_query: str = Field(..., description="The original text query")
    query_id: str = Field(..., description="Unique identifier for this query")


class SearchResult(BaseModel):
    """
    Model representing search results from the vector database
    """
    query_id: str = Field(..., description="ID of the original query")
    retrieved_chunks: List[RetrievedTextChunk] = Field(..., description="List of retrieved text chunks")
    search_time: float = Field(..., description="Time taken for the search in seconds")
    query_timestamp: datetime = Field(..., description="Timestamp when the query was made")
    total_candidates: int = Field(..., description="Total number of candidates considered")


class ValidationReport(BaseModel):
    """
    Model representing a validation report for the retrieval pipeline
    """
    validation_timestamp: datetime = Field(..., description="Timestamp when validation was performed")
    total_queries: int = Field(..., description="Total number of queries tested")
    successful_queries: int = Field(..., description="Number of successful queries")
    average_response_time: float = Field(..., description="Average response time in seconds")
    relevance_accuracy: float = Field(..., description="Accuracy of retrieval as a percentage")
    details: List[SearchResult] = Field(..., description="Detailed results for each query")
    errors: List[str] = Field(default_factory=list, description="List of errors encountered during validation")


class QueryRequest(BaseModel):
    """
    Model for incoming query requests
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The query text")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of top results to return")
    collection_name: Optional[str] = Field(default=None, description="Name of the collection to search")


class QueryResponse(BaseModel):
    """
    Model for query response
    """
    success: bool = Field(..., description="Whether the query was successful")
    results: List[RetrievedTextChunk] = Field(..., description="List of retrieved text chunks")
    search_time: float = Field(..., description="Time taken for the search in seconds")
    error: Optional[str] = Field(default=None, description="Error message if the query failed")