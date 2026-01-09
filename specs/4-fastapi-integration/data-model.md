# Data Model: FastAPI Integration for RAG Agent

## API Request Models

### QueryRequest
- **Entity**: QueryRequest
- **Fields**:
  - `query`: string (required) - User's question or query
  - `session_id`: string (optional) - Session identifier for conversation context
  - `user_id`: string (optional) - User identifier for personalization
- **Validation Rules**:
  - `query` must be between 1-1000 characters
  - `query` must not be empty or whitespace only
  - `session_id` if provided, must be a valid string (36 chars max)
- **Relationships**: None

### HealthCheckRequest
- **Entity**: HealthCheckRequest
- **Fields**: None (GET request)
- **Validation Rules**: None
- **Relationships**: None

## API Response Models

### QueryResponse
- **Entity**: QueryResponse
- **Fields**:
  - `response`: string (required) - Agent's response to the query
  - `session_id`: string (optional) - Session identifier returned
  - `success`: boolean (required) - Whether the request was successful
  - `error`: string (optional) - Error message if request failed
  - `timestamp`: string (required) - ISO 8601 timestamp of response
- **Validation Rules**:
  - If `success` is true, `response` must be provided and non-empty
  - If `success` is false, `error` must be provided
  - `timestamp` must be in ISO 8601 format
- **Relationships**: None

### HealthCheckResponse
- **Entity**: HealthCheckResponse
- **Fields**:
  - `status`: string (required) - Health status ("healthy", "degraded", "unhealthy")
  - `timestamp`: string (required) - ISO 8601 timestamp
  - `details`: object (optional) - Additional health details
- **Validation Rules**:
  - `status` must be one of ["healthy", "degraded", "unhealthy"]
  - `timestamp` must be in ISO 8601 format
- **Relationships**: None

## Internal Data Models

### AgentQuery
- **Entity**: AgentQuery
- **Fields**:
  - `query`: string (required) - The original query
  - `session_id`: string (optional) - Session context
  - `timestamp`: string (required) - When query was received
- **Validation Rules**:
  - `query` must be between 1-1000 characters
  - `timestamp` must be in ISO 8601 format
- **Relationships**: None

### AgentResponse
- **Entity**: AgentResponse
- **Fields**:
  - `content`: string (required) - Agent's response content
  - `sources`: array of objects (optional) - Retrieved content sources
  - `confidence`: number (optional) - Confidence score (0-1)
  - `processing_time`: number (required) - Time taken in milliseconds
- **Validation Rules**:
  - `content` must be provided and non-empty
  - `confidence` if provided, must be between 0 and 1
  - `processing_time` must be a positive number
- **Relationships**: None

## State Transitions

### Query Processing States
- **Initial**: Query received by API
- **Processing**: Agent is processing the query
- **Completed**: Agent returned response
- **Failed**: Error occurred during processing

### Health States
- **Healthy**: All services available and responding
- **Degraded**: Some services slow but functional
- **Unhealthy**: Critical services unavailable

## Validation Rules Summary

### Request Validation
- All string fields must be properly sanitized
- Query length limited to prevent abuse
- Session IDs validated for proper format
- Required fields must be present

### Response Validation
- Success/error mutually exclusive
- Timestamps in proper format
- Content non-empty when successful
- Error messages appropriate for user display

## Error Response Format

### Standard Error Format
```json
{
  "response": "",
  "session_id": null,
  "success": false,
  "error": "Descriptive error message",
  "timestamp": "2025-12-26T10:30:00Z"
}
```

### Validation Error Format
```json
{
  "response": "",
  "session_id": null,
  "success": false,
  "error": "Validation failed: [field] [error description]",
  "timestamp": "2025-12-26T10:30:00Z"
}
```