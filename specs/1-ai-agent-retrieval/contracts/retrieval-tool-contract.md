# API Contract: Retrieval Tool for AI Agent

## Overview
This contract defines the interface for the retrieval tool that integrates with the AI agent. The tool allows the agent to search for relevant book content from a vector database.

## Tool Definition

### retrieval_tool
- **Name**: `retrieval_tool`
- **Description**: "Searches the book content database for relevant information based on the provided query"
- **Parameters**:
  ```json
  {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "The search query to find relevant book content"
      },
      "top_k": {
        "type": "integer",
        "description": "Number of results to return (default: 3, max: 10)",
        "default": 3,
        "minimum": 1,
        "maximum": 10
      }
    },
    "required": ["query"]
  }
  ```

## Expected Behavior

### Success Response
When the tool is called successfully, it returns:
```json
{
  "retrieved_content": [
    {
      "content_id": "string",
      "text": "string",
      "metadata": {
        "source": "string",
        "page": "integer",
        "chapter": "string"
      },
      "relevance_score": "number"
    }
  ],
  "total_results": "integer"
}
```

### Error Responses
- When no relevant content is found:
```json
{
  "retrieved_content": [],
  "message": "No relevant content found for the given query",
  "total_results": 0
}
```

- When vector database is unavailable:
```json
{
  "error": "Vector database unavailable",
  "message": "Unable to retrieve content due to database connection issues"
}
```

## Integration Points

### With OpenAI Agent
- The tool is registered with the OpenAI agent using the `tools` parameter
- The agent can call this tool during conversation flow when it needs additional context
- Results are formatted to be compatible with the agent's context window

### With Qdrant Database
- The tool connects to Qdrant using the configured host and collection
- Performs semantic search using vector similarity
- Returns top-k most relevant results based on the query