# Data Model: AI Agent with Retrieval Capabilities

## Entities

### AI Agent
- **Properties**:
  - agent_id: string (unique identifier for the agent instance)
  - model: string (the underlying LLM model being used)
  - tools: list (list of available tools, including retrieval tool)
  - system_prompt: string (system message that defines agent behavior)
  - created_at: datetime (timestamp of agent creation)

### Retrieval Tool
- **Properties**:
  - tool_name: string ("retrieval_tool")
  - description: string ("Searches the book content database for relevant information")
  - parameters: object (query: string, top_k: integer)
  - function: callable (the actual retrieval function implementation)

### Book Content
- **Properties**:
  - content_id: string (unique identifier for the content chunk)
  - text: string (the actual content text)
  - metadata: object (source, page number, chapter, etc.)
  - embedding: list (vector representation for similarity search)
  - created_at: datetime (when the content was indexed)

### Agent Response
- **Properties**:
  - response_id: string (unique identifier for the response)
  - query: string (the original user query)
  - retrieved_context: list (list of retrieved content chunks used)
  - response_text: string (the agent's generated response)
  - timestamp: datetime (when the response was generated)
  - grounded_in_retrieved: boolean (whether the response was based on retrieved content)

### Query Request
- **Properties**:
  - query_id: string (unique identifier for the query)
  - text: string (the user's query text)
  - user_id: string (identifier for the user making the request)
  - timestamp: datetime (when the query was made)

## State Transitions

### Agent Response Generation
1. **Query Received**: User submits a query to the agent
2. **Retrieval Decision**: Agent determines if retrieval is needed
3. **Content Retrieved**: Relevant book content is fetched from vector database
4. **Response Generated**: Agent generates response based on retrieved context
5. **Response Returned**: Final response is returned to the user

## Validation Rules

### Agent Response
- Must contain information only from retrieved context (no hallucination)
- Must include reference to source content when possible
- Must handle cases where no relevant content is found
- Must provide appropriate error messages when vector database is unavailable

### Retrieval Tool
- Query text must be provided
- top_k parameter must be between 1 and 10
- Must return content only from the book database
- Must handle empty results appropriately