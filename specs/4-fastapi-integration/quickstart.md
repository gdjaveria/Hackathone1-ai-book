# Quickstart Guide: FastAPI Integration for RAG Agent

## Overview
This guide provides instructions for setting up and using the FastAPI backend that integrates with the RAG agent for frontend communication.

## Prerequisites
- Python 3.8 or higher
- pip package manager
- Valid OpenAI API key
- Access to Qdrant database
- Node.js and npm (for Docusaurus frontend)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathone
```

### 2. Install Backend Dependencies
```bash
pip install -r backend/requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key
OPENAI_MODEL=gpt-4o
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_HOST=localhost
QDRANT_PORT=6333
COLLECTION_NAME=book_content
```

### 4. Start the FastAPI Server
```bash
python run_api.py
# Server will start on http://localhost:8000
```

## API Usage

### Query Endpoint
Send a query to the RAG agent:

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is artificial intelligence?",
    "session_id": "optional-session-id"
  }'
```

Response format:
```json
{
  "response": "Artificial intelligence is a branch of computer science...",
  "session_id": "optional-session-id",
  "success": true,
  "error": null
}
```

### Health Check
Verify the API and agent are available:

```bash
curl http://localhost:8000/health
```

## Frontend Integration

### Docusaurus Chatbot Integration
1. Navigate to the Docusaurus project in `physical-ai-book`
2. Create a chatbot component that makes POST requests to `http://localhost:8000/query`
3. Use the API contract defined in `specs/4-fastapi-integration/contracts/api-contract.yaml`

### Example Frontend Code
```javascript
async function queryRAGAgent(query, sessionId = null) {
  try {
    const response = await fetch('http://localhost:8000/query', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: query,
        session_id: sessionId
      })
    });

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error querying RAG agent:', error);
    return {
      response: '',
      success: false,
      error: 'Network error occurred'
    };
  }
}
```

## Testing

### API Testing
Test the API endpoints:

```bash
# Test root endpoint
curl http://localhost:8000/

# Test health endpoint
curl http://localhost:8000/health

# Test query endpoint
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{"query": "Hello"}'
```

### Run API Tests
```bash
python test_api.py
```

## Error Handling

### Common Errors
- **429 - Rate Limiting**: Check OpenAI quota and billing
- **500 - Internal Server Error**: Check server logs
- **400 - Bad Request**: Verify request format

### Error Response Format
```json
{
  "response": "",
  "session_id": null,
  "success": false,
  "error": "Descriptive error message"
}
```

## Production Deployment

### Environment Configuration
For production, update the environment variables:

```env
OPENAI_API_KEY=your_production_api_key
OPENAI_MODEL=gpt-4o
QDRANT_URL=your_production_qdrant_url
QDRANT_API_KEY=your_production_qdrant_api_key
COLLECTION_NAME=book_content
```

### Running in Production
```bash
# Using uvicorn with multiple workers
uvicorn backend.rag_api:app --host 0.0.0.0 --port 8000 --workers 4
```

## Troubleshooting

### Server Won't Start
- Verify all dependencies are installed
- Check that required environment variables are set
- Ensure no other process is using port 8000

### API Returns Errors
- Check OpenAI API key validity and quota
- Verify Qdrant database connectivity
- Review server logs for detailed error information

### Frontend Integration Issues
- Verify CORS settings if accessing from different origin
- Check that API endpoints are accessible
- Validate request/response formats match the contract