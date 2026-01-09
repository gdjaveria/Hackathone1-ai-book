# Quickstart: AI Agent with Retrieval Capabilities

## Prerequisites

- Python 3.9+
- OpenAI API key
- Qdrant vector database instance (local or remote)
- Book content indexed in the vector database

## Setup

1. **Install dependencies**:
   ```bash
   pip install openai qdrant-client python-dotenv
   ```

2. **Set up environment variables**:
   Create a `.env` file with:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_HOST=your_qdrant_host
   QDRANT_PORT=6333  # default port
   COLLECTION_NAME=book_content
   ```

3. **Ensure book content is indexed**:
   - Make sure your book content is already indexed in Qdrant
   - Verify the collection exists and contains embeddings

## Running the Agent

1. **Start the agent**:
   ```bash
   python agent.py
   ```

2. **Interact with the agent**:
   - The agent will start in interactive mode
   - Type your queries and press Enter
   - The agent will retrieve relevant book content and respond based on that information

## Example Usage

```python
# For programmatic usage:
from agent import main

# This will start the interactive agent
if __name__ == "__main__":
    main()
```

## Configuration

The agent can be configured through environment variables:
- `OPENAI_MODEL`: The model to use (default: gpt-4-turbo)
- `TOP_K`: Number of results to retrieve (default: 3)
- `COLLECTION_NAME`: Qdrant collection name (default: book_content)

## Troubleshooting

- If you get API key errors, verify your OpenAI API key is set correctly
- If retrieval fails, check that Qdrant is running and the collection exists
- If responses seem unrelated, verify that book content is properly indexed