"""
Text embedding generation using Cohere models
"""
import cohere
from typing import List, Dict, Any, Optional
import logging
from backend.config import Config
from backend.utils import chunk_text

logger = logging.getLogger(__name__)

class CohereEmbeddingGenerator:
    def __init__(self, api_key: Optional[str] = None, model: str = "embed-english-v3.0"):
        """
        Initialize the Cohere embedding generator
        :param api_key: Cohere API key (if not provided, will use from config)
        :param model: Cohere embedding model to use
        """
        self.model = model
        self.api_key = api_key or Config.COHERE_API_KEY

        if not self.api_key:
            raise ValueError("Cohere API key is required")

        self.client = cohere.Client(self.api_key)

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        :param texts: List of texts to embed
        :param input_type: Type of input (search_document, search_query, etc.)
        :return: List of embedding vectors
        """
        if not texts:
            return []

        try:
            # Cohere API has limits, so we may need to batch requests
            batch_size = 96  # Cohere's recommended batch size
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type=input_type
                )

                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

            return all_embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    def embed_text_chunks(self, text_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for text chunks with metadata
        :param text_chunks: List of dictionaries containing 'text', 'url', and other metadata
        :return: List of dictionaries with embeddings added
        """
        if not text_chunks:
            return []

        # Extract just the text content for embedding
        texts = [chunk['text'] for chunk in text_chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings(texts)

        # Combine with original metadata
        result = []
        for i, chunk in enumerate(texts):
            chunk_with_embedding = text_chunks[i].copy()
            chunk_with_embedding['embedding'] = embeddings[i]
            result.append(chunk_with_embedding)

        return result

    def embed_single_text(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text
        :param text: Text to embed
        :param input_type: Type of input
        :return: Embedding vector
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type=input_type
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding for single text: {e}")
            raise


def create_cohere_client(api_key: Optional[str] = None) -> CohereEmbeddingGenerator:
    """
    Create a Cohere embedding generator instance
    :param api_key: Cohere API key (if not provided, will use from config)
    :return: CohereEmbeddingGenerator instance
    """
    api_key = api_key or Config.COHERE_API_KEY
    return CohereEmbeddingGenerator(api_key)


def chunk_and_embed_text(text: str, url: str = "", chunk_size: int = 512, overlap: int = 50,
                        api_key: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Chunk text and generate embeddings for each chunk
    :param text: Text to chunk and embed
    :param url: Source URL for metadata
    :param chunk_size: Size of text chunks
    :param overlap: Overlap between chunks
    :param api_key: Cohere API key
    :return: List of dictionaries with text chunks and their embeddings
    """
    # Chunk the text
    text_chunks = chunk_text(text, chunk_size, overlap)

    # Create chunk data with metadata
    chunk_data = []
    for i, chunk in enumerate(text_chunks):
        chunk_data.append({
            'text': chunk,
            'url': url,
            'chunk_id': f"{url}#{i}",
            'chunk_index': i
        })

    # Generate embeddings
    embedding_generator = create_cohere_client(api_key)
    return embedding_generator.embed_text_chunks(chunk_data)


def embed_documents(documents: List[Dict[str, str]], chunk_size: int = 512, overlap: int = 50,
                   api_key: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Embed a list of documents (each with 'url' and 'content' keys)
    :param documents: List of documents with 'url' and 'content' keys
    :param chunk_size: Size of text chunks
    :param overlap: Overlap between chunks
    :param api_key: Cohere API key
    :return: List of dictionaries with text chunks, metadata, and embeddings
    """
    all_embeddings = []

    for doc in documents:
        content = doc.get('content', '')
        url = doc.get('url', '')

        if content:
            doc_embeddings = chunk_and_embed_text(content, url, chunk_size, overlap, api_key)
            all_embeddings.extend(doc_embeddings)

    return all_embeddings