"""
Qdrant vector database storage and retrieval functions
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional, Union
import logging
from backend.config import Config

logger = logging.getLogger(__name__)

class QdrantStorage:
    def __init__(self, url: Optional[str] = None, api_key: Optional[str] = None,
                 host: Optional[str] = None, port: Optional[int] = None, collection_name: Optional[str] = None):
        """
        Initialize Qdrant storage client
        :param url: Qdrant URL (if using cloud)
        :param api_key: Qdrant API key
        :param host: Qdrant host (for local instance)
        :param port: Qdrant port
        :param collection_name: Name of the collection to use
        """
        self.url = url or Config.QDRANT_URL
        self.api_key = api_key or Config.QDRANT_API_KEY
        self.host = host or Config.QDRANT_HOST
        self.port = port or Config.QDRANT_PORT
        self.collection_name = collection_name or Config.QDRANT_COLLECTION_NAME

        # Initialize Qdrant client
        if self.url and self.url.startswith(('http://', 'https://')):
            # Use cloud instance
            self.client = QdrantClient(url=self.url, api_key=self.api_key)
        else:
            # Use local instance
            self.client = QdrantClient(host=self.host, port=self.port)

    def setup_collection(self, vector_size: int = 1024, distance: str = "Cosine") -> bool:
        """
        Setup the collection in Qdrant with proper configuration
        :param vector_size: Size of the embedding vectors
        :param distance: Distance metric to use (Cosine, Euclidean, Dot)
        :return: True if successful, False otherwise
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if collection_exists:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance[distance.upper()]
                ),
                # Enable indexing for payload fields we'll search by
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                    indexing_threshold=20000,
                )
            )

            logger.info(f"Collection '{self.collection_name}' created successfully")
            return True
        except Exception as e:
            logger.error(f"Error setting up collection: {e}")
            return False

    def store_embeddings(self, embeddings_data: List[Dict[str, Any]]) -> bool:
        """
        Store embeddings and associated metadata in Qdrant
        :param embeddings_data: List of dictionaries containing 'embedding', 'text', 'url', and other metadata
        :return: True if successful, False otherwise
        """
        if not embeddings_data:
            logger.warning("No embeddings data to store")
            return True

        try:
            # Prepare points for insertion
            points = []
            for i, data in enumerate(embeddings_data):
                # Ensure embedding is a list of floats
                embedding = data['embedding']
                if isinstance(embedding, (list, tuple)):
                    embedding = [float(x) for x in embedding]
                else:
                    embedding = [float(x) for x in embedding.tolist() if hasattr(embedding, 'tolist')]

                # Prepare payload with metadata
                payload = {
                    'text': data.get('text', ''),
                    'url': data.get('url', ''),
                    'chunk_id': data.get('chunk_id', ''),
                    'chunk_index': data.get('chunk_index', 0),
                    'source_title': data.get('title', ''),
                    'created_at': data.get('created_at', ''),
                    # Add any other metadata that might be in the data
                    **{k: v for k, v in data.items()
                       if k not in ['embedding', 'text', 'url', 'chunk_id', 'chunk_index', 'title', 'created_at']}
                }

                # Create point
                point = models.PointStruct(
                    id=i,  # Use sequential IDs
                    vector=embedding,
                    payload=payload
                )
                points.append(point)

            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully stored {len(embeddings_data)} embeddings in collection '{self.collection_name}'")
            return True
        except Exception as e:
            logger.error(f"Error storing embeddings: {e}")
            return False

    def retrieve_embeddings(self, ids: List[int]) -> List[Dict[str, Any]]:
        """
        Retrieve specific embeddings by their IDs
        :param ids: List of point IDs to retrieve
        :return: List of dictionaries containing embedding data
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=ids
            )

            results = []
            for record in records:
                result = {
                    'id': record.id,
                    'embedding': record.vector,
                    'payload': record.payload
                }
                results.append(result)

            return results
        except Exception as e:
            logger.error(f"Error retrieving embeddings: {e}")
            return []

    def search_similar(self, query_embedding: List[float], top_k: int = 10,
                      filter_conditions: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings to the query embedding
        :param query_embedding: The embedding vector to search for similar items
        :param top_k: Number of top results to return
        :param filter_conditions: Optional filter conditions for metadata
        :return: List of dictionaries with similarity scores and metadata
        """
        try:
            # Prepare filter if provided
            qdrant_filter = None
            if filter_conditions:
                conditions = []
                for key, value in filter_conditions.items():
                    if isinstance(value, str):
                        conditions.append(models.FieldCondition(
                            key=key,
                            match=models.MatchText(text=value)
                        ))
                    elif isinstance(value, (int, float)):
                        conditions.append(models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        ))

                if conditions:
                    qdrant_filter = models.Filter(must=conditions)

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                query_filter=qdrant_filter,
                with_payload=True,
                with_vectors=False
            )

            results = []
            for hit in search_results:
                result = {
                    'id': hit.id,
                    'score': hit.score,
                    'payload': hit.payload,
                    'text': hit.payload.get('text', ''),
                    'url': hit.payload.get('url', ''),
                    'chunk_id': hit.payload.get('chunk_id', '')
                }
                results.append(result)

            return results
        except Exception as e:
            logger.error(f"Error searching for similar embeddings: {e}")
            return []


def initialize_qdrant_client(url: Optional[str] = None, api_key: Optional[str] = None,
                           host: Optional[str] = None, port: Optional[int] = None,
                           collection_name: Optional[str] = None) -> QdrantStorage:
    """
    Initialize Qdrant client with configuration
    :param url: Qdrant URL (if using cloud)
    :param api_key: Qdrant API key
    :param host: Qdrant host (for local instance)
    :param port: Qdrant port
    :param collection_name: Name of the collection to use
    :return: QdrantStorage instance
    """
    return QdrantStorage(url, api_key, host, port, collection_name)


def store_text_chunks_with_embeddings(text_chunks: List[Dict[str, Any]],
                                    vector_size: int = 1024) -> bool:
    """
    Store text chunks with embeddings in Qdrant
    :param text_chunks: List of dictionaries containing text, embeddings, and metadata
    :param vector_size: Size of the embedding vectors
    :return: True if successful, False otherwise
    """
    storage = initialize_qdrant_client()

    # Setup collection if it doesn't exist
    if not storage.setup_collection(vector_size=vector_size):
        return False

    # Store embeddings
    return storage.store_embeddings(text_chunks)


def retrieve_similar_texts(query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve texts similar to the query embedding
    :param query_embedding: The embedding vector to search for
    :param top_k: Number of results to return
    :return: List of similar text chunks with metadata
    """
    storage = initialize_qdrant_client()
    return storage.search_similar(query_embedding, top_k=top_k)