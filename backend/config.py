"""
Configuration module for the backend
"""
import os
from typing import List, Optional
from pydantic import BaseModel
import logging

# Load environment variables from .env file
from dotenv import load_dotenv
import sys
import os
# Load from both the root directory and backend directory
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env'))  # Root .env
load_dotenv()  # Backend .env (current directory)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Config:
    """Configuration class for backend services"""

    # Qdrant configuration
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "docusaurus_embeddings")
    QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT = int(os.getenv("QDRANT_PORT", "6333"))

    # Cohere configuration
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-english-v3.0")

    # API configuration
    API_HOST = os.getenv("API_HOST", "0.0.0.0")
    API_PORT = int(os.getenv("API_PORT", "8000"))
    DEBUG = os.getenv("DEBUG", "false").lower() == "true"

    # Other configurations
    LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

    @classmethod
    def validate(cls) -> List[str]:
        """Validate configuration and return a list of errors"""
        errors = []

        # Validate required environment variables
        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")

        # Validate Qdrant URL format if provided
        if cls.QDRANT_URL and not cls.QDRANT_URL.startswith(('http://', 'https://')):
            errors.append("QDRANT_URL must start with http:// or https://")

        # Validate port range
        if cls.QDRANT_PORT < 1 or cls.QDRANT_PORT > 65535:
            errors.append("QDRANT_PORT must be between 1 and 65535")

        if cls.API_PORT < 1 or cls.API_PORT > 65535:
            errors.append("API_PORT must be between 1 and 65535")

        # Validate collection name
        if not cls.QDRANT_COLLECTION_NAME:
            errors.append("QDRANT_COLLECTION_NAME cannot be empty")

        return errors

# Note: Configuration validation should be called explicitly by the application
# when it needs to validate configuration, rather than at import time.
# This prevents import errors when environment variables are not set during development.