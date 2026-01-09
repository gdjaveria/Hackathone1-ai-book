"""
Utility functions for text cleaning and processing in the embedding pipeline
"""
import re
from typing import List, Optional
from bs4 import BeautifulSoup
import logging

logger = logging.getLogger(__name__)

def clean_html_content(html_content: str) -> str:
    """
    Clean HTML content by removing navigation, headers, footers, and other non-content elements
    """
    if not html_content:
        return ""

    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove common navigation and layout elements
    for element in soup(['nav', 'header', 'footer', 'aside', 'script', 'style', 'meta', 'link']):
        element.decompose()

    # Remove elements with common class names for navigation/components
    for element in soup.find_all(class_=re.compile(r'navbar|nav|header|footer|sidebar|toc|menu|advertisement|ads')):
        element.decompose()

    # Get the cleaned text content
    text = soup.get_text()

    # Clean up excessive whitespace
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    return text


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Split text into chunks of specified size with overlap
    """
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If we're near the end, make sure to include the remaining text
        if end > len(text):
            end = len(text)

        chunk = text[start:end]
        chunks.append(chunk)

        # Move start position by chunk_size - overlap to create overlap
        start = end - overlap

        # If we've reached the end, break
        if start >= len(text):
            break

    return chunks


def extract_title_from_html(html_content: str) -> Optional[str]:
    """
    Extract the title from HTML content
    """
    try:
        soup = BeautifulSoup(html_content, 'html.parser')
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()

        # Try to find h1 as an alternative
        h1_tag = soup.find('h1')
        if h1_tag:
            return h1_tag.get_text().strip()

        return None
    except Exception as e:
        logger.error(f"Error extracting title from HTML: {e}")
        return None


def remove_boilerplate_text(text: str) -> str:
    """
    Remove common boilerplate text from documents
    """
    if not text:
        return ""

    # Remove common navigation text patterns
    patterns_to_remove = [
        r'previous\s+next',
        r'«\s+»',
        r'<\s+>',
        r'back\s+to\s+top',
        r'copyright\s+\d{4}',
        r'©\s+\d{4}',
    ]

    for pattern in patterns_to_remove:
        text = re.sub(pattern, '', text, flags=re.IGNORECASE)

    # Clean up excessive whitespace again after removals
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    return text


def normalize_text(text: str) -> str:
    """
    Normalize text by standardizing whitespace and other common issues
    """
    if not text:
        return ""

    # Replace multiple spaces with single space
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    return text