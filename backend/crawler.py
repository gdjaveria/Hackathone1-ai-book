"""
Web crawler for Docusaurus websites - handles URL discovery, content fetching, and text extraction
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse, urlunparse
from typing import List, Dict, Optional, Set
import time
import logging
import xml.etree.ElementTree as ET
from backend.utils import clean_html_content, extract_title_from_html, remove_boilerplate_text, normalize_text

logger = logging.getLogger(__name__)

class DocusaurusCrawler:
    def __init__(self, base_url: str, delay: float = 1.0, timeout: int = 30):
        """
        Initialize the Docusaurus crawler
        :param base_url: The base URL of the Docusaurus site to crawl
        :param delay: Delay between requests in seconds to respect rate limits
        :param timeout: Request timeout in seconds
        """
        self.base_url = base_url
        self.delay = delay
        self.timeout = timeout
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

        # Track visited URLs to avoid infinite loops
        self.visited_urls: Set[str] = set()

        # Track all discovered URLs
        self.discovered_urls: Set[str] = set()

    def is_valid_url(self, url: str) -> bool:
        """
        Check if URL is valid and belongs to the same domain
        """
        parsed_base = urlparse(self.base_url)
        parsed_url = urlparse(url)

        # Same domain check
        if parsed_base.netloc != parsed_url.netloc:
            return False

        # Check if it's an HTML page
        if any(url.lower().endswith(ext) for ext in ['.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe']):
            return False

        return True

    def get_urls_from_sitemap(self) -> Set[str]:
        """
        Get URLs from sitemap.xml
        """
        sitemap_url = urljoin(self.base_url, 'sitemap.xml')
        discovered_urls = set()

        try:
            logger.info(f"Attempting to fetch sitemap from {sitemap_url}")
            response = self.session.get(sitemap_url, timeout=self.timeout)
            response.raise_for_status()

            # Parse the sitemap XML
            root = ET.fromstring(response.content)

            # Handle both sitemap index files and regular sitemaps
            namespaces = {
                'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
                'xhtml': 'http://www.w3.org/1999/xhtml'
            }

            # Check if this is a sitemap index (sitemap of sitemaps)
            sitemap_tags = root.findall('sitemap:sitemap', namespaces)
            if sitemap_tags:
                logger.info(f"Found sitemap index with {len(sitemap_tags)} sub-sitemaps")
                for sitemap_tag in sitemap_tags:
                    loc = sitemap_tag.find('sitemap:loc', namespaces)
                    if loc is not None:
                        sub_sitemap_url = loc.text
                        logger.info(f"Processing sub-sitemap: {sub_sitemap_url}")
                        sub_sitemap_urls = self._get_urls_from_single_sitemap(sub_sitemap_url)
                        discovered_urls.update(sub_sitemap_urls)
            else:
                # This is a regular sitemap
                discovered_urls.update(self._get_urls_from_single_sitemap(sitemap_url))

        except requests.RequestException as e:
            logger.warning(f"Sitemap not found or not accessible ({sitemap_url}): {e}")
            logger.info("Falling back to link discovery method")
            return set()
        except ET.ParseError as e:
            logger.warning(f"Could not parse sitemap XML: {e}")
            logger.info("Falling back to link discovery method")
            return set()
        except Exception as e:
            logger.warning(f"Unexpected error reading sitemap: {e}")
            logger.info("Falling back to link discovery method")
            return set()

        logger.info(f"Found {len(discovered_urls)} URLs from sitemap")
        return discovered_urls

    def _get_urls_from_single_sitemap(self, sitemap_url: str) -> Set[str]:
        """
        Helper method to get URLs from a single sitemap file
        """
        urls = set()

        try:
            response = self.session.get(sitemap_url, timeout=self.timeout)
            response.raise_for_status()

            root = ET.fromstring(response.content)
            namespaces = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

            # Find all URL entries in the sitemap
            url_tags = root.findall('sitemap:url', namespaces)

            for url_tag in url_tags:
                loc = url_tag.find('sitemap:loc', namespaces)
                if loc is not None:
                    url = loc.text
                    if self.is_valid_url(url):
                        urls.add(url)

        except Exception as e:
            logger.warning(f"Error processing sitemap {sitemap_url}: {e}")

        return urls

    def discover_urls(self, use_sitemap: bool = True) -> List[str]:
        """
        Discover all URLs on the Docusaurus site using sitemap if available, otherwise crawl links
        """
        logger.info(f"Starting URL discovery for {self.base_url}")

        # First, try to get URLs from sitemap
        if use_sitemap:
            sitemap_urls = self.get_urls_from_sitemap()
            if sitemap_urls:
                logger.info(f"Using sitemap to discover {len(sitemap_urls)} URLs")
                self.discovered_urls = sitemap_urls
                return list(self.discovered_urls)
            else:
                logger.info("Sitemap not available, falling back to link discovery")

        # If sitemap is not available or not used, fall back to link discovery
        logger.info("Starting URL discovery by crawling links")

        # Start with the base URL
        urls_to_visit = [self.base_url]
        self.discovered_urls.add(self.base_url)

        while urls_to_visit:
            current_url = urls_to_visit.pop(0)

            if current_url in self.visited_urls:
                continue

            self.visited_urls.add(current_url)
            logger.info(f"Crawling: {current_url} ({len(self.visited_urls)} URLs processed)")

            try:
                # Respect rate limits
                time.sleep(self.delay)

                response = self.session.get(current_url, timeout=self.timeout)
                response.raise_for_status()

                soup = BeautifulSoup(response.content, 'html.parser')

                # Find all links on the page
                for link in soup.find_all('a', href=True):
                    href = link['href']

                    # Convert relative URLs to absolute
                    absolute_url = urljoin(current_url, href)

                    # Check if URL is valid and not already discovered
                    if self.is_valid_url(absolute_url) and absolute_url not in self.discovered_urls:
                        self.discovered_urls.add(absolute_url)
                        urls_to_visit.append(absolute_url)

            except requests.RequestException as e:
                logger.error(f"Error crawling {current_url}: {e}")
                continue
            except Exception as e:
                logger.error(f"Unexpected error crawling {current_url}: {e}")
                continue

        logger.info(f"URL discovery completed. Found {len(self.discovered_urls)} URLs")
        return list(self.discovered_urls)

    def fetch_page_content(self, url: str) -> Optional[Dict[str, str]]:
        """
        Fetch content from a single page
        :param url: URL to fetch content from
        :return: Dictionary with 'url', 'title', and 'content' keys
        """
        try:
            time.sleep(self.delay)  # Respect rate limits
            response = self.session.get(url, timeout=self.timeout)
            response.raise_for_status()

            # Extract title and content
            title = extract_title_from_html(response.text)
            raw_content = clean_html_content(response.text)
            cleaned_content = remove_boilerplate_text(raw_content)
            normalized_content = normalize_text(cleaned_content)

            return {
                'url': url,
                'title': title or '',
                'content': normalized_content
            }
        except requests.RequestException as e:
            logger.error(f"Error fetching content from {url}: {e}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error fetching content from {url}: {e}")
            return None

    def crawl_all_pages(self) -> List[Dict[str, str]]:
        """
        Crawl all discovered pages and extract content
        :return: List of dictionaries containing page data
        """
        all_pages = []
        urls_to_crawl = self.discover_urls()

        logger.info(f"Starting content extraction for {len(urls_to_crawl)} pages")

        for url in urls_to_crawl:
            page_data = self.fetch_page_content(url)
            if page_data:
                all_pages.append(page_data)

        logger.info(f"Content extraction completed. Extracted content from {len(all_pages)} pages")
        return all_pages


def crawl_docusaurus_site(base_url: str, delay: float = 1.0, timeout: int = 30, use_sitemap: bool = True) -> List[Dict[str, str]]:
    """
    Convenience function to crawl a Docusaurus site
    :param base_url: The base URL of the Docusaurus site to crawl
    :param delay: Delay between requests in seconds
    :param timeout: Request timeout in seconds
    :param use_sitemap: Whether to use sitemap.xml for URL discovery (default: True)
    :return: List of dictionaries containing page data
    """
    crawler = DocusaurusCrawler(base_url, delay, timeout)
    # Override crawl_all_pages to use sitemap if specified
    all_pages = []
    urls_to_crawl = crawler.discover_urls(use_sitemap=use_sitemap)

    logger.info(f"Starting content extraction for {len(urls_to_crawl)} pages")

    for url in urls_to_crawl:
        page_data = crawler.fetch_page_content(url)
        if page_data:
            all_pages.append(page_data)

    logger.info(f"Content extraction completed. Extracted content from {len(all_pages)} pages")
    return all_pages


def extract_text_from_url(url: str, timeout: int = 30) -> Optional[Dict[str, str]]:
    """
    Extract text content from a single URL
    :param url: URL to extract content from
    :param timeout: Request timeout in seconds
    :return: Dictionary with 'url', 'title', and 'content' keys
    """
    try:
        session = requests.Session()
        session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })

        response = session.get(url, timeout=timeout)
        response.raise_for_status()

        title = extract_title_from_html(response.text)
        raw_content = clean_html_content(response.text)
        cleaned_content = remove_boilerplate_text(raw_content)
        normalized_content = normalize_text(cleaned_content)

        return {
            'url': url,
            'title': title or '',
            'content': normalized_content
        }
    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return None