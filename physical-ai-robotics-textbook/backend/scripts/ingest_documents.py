#!/usr/bin/env python3
"""
Document ingestion script for the RAG Chatbot.
Parses documents, chunks them, generates embeddings, and stores them in Qdrant.
"""
import asyncio
import sys
import os
from pathlib import Path
from typing import List, Dict, Any
import time
import argparse

# Add the backend directory to the Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.embedding_service import embedding_service
from app.services.qdrant_service import qdrant_service
from app.utils.markdown_parser import markdown_parser
from app.utils.text_processing import text_processor
from app.config import settings
from app.utils.logger import get_logger


async def ingest_single_document(
    file_path: str, force_refresh: bool = False
) -> Dict[str, Any]:
    """
    Ingest a single document: parse, chunk, embed, and store.

    Args:
        file_path: Path to the document to ingest
        force_refresh: Whether to overwrite existing embeddings

    Returns:
        Dictionary with ingestion results
    """
    logger = get_logger(f"ingest_{Path(file_path).stem}")
    start_time = time.time()

    try:
        logger.info(f"Starting ingestion for: {file_path}")

        # Parse the document
        parsed_doc = markdown_parser.parse_markdown_file(file_path)
        content = parsed_doc["content"]
        module = parsed_doc["module"]
        chapter = parsed_doc["chapter"]
        source_file = parsed_doc["file_path"]

        logger.info(f"Parsed document: {module}/{chapter}")

        # Chunk the content
        chunks = text_processor.chunk_text(content)
        logger.info(f"Created {len(chunks)} chunks")

        # Process each chunk
        stored_count = 0
        for i, chunk in enumerate(chunks):
            # Preprocess the chunk for embedding
            processed_content = text_processor.preprocess_for_embedding(
                chunk["content"]
            )

            if not processed_content:
                logger.warning(f"Skipping empty chunk {i} in {file_path}")
                continue

            # Generate embedding
            embedding = await embedding_service.generate_embedding(processed_content)

            # Store in Qdrant
            doc_id = await qdrant_service.store_embedding(
                content=processed_content,
                embedding=embedding,
                module=module,
                chapter=chapter,
                source_file=source_file,
                chunk_index=chunk["chunk_index"],
                metadata={
                    "original_length": len(chunk["content"]),
                    "processed_length": len(processed_content),
                    "start_pos": chunk["start_pos"],
                    "end_pos": chunk["end_pos"],
                },
            )

            stored_count += 1
            logger.debug(f"Stored chunk {i} with ID: {doc_id}")

        duration = time.time() - start_time
        logger.info(
            f"Completed ingestion for {file_path} in {duration:.2f}s ({stored_count} chunks stored)"
        )

        return {
            "file_path": file_path,
            "module": module,
            "chapter": chapter,
            "chunks_processed": len(chunks),
            "chunks_stored": stored_count,
            "processing_time": duration,
            "status": "success",
        }

    except Exception as e:

        logger.error(f"Error ingesting document {file_path}: {str(e)}")

        return {"file_path": file_path, "error": str(e), "status": "error"}


async def ingest_directory(
    directory_path: str, force_refresh: bool = False, file_extensions: List[str] = None
) -> Dict[str, Any]:
    """
    Ingest all documents in a directory.

    Args:
        directory_path: Path to the directory containing documents
        force_refresh: Whether to overwrite existing embeddings
        file_extensions: List of file extensions to process (default: ['.md'])

    Returns:
        Dictionary with overall ingestion results
    """
    if file_extensions is None:
        file_extensions = [".md"]

    logger = get_logger("ingest_directory")
    start_time = time.time()

    # Find all markdown files in the directory (recursively)
    directory = Path(directory_path)
    if not directory.exists():
        raise FileNotFoundError(f"Directory not found: {directory_path}")

    files = []
    for ext in file_extensions:
        files.extend(directory.rglob(f"*{ext}"))

    logger.info(f"Found {len(files)} files to process")

    # If force refresh, delete the existing collection first
    if force_refresh:
        logger.info("Force refresh enabled - deleting existing collection")
        try:
            await qdrant_service.delete_collection()
            logger.info("Existing collection deleted, creating new one...")
            await qdrant_service.init_collection()
        except Exception as e:
            logger.error(f"Error during force refresh (collection deletion): {e}")
            # Continue anyway, as init_collection below will create it if it doesn't exist

    # Process each file
    results = []
    successful = 0
    failed = 0

    for i, file_path in enumerate(files):
        logger.info(f"Processing file {i+1}/{len(files)}: {file_path}")

        result = await ingest_single_document(str(file_path), force_refresh)
        results.append(result)

        if result["status"] == "success":
            successful += 1
        else:
            failed += 1

        # Log progress
        if (i + 1) % 10 == 0 or (i + 1) == len(files):
            logger.info(
                f"Progress: {i+1}/{len(files)} files processed ({successful} successful, {failed} failed)"
            )

    total_duration = time.time() - start_time

    # Calculate ingestion statistics for T041
    total_chunks_created = sum(
        result.get("chunks_stored", 0)
        for result in results
        if result.get("status") == "success"
    )

    summary = {
        "directory": str(directory_path),
        "files_found": len(files),
        "files_processed": len([r for r in results if "error" not in r]),
        "successful": successful,
        "failed": failed,
        "total_chunks_created": total_chunks_created,
        "total_processing_time": total_duration,
        "average_time_per_file": total_duration / len(files) if files else 0,
        "results": results,
    }

    logger.info(
        f"Ingestion completed: {successful} successful, {failed} failed in {total_duration:.2f}s"
    )

    logger.info(f"Total chunks created: {total_chunks_created}")

    return summary


async def validate_ingestion() -> Dict[str, Any]:
    """
    Validate the ingestion by checking the Qdrant collection.

    Returns:
        Dictionary with validation results
    """
    logger = get_logger("ingestion_validation")

    try:
        # Get collection info
        collection_info = await qdrant_service.get_collection_info()
        doc_count = await qdrant_service.count_documents()

        logger.info(f"Collection contains {doc_count} documents")

        validation_result = {
            "collection_exists": "error" not in collection_info,
            "document_count": doc_count,
            "collection_info": collection_info,
            "status": "valid",
        }

        return validation_result

    except Exception as e:

        logger.error(f"Error validating ingestion: {str(e)}")

        return {"error": str(e), "status": "error"}


async def main():
    """
    Main function to run the ingestion script.
    """
    parser = argparse.ArgumentParser(
        description="Document ingestion script for RAG Chatbot"
    )
    parser.add_argument("source", help="Path to document file or directory")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force refresh (overwrite existing embeddings)",
    )
    parser.add_argument(
        "--ext",
        nargs="+",
        default=[".md"],
        help="File extensions to process (default: .md)",
    )
    parser.add_argument(
        "--validate", action="store_true", help="Validate ingestion after processing"
    )

    args = parser.parse_args()

    print("RAG Chatbot Document Ingestion Script")
    print("=" * 50)

    # Initialize services
    print("Initializing services...")
    await qdrant_service.init_collection()
    print("V Services initialized")

    # Check if source is a file or directory
    source_path = Path(args.source)

    if source_path.is_file():

        # Process single file

        print(f"Ingesting single file: {source_path}")

        result = await ingest_single_document(str(source_path), args.force)

        print(f"V File ingestion completed: {result['status']}")

        if result["status"] == "error":

            print(f"  Error: {result.get('error', 'Unknown error')}")

        else:

            print(f"  Chunks stored: {result['chunks_stored']}")

    elif source_path.is_dir():

        # Process directory

        print(f"Ingesting directory: {source_path}")

        result = await ingest_directory(str(source_path), args.force, args.ext)

        print(f"V Directory ingestion completed")

        print(f"  Files processed: {result['files_processed']}")

        print(f"  Successful: {result['successful']}")

        print(f"  Failed: {result['failed']}")

        print(f"  Total time: {result['total_processing_time']:.2f}s")

    else:

        print(f"Error: Source path does not exist: {source_path}")

        return 1

    # Validate if requested

    if args.validate:

        print("\nValidating ingestion...")

        validation_result = validate_ingestion()

        if validation_result["status"] == "valid":

            print(
                f"V Validation passed: {validation_result['document_count']} documents in collection"
            )

        else:

            print(
                f"X Validation failed: {validation_result.get('error', 'Unknown error')}"
            )

            return 1

    print("\nV Document ingestion completed successfully!")

    return 0


if __name__ == "__main__":

    exit_code = asyncio.run(main())

    sys.exit(exit_code)
