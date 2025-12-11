"""
Test script to verify vector search functionality with sample queries.
"""
import asyncio
import os
import sys
from app.services.vector_search_service import vector_search_service
from app.config import settings


async def test_ros2_query():
    """Test the search functionality with 'What is ROS 2?' query."""
    print("Testing vector search with sample query: 'What is ROS 2?'")
    
    try:
        # Perform the search
        results = await vector_search_service.search(
            query="What is ROS 2?",
            top_k=5
        )
        
        print(f"Found {len(results)} results for query 'What is ROS 2?'")
        
        if results:
            print("\nTop 5 results:")
            for i, result in enumerate(results, 1):
                print(f"{i}. Score: {result['relevance_score']:.3f}")
                print(f"   Content preview: {result['content'][:100]}...")
                print(f"   Module: {result['module']}, Chapter: {result['chapter']}")
                print(f"   Source: {result['source_file']}")
                print()
                
                # Check if the content is relevant to ROS 2
                content_lower = result['content'].lower()
                if 'ros' in content_lower and '2' in content_lower:
                    print(f"✓ Result {i} appears relevant to ROS 2")
                else:
                    print(f"⚠ Result {i} may not be directly relevant to ROS 2")
                print()
        else:
            print("No results found. This might indicate:")
            print("- The document collection is empty")
            print("- The ingestion process hasn't been run yet")
            print("- The Qdrant connection isn't working properly")
            
        return results
        
    except Exception as e:
        print(f"Error during search: {str(e)}")
        import traceback
        traceback.print_exc()
        return None


async def test_selected_text_integration():
    """Test the search functionality with selected text."""
    print("\nTesting vector search with selected text integration...")
    
    query = "Explain this middleware concept"
    selected_text = "ROS 2 is a middleware for robotics applications"
    
    try:
        results = await vector_search_service.search_with_selected_text(
            query=query,
            selected_text=selected_text,
            top_k=3
        )
        
        print(f"Found {len(results)} results when combining query '{query}' with selected text")
        
        for i, result in enumerate(results, 1):
            print(f"{i}. Score: {result['relevance_score']:.3f}")
            print(f"   Content preview: {result['content'][:100]}...")
            print()
            
        return results
        
    except Exception as e:
        print(f"Error during search with selected text: {str(e)}")
        import traceback
        traceback.print_exc()
        return None


async def test_filtering():
    """Test the search functionality with module/chapter filtering."""
    print("\nTesting vector search with filtering...")
    
    try:
        # Search for ROS 2 content in a specific module (if available)
        results = await vector_search_service.search_by_module_or_chapter(
            query="What is ROS 2?",
            module="ROS 2 Fundamentals",  # Adjust based on actual modules in the textbook
            top_k=3
        )
        
        print(f"Found {len(results)} results for 'What is ROS 2?' in 'ROS 2 Fundamentals' module")
        
        for i, result in enumerate(results, 1):
            print(f"{i}. Score: {result['relevance_score']:.3f}")
            print(f"   Content preview: {result['content'][:100]}...")
            print(f"   Module: {result['module']}, Chapter: {result['chapter']}")
            print()
            
        return results
        
    except Exception as e:
        print(f"Error during filtered search: {str(e)}")
        import traceback
        traceback.print_exc()
        return None


async def test_edge_cases():
    """Test edge cases like empty results and short queries."""
    print("\nTesting edge cases...")
    
    # Test short query validation
    try:
        is_valid = await vector_search_service.validate_search_query("ab")
        print(f"Is 'ab' a valid query? {is_valid} (should be False)")
        
        is_valid = await vector_search_service.validate_search_query("What is ROS 2?")
        print(f"Is 'What is ROS 2?' a valid query? {is_valid} (should be True)")
    except Exception as e:
        print(f"Error validating queries: {str(e)}")
    
    # Test empty results (this depends on the actual data in Qdrant)
    try:
        # This query is unlikely to match anything in a robotics textbook
        results = await vector_search_service.search("asdkfjlasdjfklsdjf", top_k=5)
        print(f"Found {len(results)} results for random query (might be 0 if working correctly)")
    except Exception as e:
        print(f"Error testing empty results: {str(e)}")


async def main():
    """Main function to run all tests."""
    print("Starting vector search functionality tests...\n")

    # Test main query
    await test_ros2_query()

    # Test selected text integration
    await test_selected_text_integration()

    # Test filtering
    await test_filtering()

    # Test edge cases
    await test_edge_cases()

    # Get search statistics
    try:
        stats = await vector_search_service.get_search_statistics()
        print("\nSearch System Statistics:")
        print(f"- Total Documents: {stats['total_documents']}")
        print(f"- Vector Size: {stats['vector_size']}")
        print(f"- Collection: {stats['collection_name']}")
        print(f"- Status: {stats['status']}")
    except Exception as e:
        print(f"\nError getting search statistics: {str(e)}")

    print("\nVector search tests completed!")


if __name__ == "__main__":
    # Set up the environment to use the correct settings
    # Since this is only for testing, we'll just run the tests
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"Main execution error: {str(e)}")