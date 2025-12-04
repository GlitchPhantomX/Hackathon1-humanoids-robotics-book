from database import engine, Base
import os
from dotenv import load_dotenv

# Load environment variables (especially DATABASE_URL)
load_dotenv()

def create_db_tables():
    print("Creating database tables...")
    Base.metadata.create_all(engine)
    print("Database tables created successfully.")

if __name__ == "__main__":
    # Ensure DATABASE_URL is set before trying to create tables
    if not os.getenv("DATABASE_URL"):
        print("Error: DATABASE_URL environment variable is not set.")
        print("Please create a .env file in the same directory as database.py and create_tables.py with:")
        print("DATABASE_URL=\"postgresql://user:password@host/db?sslmode=require\"")
        exit(1)
    
    create_db_tables()

