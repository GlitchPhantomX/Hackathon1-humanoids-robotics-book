import { drizzle } from 'drizzle-orm/node-postgres';
import { Pool } from 'pg';
import * as schema from './schema.js';

if (!process.env.DATABASE_URL) {
  throw new Error('DATABASE_URL environment variable is required');
}

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 20, // Maximum number of clients in the pool
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 2000,
});

// Test connection on startup
pool.on('connect', () => {
  console.log('✅ PostgreSQL database connected successfully');
});

pool.on('error', (err) => {
  console.error('❌ Unexpected database error:', err);
});

// Test the connection immediately
(async () => {
  try {
    const client = await pool.connect();
    console.log('✅ Database pool initialized');
    await client.query('SELECT NOW()'); // Test query
    client.release();
  } catch (err) {
    console.error('❌ Failed to connect to PostgreSQL database:', err);
    console.error('Make sure PostgreSQL is running on localhost:5432');
    console.error('Database name: physical_ai');
    process.exit(-1);
  }
})();

// Create Drizzle instance with schema
export const db = drizzle(pool, { schema });

// Export pool for cleanup if needed
export { pool };