import { Request, Response, NextFunction } from 'express';

// Simple logger implementation
export const logger = {
  info: (message: string, meta?: any) => {
    const timestamp = new Date().toISOString();
    console.log(`[INFO] ${timestamp}: ${message}`, meta ? JSON.stringify(meta) : '');
  },

  error: (message: string, meta?: any) => {
    const timestamp = new Date().toISOString();
    console.error(`[ERROR] ${timestamp}: ${message}`, meta ? JSON.stringify(meta) : '');
  },

  warn: (message: string, meta?: any) => {
    const timestamp = new Date().toISOString();
    console.warn(`[WARN] ${timestamp}: ${message}`, meta ? JSON.stringify(meta) : '');
  },

  debug: (message: string, meta?: any) => {
    if (process.env.NODE_ENV !== 'production') {
      const timestamp = new Date().toISOString();
      console.log(`[DEBUG] ${timestamp}: ${message}`, meta ? JSON.stringify(meta) : '');
    }
  }
};

// Express middleware for request logging
export const requestLogger = (req: Request, res: Response, next: NextFunction) => {
  const startTime = Date.now();

  // Declare variables ONCE → usable everywhere
  const method = req.method;
  const url = req.url;
  const userAgent = req.get('User-Agent') || 'unknown';

  res.on('finish', () => {
    const duration = Date.now() - startTime;
    const statusCode = res.statusCode;

    if (statusCode >= 500) {
      logger.error(`Request failed`, {
        method,
        url,
        statusCode,
        duration: `${duration}ms`,
        userAgent
      });
    } else if (statusCode >= 400) {
      logger.warn(`Request warning`, {
        method,
        url,
        statusCode,
        duration: `${duration}ms`,
        userAgent
      });
    } else {
      logger.info(`Request completed`, {
        method,
        url,
        statusCode,
        duration: `${duration}ms`,
        userAgent
      });
    }
  });

  // Debug log at request start
  logger.debug(`Request started`, {
    method,
    url,
    userAgent,
    ip: req.ip,
    body: req.body && Object.keys(req.body).length > 0 ? '[present]' : '[empty]'
  });

  next();
};

// Error logging middleware
export const errorLogger = (err: Error, req: Request, res: Response, next: NextFunction) => {
  logger.error(`Unhandled error`, {
    message: err.message,
    stack: err.stack,
    url: req.url,
    method: req.method,
    userAgent: req.get('User-Agent') || 'unknown'
  });

  next(err);
};
