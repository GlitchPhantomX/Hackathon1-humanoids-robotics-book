# Deployment Guide: Multi-Language Translation System

## Overview
This guide provides step-by-step instructions for deploying the multi-language translation system for the Physical AI Robotics Textbook. The system consists of a Docusaurus frontend with translation capabilities and a FastAPI backend for authentication and translation services.

## Prerequisites

### System Requirements
- **Node.js**: Version 18 or higher
- **Python**: Version 3.8 or higher
- **npm** or **yarn** package manager
- **Git** for version control
- **Access to OpenAI API key** (for translation generation)

### Environment Requirements
- Web server capable of serving static files (for frontend)
- Backend server with Python support (for FastAPI backend)
- Database (PostgreSQL recommended)
- SSL certificate for secure connections

## Build Instructions

### Frontend (Docusaurus) Build

1. **Navigate to the frontend directory**:
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   ```

2. **Install dependencies**:
   ```bash
   npm install
   # OR
   yarn install
   ```

3. **Build the static site**:
   ```bash
   npm run build
   # OR
   yarn build
   ```

4. **Verify the build**:
   ```bash
   npm run serve
   # OR
   yarn serve
   ```
   The site will be available at `http://localhost:3000`

### Backend (FastAPI) Build

1. **Navigate to the backend directory**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   ```

2. **Create a virtual environment** (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables** (see Environment Variables section below)

## Environment Variables

### Frontend (.env file in docusaurus directory)
```bash
# API endpoints
REACT_APP_API_URL=http://your-backend-domain.com
REACT_APP_AUTH_API_URL=http://your-backend-domain.com

# OpenAI API key for translation generation
REACT_APP_OPENAI_API_KEY=your_openai_api_key_here

# Optional: Custom configuration
REACT_APP_DEFAULT_LANGUAGE=en
REACT_APP_ENABLE_TRANSLATION_LOGGING=true
```

### Backend (.env file in backend directory)
```bash
# Database configuration
DATABASE_URL=postgresql://username:password@localhost:5432/your_database_name
NEON_DATABASE_URL=your_neon_database_url

# Authentication
SECRET_KEY=your_very_long_secret_key_here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# OpenAI API
OPENAI_API_KEY=your_openai_api_key_here

# Frontend URL for CORS
FRONTEND_URL=http://your-frontend-domain.com

# Optional: Email configuration for auth
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your_email@gmail.com
SMTP_PASSWORD=your_app_password
```

## Deployment Steps

### Option 1: Deploy to Static Hosting (Netlify, Vercel, GitHub Pages)

1. **Build the frontend** (if not already done):
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm run build
   ```

2. **Deploy the `build` directory** to your static hosting provider
   - For Netlify: Drag and drop the `build` folder
   - For Vercel: Select the `build` folder during deployment
   - For GitHub Pages: Push the `build` folder to a `gh-pages` branch

3. **Configure custom domain** (if applicable) in your hosting provider

4. **Set up custom headers** (if needed) for proper caching:
   ```
   # Cache static assets
   /static/* 31536000
   /fonts/* 31536000
   ```

### Option 2: Deploy to Self-Hosted Server

1. **Prepare the build files**:
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm run build
   ```

2. **Upload build files** to your web server (e.g., to `/var/www/robotics-textbook`)

3. **Configure web server** (example for Nginx):
   ```nginx
   server {
       listen 80;
       server_name your-domain.com;

       root /var/www/robotics-textbook;
       index index.html;

       location / {
           try_files $uri $uri/ /index.html;
       }

       # Proxy API requests to backend
       location /api/ {
           proxy_pass http://localhost:8001/;
           proxy_set_header Host $host;
           proxy_set_header X-Real-IP $remote_addr;
           proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
           proxy_set_header X-Forwarded-Proto $scheme;
       }
   }
   ```

4. **Start the backend server**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   uvicorn main:app --host 0.0.0.0 --port 8001 --workers 4
   ```

### Backend Deployment

1. **Prepare the backend**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   ```

2. **Install production dependencies**:
   ```bash
   pip install gunicorn
   ```

3. **Start the backend with Gunicorn**:
   ```bash
   gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8001
   ```

4. **Set up process manager** (using systemd on Linux):
   Create `/etc/systemd/system/robotics-textbook-backend.service`:
   ```ini
   [Unit]
   Description=Robotics Textbook Backend
   After=network.target

   [Service]
   User=www-data
   Group=www-data
   WorkingDirectory=/path/to/physical-ai-robotics-textbook/backend
   ExecStart=/path/to/venv/bin/gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8001
   Restart=always

   [Install]
   WantedBy=multi-user.target
   ```

5. **Enable and start the service**:
   ```bash
   sudo systemctl enable robotics-textbook-backend
   sudo systemctl start robotics-textbook-backend
   ```

## Database Setup

### PostgreSQL Configuration
1. **Create the database**:
   ```sql
   CREATE DATABASE robotics_textbook;
   ```

2. **Run migrations** (if using Alembic):
   ```bash
   cd physical-ai-robotics-textbook/backend
   alembic upgrade head
   ```

3. **Set up initial data** (if needed):
   ```bash
   python scripts/setup_initial_data.py
   ```

## SSL/HTTPS Configuration

1. **Obtain SSL certificate** (using Let's Encrypt):
   ```bash
   sudo apt-get install certbot python3-certbot-nginx
   sudo certbot --nginx -d your-domain.com
   ```

2. **Update Nginx configuration** for HTTPS:
   ```nginx
   server {
       listen 443 ssl http2;
       server_name your-domain.com;

       ssl_certificate /path/to/cert.pem;
       ssl_certificate_key /path/to/private.key;

       # ... rest of configuration
   }
   ```

## Monitoring and Logging

### Frontend Logging
- Enable logging in production by setting `REACT_APP_ENABLE_TRANSLATION_LOGGING=true`
- Monitor browser console for translation loading errors
- Use analytics tools to track language usage

### Backend Logging
- Configure logging in `config.py`:
  ```python
  import logging
  logging.basicConfig(level=logging.INFO)
  ```

- Monitor backend logs:
  ```bash
  tail -f /var/log/robotics-textbook-backend.log
  ```

## Performance Optimization

### Frontend Optimizations
1. **Enable gzip compression** in web server
2. **Set proper caching headers** for static assets
3. **Use CDN** for static assets and fonts
4. **Implement lazy loading** for translation files

### Backend Optimizations
1. **Use connection pooling** for database connections
2. **Implement caching** for frequently accessed data
3. **Optimize API endpoints** for performance
4. **Use Redis** for session storage (optional)

## Troubleshooting

### Common Issues

**Issue**: Language toggle not appearing
- **Cause**: Frontend not built with translation system
- **Solution**: Ensure LanguageToggle component is properly integrated

**Issue**: Translation files not loading
- **Cause**: Incorrect file paths or permissions
- **Solution**: Verify translation files are in correct directory structure

**Issue**: Authentication not working
- **Cause**: Backend API endpoints not accessible
- **Solution**: Check backend server status and CORS configuration

**Issue**: RTL languages not rendering correctly
- **Cause**: Missing font files or CSS rules
- **Solution**: Verify font files are properly hosted and CSS includes RTL rules

### Debugging Steps
1. Check browser console for JavaScript errors
2. Verify network requests in browser dev tools
3. Check backend logs for API errors
4. Confirm environment variables are set correctly
5. Test API endpoints directly using curl or Postman

## Rollback Plan

In case of deployment issues:

1. **Frontend Rollback**: Replace current build with previous version
2. **Backend Rollback**: Revert to previous Git commit and restart service
3. **Database Rollback**: Use Alembic to downgrade if needed:
   ```bash
   alembic downgrade -1
   ```

## Maintenance Schedule

### Daily
- Monitor application logs
- Check translation loading success rates
- Verify authentication system

### Weekly
- Update translation files if needed
- Review performance metrics
- Check for security updates

### Monthly
- Review and update dependencies
- Audit translation quality
- Check SSL certificate expiration

---

*This deployment guide ensures the multi-language translation system is properly deployed with all features working correctly. For additional support, refer to the developer documentation or contact the development team.*