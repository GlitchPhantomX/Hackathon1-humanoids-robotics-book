# ڈیپلومنٹ گائیڈ: ملٹی لینگوئج ترجمہ سسٹم

## جائزہ
یہ گائیڈ فزیکل ای روبوٹکس ٹیکس بک کے لیے ملٹی لینگوئج ترجمہ سسٹم کو ڈیپلو کرنے کے لیے قدم بہ قدم ہدایات فراہم کرتا ہے۔ سسٹم میں ترجمہ کی صلاحیتوں کے ساتھ ایک ڈوکوسورس فرنٹ اینڈ اور تصدیق اور ترجمہ کی خدمات کے لیے ایک فاسٹ ای پی بیک اینڈ پر مشتمل ہے۔

## ضروریات

### سسٹم کی ضروریات
- **Node.js**: ورژن 18 یا اس سے زیادہ
- **پائی تھون**: ورژن 3.8 یا اس سے زیادہ
- **npm** یا **yarn** پیکج مینیجر
- **Git** ورژن کنٹرول کے لیے
- **OpenAI API کی کلید** (ترجمہ جنریشن کے لیے)

### ماحول کی ضروریات
- اسٹیٹک فائلز کو سرور کرنے کے قابل ویب سرور (فرنٹ اینڈ کے لیے)
- پائی تھون کی حمایت کے ساتھ بیک اینڈ سرور (فاسٹ ای پی بیک اینڈ کے لیے)
- ڈیٹا بیس (PostgreSQL تجویز کردہ)
- محفوظ کنکشنز کے لیے SSL سرٹیفکیٹ

## تعمیر کی ہدایات

### فرنٹ اینڈ (ڈوکوسورس) تعمیر

1. **فرنٹ اینڈ ڈائریکٹری میں جائیں**:
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   ```

2. **انحصاروں کو انسٹال کریں**:
   ```bash
   npm install
   # یا
   yarn install
   ```

3. **اسٹیٹک سائٹ کو تعمیر کریں**:
   ```bash
   npm run build
   # یا
   yarn build
   ```

4. **تعمیر کی تصدیق کریں**:
   ```bash
   npm run serve
   # یا
   yarn serve
   ```
   سائٹ `http://localhost:3000` پر دستیاب ہوگی

### بیک اینڈ (فاسٹ ای پی) تعمیر

1. **بیک اینڈ ڈائریکٹری میں جائیں**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   ```

2. **ویچوئل اینوائرمنٹ بنائیں** (تجویز کردہ):
   ```bash
   python -m venv venv
   source venv/bin/activate  # ونڈوز پر: venv\Scripts\activate
   ```

3. **پائی تھون انحصاروں کو انسٹال کریں**:
   ```bash
   pip install -r requirements.txt
   ```

4. **ماحول کی متغیرات سیٹ کریں** (ذیل میں ماحول کی متغیرات سیکشن دیکھیں)

## ماحول کی متغیرات

### فرنٹ اینڈ (docusaurus ڈائریکٹری میں .env فائل)

```bash
# API اینڈ پوائنٹس
REACT_APP_API_URL=http://your-backend-domain.com
REACT_APP_AUTH_API_URL=http://your-backend-domain.com

# ترجمہ جنریشن کے لیے OpenAI API کلید
REACT_APP_OPENAI_API_KEY=your_openai_api_key_here

# اختیاری: حسب ضرورت کنفیگریشن
REACT_APP_DEFAULT_LANGUAGE=en
REACT_APP_ENABLE_TRANSLATION_LOGGING=true
```

### بیک اینڈ (backend ڈائریکٹری میں .env فائل)

```bash
# ڈیٹا بیس کنفیگریشن
DATABASE_URL=postgresql://username:password@localhost:5432/your_database_name
NEON_DATABASE_URL=your_neon_database_url

# تصدیق
SECRET_KEY=your_very_long_secret_key_here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# OpenAI API
OPENAI_API_KEY=your_openai_api_key_here

# CORS کے لیے فرنٹ اینڈ URL
FRONTEND_URL=http://your-frontend-domain.com

# اختیاری: تصدیق کے لیے ای میل کنفیگریشن
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your_email@gmail.com
SMTP_PASSWORD=your_app_password
```

## ڈیپلومنٹ اسٹیپس

### آپشن 1: اسٹیٹک ہوسٹنگ کے لیے ڈیپلو کریں (Netlify، Vercel، GitHub Pages)

1. **فرنٹ اینڈ کو تعمیر کریں** (اگر پہلے سے نہیں ہوا):
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm run build
   ```

2. **تعمیر ڈائریکٹری کو اپنی اسٹیٹک ہوسٹنگ فراہم کنندہ پر ڈیپلو کریں**
   - Netlify کے لیے: ڈراگ اینڈ ڈراپ کریں `build` فولڈر
   - Vercel کے لیے: ڈیپلومنٹ کے دوران `build` فولڈر منتخب کریں
   - GitHub Pages کے لیے: `build` فولڈر کو `gh-pages` برانچ پر پش کریں

3. **اپنی ہوسٹنگ فراہم کنندہ میں حسب ضرورت ڈومین کنفیگر کریں**

4. ** مناسب کیش کے لیے حسب ضرورت ہیڈرز سیٹ کریں** (اگر ضروری ہو):
   ```
   # اسٹیٹک اثاثوں کو کیش کریں
   /static/* 31536000
   /fonts/* 31536000
   ```

### آپشن 2: خود ہوسٹڈ سرور پر ڈیپلو کریں

1. **تعمیر فائلز تیار کریں**:
   ```bash
   cd physical-ai-robotics-textbook/docusaurus
   npm run build
   ```

2. **ویب سرور پر تعمیر فائلز اپ لوڈ کریں** (مثال کے طور پر `/var/www/robotics-textbook` میں)

3. **ویب سرور کنفیگر کریں** (Nginx کی مثال):
   ```nginx
   server {
       listen 80;
       server_name your-domain.com;

       root /var/www/robotics-textbook;
       index index.html;

       location / {
           try_files $uri $uri/ /index.html;
       }

       # بیک اینڈ کے لیے API درخواستوں کو پراکسی کریں
       location /api/ {
           proxy_pass http://localhost:8001/;
           proxy_set_header Host $host;
           proxy_set_header X-Real-IP $remote_addr;
           proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
           proxy_set_header X-Forwarded-Proto $scheme;
       }
   }
   ```

4. **بیک اینڈ سرور شروع کریں**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   uvicorn main:app --host 0.0.0.0 --port 8001 --workers 4
   ```

### بیک اینڈ ڈیپلومنٹ

1. **بیک اینڈ تیار کریں**:
   ```bash
   cd physical-ai-robotics-textbook/backend
   ```

2. **پروڈکشن انحصاروں کو انسٹال کریں**:
   ```bash
   pip install gunicorn
   ```

3. **Gunicorn کے ساتھ بیک اینڈ شروع کریں**:
   ```bash
   gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8001
   ```

4. **پروسیس مینیجر سیٹ کریں** (لینکس پر systemd استعمال کرتے ہوئے):
   `/etc/systemd/system/robotics-textbook-backend.service` بنائیں:
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

5. **سروس کو فعال اور شروع کریں**:
   ```bash
   sudo systemctl enable robotics-textbook-backend
   sudo systemctl start robotics-textbook-backend
   ```

## ڈیٹا بیس سیٹ اپ

### PostgreSQL کنفیگریشن
1. **ڈیٹا بیس بنائیں**:
   ```sql
   CREATE DATABASE robotics_textbook;
   ```

2. **مایگریشنز چلائیں** (اگر Alembic استعمال کر رہے ہیں):
   ```bash
   cd physical-ai-robotics-textbook/backend
   alembic upgrade head
   ```

3. **اولیہ ڈیٹا سیٹ اپ کریں** (اگر ضروری ہو):
   ```bash
   python scripts/setup_initial_data.py
   ```

## SSL/HTTPS کنفیگریشن

1. **SSL سرٹیفکیٹ حاصل کریں** (Let's Encrypt استعمال کرتے ہوئے):
   ```bash
   sudo apt-get install certbot python3-certbot-nginx
   sudo certbot --nginx -d your-domain.com
   ```

2. **HTTPS کے لیے Nginx کنفیگریشن اپ ڈیٹ کریں**:
   ```nginx
   server {
       listen 443 ssl http2;
       server_name your-domain.com;

       ssl_certificate /path/to/cert.pem;
       ssl_certificate_key /path/to/private.key;

       # ... کنفیگریشن کا بقیہ حصہ
   }
   ```

## مانیٹرنگ اور لاگنگ

### فرنٹ اینڈ لاگنگ
- ڈیپلومنٹ میں لاگنگ کو فعال کریں `REACT_APP_ENABLE_TRANSLATION_LOGGING=true` سیٹ کر کے
- ترجمہ لوڈنگ کی خامیوں کے لیے براؤزر کنسول کو مانیٹر کریں
- زبان کے استعمال کو ٹریک کرنے کے لیے تجزیاتی ٹولز استعمال کریں

### بیک اینڈ لاگنگ
- `config.py` میں لاگنگ کنفیگر کریں:
  ```python
  import logging
  logging.basicConfig(level=logging.INFO)
  ```

- بیک اینڈ لاگس کو مانیٹر کریں:
  ```bash
  tail -f /var/log/robotics-textbook-backend.log
  ```

## کارکردگی کی اصلاح

### فرنٹ اینڈ اصلاحات
1. **ویب سرور میں gzip کمپریشن** فعال کریں
2. **اسٹیٹک اثاثوں کے لیے مناسب کیش ہیڈرز** سیٹ کریں
3. **اسٹیٹک اثاثوں اور فونٹس کے لیے CDN** استعمال کریں
4. **ترجمہ فائلز کے لیے لیزی لوڈنگ** نافذ کریں

### بیک اینڈ اصلاحات
1. **ڈیٹا بیس کنکشنز کے لیے کنکشن پولنگ** استعمال کریں
2. **اکثر رسائی والے ڈیٹا کے لیے کیش** نافذ کریں
3. **کارکردگی کے لیے API اینڈ پوائنٹس کو بہتر بنائیں**
4. **سیشن اسٹوریج کے لیے Redis** استعمال کریں (اختیاری)

## مسئلہ حل کرنا

### عام مسائل

**مسئلہ**: زبان ٹوگل ظاہر نہیں ہو رہا
- **وجہ**: فرنٹ اینڈ ترجمہ سسٹم کے ساتھ تعمیر نہیں کیا گیا
- **حل**: یقینی بنائیں کہ LanguageToggle کمپوننٹ مناسب طریقے سے انضمام ہے

**مسئلہ**: ترجمہ فائلز لوڈ نہیں ہو رہی
- **وجہ**: غلط فائل پاتھ یا اجازتیں
- **حل**: یقینی بنائیں کہ ترجمہ فائلز صحیح ڈائریکٹری سٹرکچر میں ہیں

**مسئلہ**: تصدیق کام نہیں کر رہی
- **وجہ**: بیک اینڈ API اینڈ پوائنٹس قابل رسائی نہیں ہیں
- **حل**: بیک اینڈ سرور کی حیثیت اور CORS کنفیگریشن چیک کریں

**مسئلہ**: RTL زبانیں صحیح طریقے سے رینڈر نہیں ہو رہی
- **وجہ**: فونٹ فائلز یا CSS قواعد غائب ہیں
- **حل**: یقینی بنائیں کہ فونٹ فائلز مناسب طریقے سے ہوسٹ کی گئی ہیں اور CSS میں RTL قواعد شامل ہیں

### ڈیبگنگ اسٹیپس
1. جاوا اسکرپٹ خامیوں کے لیے براؤزر کنسول چیک کریں
2. براؤزر ڈیو ٹولز میں نیٹ ورک درخواستوں کی تصدیق کریں
3. API خامیوں کے لیے بیک اینڈ لاگس چیک کریں
4. ماحول کی متغیرات صحیح طریقے سے سیٹ ہیں یہ تصدیق کریں
5. curl یا Postman کا استعمال کرتے ہوئے API اینڈ پوائنٹس کو براہ راست ٹیسٹ کریں

## رول بیک منصوبہ

ڈیپلومنٹ کے مسائل کی صورت میں:

1. **فرنٹ اینڈ رول بیک**: موجودہ تعمیر کو پچھلے ورژن کے ساتھ تبدیل کریں
2. **بیک اینڈ رول بیک**: Git کمٹ پر واپس جائیں اور سروس دوبارہ شروع کریں
3. **ڈیٹا بیس رول بیک**: ضرورت کے مطابق Alembic کا استعمال کریں:
   ```bash
   alembic downgrade -1
   ```

## دیکھ بھال کا شیڈول

### روزانہ
- ایپلی کیشن لاگس کی نگرانی کریں
- ترجمہ لوڈنگ کی کامیابی کی شرح کی جانچ کریں
- تصدیق سسٹم کی تصدیق کریں

### ہفتہ وار
- ترجمہ فائلز کی ضرورت کے مطابق اپ ڈیٹ کریں
- کارکردگی کے میٹرکس کا جائزہ لیں
- سیکورٹی اپ ڈیٹس کی جانچ کریں

### ماہانہ
- انحصاروں کو اپ ڈیٹ کریں
- ترجمہ کی معیار کا آڈٹ کریں
- SSL سرٹیفکیٹ کی میعاد ختم ہونے کی جانچ کریں

---

*یہ ڈیپلومنٹ گائیڈ یقینی بناتا ہے کہ ملٹی لینگوئج ترجمہ سسٹم تمام خصوصیات کے ساتھ مناسب طریقے سے ڈیپلو ہو جائے۔ مزید معاونت کے لیے ڈیولپر دستاویزات دیکھیں یا ترقی ٹیم سے رابطہ کریں۔*