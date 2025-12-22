import { useState, useEffect } from 'react';

const TRANSLATION_API_URL = 'http://127.0.0.1:8000';

interface TranslationResponse {
  translated_content: string;
  chapter_id: string;
  cached: boolean;
}

interface UseTranslationReturn {
  language: 'en' | 'ur';
  loading: boolean;
  toggleLanguage: () => Promise<void>;
}

export default function useTranslation(chapterId: string): UseTranslationReturn {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    const savedLang = (localStorage.getItem('doc-language') || 'en') as 'en' | 'ur';
    setLanguage(savedLang);
  }, []);

  const toggleLanguage = async (): Promise<void> => {
    console.log('==========================================');
    console.log('🔄 Toggle clicked! Current:', language);
    console.log('📖 Chapter:', chapterId);
    console.log('==========================================');

    const newLang: 'en' | 'ur' = language === 'en' ? 'ur' : 'en';

    // Switch back to English
    if (newLang === 'en') {
      console.log('✅ Switching to English');
      setLanguage('en');
      localStorage.setItem('doc-language', 'en');
      window.location.reload();
      return;
    }

    console.log('🌐 Starting Urdu translation...');
    setLoading(true);

    try {
      const article = document.querySelector('article');
      
      if (!article) {
        console.error('❌ Article element not found');
        alert('Page content not found!');
        setLoading(false);
        return;
      }

      console.log('📄 Article element found');

      // ✅ Get all text nodes while skipping code blocks
      const walker = document.createTreeWalker(
        article,
        NodeFilter.SHOW_TEXT,
        {
          acceptNode: (node: Node) => {
            const parent = node.parentElement;
            if (!parent) return NodeFilter.FILTER_REJECT;
            
            const tagName = parent.tagName.toLowerCase();
            
            // Skip code blocks, scripts, styles
            if (['script', 'style', 'code', 'pre'].includes(tagName)) {
              return NodeFilter.FILTER_REJECT;
            }
            
            // Skip empty text nodes
            if (!node.textContent?.trim()) {
              return NodeFilter.FILTER_REJECT;
            }
            
            return NodeFilter.FILTER_ACCEPT;
          }
        }
      );

      const textNodes: Text[] = [];
      let node: Node | null;
      
      while ((node = walker.nextNode())) {
        textNodes.push(node as Text);
      }

      console.log(`📝 Found ${textNodes.length} text nodes to translate`);

      // Get all text content
      const allText = textNodes
        .map(n => n.textContent?.trim() || '')
        .filter(t => t.length > 0)
        .join('\n');

      console.log(`📤 Sending ${allText.length} characters for translation`);

      // Call translation API
      const response = await fetch(`${TRANSLATION_API_URL}/api/translation/urdu`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: 'test-user',
          chapter_id: chapterId || 'unknown',
          content: allText,
          source_language: 'en',
          target_language: 'ur',
        }),
      });

      console.log('📡 Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('❌ API Error:', errorText);
        throw new Error(`API failed with status ${response.status}`);
      }

      const data: TranslationResponse = await response.json();
      
      console.log('✅ Translation received:', data.translated_content.length, 'chars');
      console.log('📥 First 150 chars:', data.translated_content.substring(0, 150));

      // Check if translation has Urdu characters
      const urduChars = (data.translated_content.match(/[\u0600-\u06FF]/g) || []).length;
      console.log('📊 Urdu characters found:', urduChars);

      if (urduChars < 50) {
        console.warn('⚠️ Warning: Translation may have failed - not enough Urdu characters');
      }

      // ✅ Replace text nodes with translated content
      const translatedLines = data.translated_content
        .split('\n')
        .filter(line => line.trim().length > 0);

      console.log(`🔄 Replacing ${textNodes.length} text nodes with ${translatedLines.length} translated lines`);

      textNodes.forEach((node, index) => {
        if (translatedLines[index]) {
          node.textContent = translatedLines[index];
        }
      });

      // ✅ Apply RTL attributes to article
      article.setAttribute('dir', 'rtl');
      article.setAttribute('lang', 'ur');
      
      // Apply inline styles as backup
      article.style.direction = 'rtl';
      article.style.textAlign = 'right';
      article.style.fontFamily = "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', Arial, sans-serif";
      article.style.fontSize = '1.15rem';
      article.style.lineHeight = '2.2';

      // Update state
      setLanguage('ur');
      localStorage.setItem('doc-language', 'ur');

      console.log('✅ Translation applied successfully!');
      console.log('✅ RTL and Urdu font styling applied!');
      console.log('==========================================');

    } catch (error) {
      console.error('❌ Translation error:', error);
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      alert(
        `Translation failed!\n\n` +
        `Error: ${errorMessage}\n\n` +
        `Troubleshooting:\n` +
        `1. Check if backend is running at ${TRANSLATION_API_URL}\n` +
        `2. Open browser console for detailed logs\n` +
        `3. Visit ${TRANSLATION_API_URL}/ to verify backend health`
      );
    } finally {
      setLoading(false);
    }
  };

  return { language, loading, toggleLanguage };
}