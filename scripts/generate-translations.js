/**
 * Translation Generation Script
 *
 * This script translates educational content from English to Urdu and Arabic using OpenAI API.
 * It preserves HTML tags, CSS classes, and code blocks while translating text content.
 *
 * Usage: node scripts/generate-translations.js [language] [chapter]
 * Example: node scripts/generate-translations.js ur 00-introduction
 */

const fs = require('fs').promises;
const path = require('path');
const OpenAI = require('openai');

// Configuration
const OPENAI_API_KEY = process.env.OPENAI_API_KEY;
if (!OPENAI_API_KEY) {
  console.error('Error: OPENAI_API_KEY environment variable is required');
  process.exit(1);
}

const client = new OpenAI({
  apiKey: OPENAI_API_KEY,
});

// Source and target languages
const SOURCE_LANGUAGE = 'en';
const TARGET_LANGUAGES = ['ur', 'ar'];
const CHAPTERS = [
  '00-introduction',
  '01-ros2',
  '02-simulation',
  '03-isaac',
  '04-vla',
  '05-capstone'
];

// Base paths
const DOCS_PATH = path.join(__dirname, '..', 'physical-ai-robotics-textbook', 'docusaurus', 'docs');
const TRANSLATIONS_PATH = path.join(__dirname, '..', 'physical-ai-robotics-textbook', 'docusaurus', 'src', 'translations');

/**
 * Validates that HTML tags are preserved in translation
 */
function validateHTMLTags(original, translated) {
  // Extract all HTML tags from original content
  const originalTags = (original.match(/<[^>]+>/g) || []).sort();
  const translatedTags = (translated.match(/<[^>]+>/g) || []).sort();

  // Compare tag counts
  const originalTagCounts = {};
  const translatedTagCounts = {};

  originalTags.forEach(tag => {
    originalTagCounts[tag] = (originalTagCounts[tag] || 0) + 1;
  });

  translatedTags.forEach(tag => {
    translatedTagCounts[tag] = (translatedTagCounts[tag] || 0) + 1;
  });

  // Check if all tags are present in the same quantity
  for (const tag in originalTagCounts) {
    if (originalTagCounts[tag] !== translatedTagCounts[tag]) {
      return false;
    }
  }

  for (const tag in translatedTagCounts) {
    if (originalTagCounts[tag] !== translatedTagCounts[tag]) {
      return false;
    }
  }

  return true;
}

/**
 * Validates that CSS classes are preserved in translation
 */
function validateCSSClasses(original, translated) {
  // Extract CSS classes from original content
  const originalClasses = (original.match(/class="([^"]*)"/g) || []).sort();
  const translatedClasses = (translated.match(/class="([^"]*)"/g) || []).sort();

  // Compare class counts
  const originalClassCounts = {};
  const translatedClassCounts = {};

  originalClasses.forEach(cls => {
    originalClassCounts[cls] = (originalClassCounts[cls] || 0) + 1;
  });

  translatedClasses.forEach(cls => {
    translatedClassCounts[cls] = (translatedClassCounts[cls] || 0) + 1;
  });

  // Check if all classes are present in the same quantity
  for (const cls in originalClassCounts) {
    if (originalClassCounts[cls] !== translatedClassCounts[cls]) {
      return false;
    }
  }

  for (const cls in translatedClassCounts) {
    if (originalClassCounts[cls] !== translatedClassCounts[cls]) {
      return false;
    }
  }

  return true;
}

/**
 * Validates that code blocks are preserved in translation
 */
function validateCodeBlocks(original, translated) {
  // Extract code blocks from original content
  const originalCodeBlocks = (original.match(/(```[\s\S]*?```|`[^`]*`|<code>[\s\S]*?<\/code>)/g) || []);
  const translatedCodeBlocks = (translated.match(/(```[\s\S]*?```|`[^`]*`|<code>[\s\S]*?<\/code>)/g) || []);

  // For now, just check if the number of code blocks is preserved
  return originalCodeBlocks.length === translatedCodeBlocks.length;
}

/**
 * Validates translation quality
 */
function validateTranslation(original, translated) {
  return {
    htmlTagsPreserved: validateHTMLTags(original, translated),
    classesPreserved: validateCSSClasses(original, translated),
    codeIntact: validateCodeBlocks(original, translated),
    valid: true // Placeholder - can add more validation checks
  };
}

/**
 * Translates content using OpenAI API
 */
async function translateContent(content, targetLanguage) {
  const languageNames = {
    ur: 'Urdu',
    ar: 'Arabic'
  };

  const systemPrompt = `You are a professional technical translator for robotics education.

RULES:
1. Preserve ALL HTML tags exactly as they are in the original
2. Preserve ALL CSS classes and IDs exactly as they are in the original
3. Keep code blocks in English (do not translate code)
4. Use technical term transliteration where appropriate
5. Maintain professional educational tone
6. Preserve all formatting and structure`;

  const userPrompt = `Translate this educational content from English to ${languageNames[targetLanguage]}.

CRITICAL: Preserve ALL HTML tags, CSS classes, and IDs exactly as they are in the original.
Only translate text content, not code blocks.
Keep code blocks in English.

Content to translate:
${content}`;

  try {
    const response = await client.chat.completions.create({
      model: 'gpt-4',
      messages: [
        { role: 'system', content: systemPrompt },
        { role: 'user', content: userPrompt }
      ],
      temperature: 0.3,
      max_tokens: 4000, // Adjust based on content length
    });

    return response.choices[0].message.content.trim();
  } catch (error) {
    console.error(`Error translating to ${targetLanguage}:`, error);
    throw error;
  }
}

/**
 * Reads markdown file and extracts content
 */
async function readMarkdownFile(filePath) {
  try {
    const content = await fs.readFile(filePath, 'utf8');
    return content;
  } catch (error) {
    console.error(`Error reading file ${filePath}:`, error);
    throw error;
  }
}

/**
 * Creates translation file structure
 */
function createTranslationStructure(content, targetLanguage, chapter, sourceFile) {
  const languageNames = {
    ur: 'Urdu',
    ar: 'Arabic'
  };

  return {
    meta: {
      title: `Translation of ${chapter}`,
      description: `Translated content for ${chapter} in ${languageNames[targetLanguage]}`,
      language: targetLanguage,
      chapter: chapter,
      sourceFile: sourceFile,
      lastUpdated: new Date().toISOString(),
    },
    content: {
      headings: {},
      paragraphs: {},
      lists: {},
    },
    html: content,
  };
}

/**
 * Saves translation to JSON file
 */
async function saveTranslation(translationData, targetLanguage, chapter, fileName) {
  try {
    // Create directory if it doesn't exist
    const chapterDir = path.join(TRANSLATIONS_PATH, targetLanguage, chapter);
    await fs.mkdir(chapterDir, { recursive: true });

    // Save translation
    const filePath = path.join(chapterDir, `${fileName}.json`);
    await fs.writeFile(filePath, JSON.stringify(translationData, null, 2));

    console.log(`✓ Saved translation: ${filePath}`);
  } catch (error) {
    console.error(`Error saving translation ${targetLanguage}/${chapter}/${fileName}.json:`, error);
    throw error;
  }
}

/**
 * Translates a single chapter
 */
async function translateChapter(chapter, targetLanguage) {
  console.log(`\n🔄 Translating ${chapter} to ${targetLanguage}...`);

  const chapterPath = path.join(DOCS_PATH, chapter);
  const files = await fs.readdir(chapterPath);

  // Process each markdown file in the chapter
  for (const file of files) {
    if (path.extname(file) === '.md') {
      const fileName = path.basename(file, '.md');
      const filePath = path.join(chapterPath, file);

      console.log(`  📄 Processing: ${file}`);

      try {
        // Read original content
        const originalContent = await readMarkdownFile(filePath);

        // Translate content
        console.log(`    🌐 Translating...`);
        const translatedContent = await translateContent(originalContent, targetLanguage);

        // Validate translation
        console.log(`    ✅ Validating...`);
        const validation = validateTranslation(originalContent, translatedContent);

        if (!validation.htmlTagsPreserved || !validation.classesPreserved || !validation.codeIntact) {
          console.warn(`    ⚠️  Validation issues detected for ${file}:`);
          if (!validation.htmlTagsPreserved) console.warn(`      HTML tags not preserved`);
          if (!validation.classesPreserved) console.warn(`      CSS classes not preserved`);
          if (!validation.codeIntact) console.warn(`      Code blocks not preserved`);

          // Retry with more specific instructions if validation fails
          console.log(`    🔄 Retrying translation with more specific instructions...`);
          const retryContent = await translateContentWithSpecificInstructions(originalContent, targetLanguage);
          const retryValidation = validateTranslation(originalContent, retryContent);

          if (retryValidation.htmlTagsPreserved && retryValidation.classesPreserved && retryValidation.codeIntact) {
            console.log(`    ✅ Retry successful`);
            await saveTranslation(
              createTranslationStructure(retryContent, targetLanguage, chapter, file),
              targetLanguage,
              chapter,
              fileName
            );
          } else {
            console.error(`    ❌ Retry failed, saving with validation warnings`);
            await saveTranslation(
              createTranslationStructure(translatedContent, targetLanguage, chapter, file),
              targetLanguage,
              chapter,
              fileName
            );
          }
        } else {
          // Validation passed, save translation
          await saveTranslation(
            createTranslationStructure(translatedContent, targetLanguage, chapter, file),
            targetLanguage,
            chapter,
            fileName
          );
        }
      } catch (error) {
        console.error(`    ❌ Error processing ${file}:`, error);
      }
    }
  }
}

/**
 * Translate content with more specific instructions for validation failures
 */
async function translateContentWithSpecificInstructions(content, targetLanguage) {
  const languageNames = {
    ur: 'Urdu',
    ar: 'Arabic'
  };

  const systemPrompt = `You are a professional technical translator for robotics education.

RULES:
1. Preserve ALL HTML tags exactly as they are in the original - match opening and closing tags precisely
2. Preserve ALL CSS classes and IDs exactly as they are in the original - do not modify class="..." or id="..." attributes
3. Keep code blocks in English (do not translate anything inside \`\`\` or \` or <code></code> tags)
4. Use technical term transliteration where appropriate
5. Maintain professional educational tone
6. Preserve all formatting and structure
7. If the original has <p class="example">, the translation must also have <p class="example">
8. If the original has <h2 id="section-1">, the translation must also have <h2 id="section-1">`;

  const userPrompt = `Translate this educational content from English to ${languageNames[targetLanguage]}.

CRITICAL: Preserve ALL HTML tags, CSS classes, and IDs exactly as they are in the original.
Only translate text content, not code blocks.
Keep code blocks in English.

Here is the content to translate:
${content}

REMEMBER: Every HTML tag, CSS class, and ID from the original must appear exactly the same in your translation.`;

  try {
    const response = await client.chat.completions.create({
      model: 'gpt-4',
      messages: [
        { role: 'system', content: systemPrompt },
        { role: 'user', content: userPrompt }
      ],
      temperature: 0.2, // Lower temperature for more consistent output
      max_tokens: 4000,
    });

    return response.choices[0].message.content.trim();
  } catch (error) {
    console.error(`Error translating to ${targetLanguage}:`, error);
    throw error;
  }
}

/**
 * Main function to run the translation process
 */
async function main() {
  console.log('🚀 Starting Translation Generation Process\n');

  // Get command line arguments
  const args = process.argv.slice(2);
  const targetLanguage = args[0];
  const chapter = args[1];

  if (args.length === 0) {
    // Translate all languages and chapters
    console.log('🌍 Translating all chapters to all languages...');

    for (const lang of TARGET_LANGUAGES) {
      for (const chap of CHAPTERS) {
        await translateChapter(chap, lang);
      }
    }
  } else if (args.length === 1) {
    // Translate all chapters for a specific language
    if (!TARGET_LANGUAGES.includes(targetLanguage)) {
      console.error(`❌ Invalid language: ${targetLanguage}. Valid options: ${TARGET_LANGUAGES.join(', ')}`);
      process.exit(1);
    }

    console.log(`🌍 Translating all chapters to ${targetLanguage}...`);

    for (const chap of CHAPTERS) {
      await translateChapter(chap, targetLanguage);
    }
  } else if (args.length === 2) {
    // Translate specific chapter for specific language
    if (!TARGET_LANGUAGES.includes(targetLanguage)) {
      console.error(`❌ Invalid language: ${targetLanguage}. Valid options: ${TARGET_LANGUAGES.join(', ')}`);
      process.exit(1);
    }

    if (!CHAPTERS.includes(chapter)) {
      console.error(`❌ Invalid chapter: ${chapter}. Valid options: ${CHAPTERS.join(', ')}`);
      process.exit(1);
    }

    console.log(`🌍 Translating ${chapter} to ${targetLanguage}...`);
    await translateChapter(chapter, targetLanguage);
  } else {
    console.log('Usage:');
    console.log('  node scripts/generate-translations.js                    # Translate all chapters to all languages');
    console.log('  node scripts/generate-translations.js ur                 # Translate all chapters to Urdu only');
    console.log('  node scripts/generate-translations.js ur 00-introduction # Translate specific chapter to specific language');
    process.exit(1);
  }

  console.log('\n✅ Translation Generation Process Completed!');
}

// Run the main function
if (require.main === module) {
  main().catch(error => {
    console.error('❌ Fatal error:', error);
    process.exit(1);
  });
}

// Export functions for testing
module.exports = {
  validateHTMLTags,
  validateCSSClasses,
  validateCodeBlocks,
  validateTranslation,
  translateContent,
  createTranslationStructure,
  translateChapter,
  main
};