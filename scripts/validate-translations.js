/**
 * Validation Checks for Translation Quality
 *
 * This script provides functions to validate that translations
 * preserve HTML tags, CSS classes, and code blocks correctly.
 */

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
      console.error(`HTML tag mismatch: ${tag} appears ${originalTagCounts[tag]} times in original but ${translatedTagCounts[tag]} times in translation`);
      return false;
    }
  }

  for (const tag in translatedTagCounts) {
    if (originalTagCounts[tag] !== translatedTagCounts[tag]) {
      console.error(`HTML tag mismatch: ${tag} appears ${translatedTagCounts[tag]} times in translation but ${originalTagCounts[tag]} times in original`);
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
      console.error(`CSS class mismatch: ${cls} appears ${originalClassCounts[cls]} times in original but ${translatedClassCounts[cls]} times in translation`);
      return false;
    }
  }

  for (const cls in translatedClassCounts) {
    if (originalClassCounts[cls] !== translatedClassCounts[cls]) {
      console.error(`CSS class mismatch: ${cls} appears ${translatedClassCounts[cls]} times in translation but ${originalClassCounts[cls]} times in original`);
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
  if (originalCodeBlocks.length !== translatedCodeBlocks.length) {
    console.error(`Code block count mismatch: ${originalCodeBlocks.length} in original vs ${translatedCodeBlocks.length} in translation`);
    return false;
  }

  // Additional validation: Check that code content is preserved (for backtick code blocks)
  const originalCodeContent = originalCodeBlocks
    .filter(block => block.startsWith('```'))
    .map(block => block.slice(3, block.lastIndexOf('```')));

  const translatedCodeContent = translatedCodeBlocks
    .filter(block => block.startsWith('```'))
    .map(block => block.slice(3, block.lastIndexOf('```')));

  if (originalCodeContent.length !== translatedCodeContent.length) {
    console.error(`Code content mismatch: different number of code blocks`);
    return false;
  }

  // Check if code content is preserved (should be identical)
  for (let i = 0; i < originalCodeContent.length; i++) {
    if (originalCodeContent[i].trim() !== translatedCodeContent[i].trim()) {
      console.error(`Code content mismatch in block ${i}`);
      return false;
    }
  }

  return true;
}

/**
 * Validates translation quality comprehensively
 */
function validateTranslation(original, translated) {
  const results = {
    htmlTagsPreserved: validateHTMLTags(original, translated),
    classesPreserved: validateCSSClasses(original, translated),
    codeIntact: validateCodeBlocks(original, translated),
    overall: false
  };

  results.overall = results.htmlTagsPreserved && results.classesPreserved && results.codeIntact;

  return results;
}

/**
 * Main validation function for command line usage
 */
function main() {
  if (process.argv.length < 4) {
    console.log('Usage: node validate-translations.js <original-file> <translated-file>');
    process.exit(1);
  }

  const fs = require('fs');

  try {
    const originalFile = process.argv[2];
    const translatedFile = process.argv[3];

    const originalContent = fs.readFileSync(originalFile, 'utf8');
    const translatedContent = fs.readFileSync(translatedFile, 'utf8');

    console.log('🔍 Validating translation...\n');

    const results = validateTranslation(originalContent, translatedContent);

    console.log('📊 Validation Results:');
    console.log(`  HTML Tags Preserved: ${results.htmlTagsPreserved ? '✅' : '❌'}`);
    console.log(`  CSS Classes Preserved: ${results.classesPreserved ? '✅' : '❌'}`);
    console.log(`  Code Blocks Intact: ${results.codeIntact ? '✅' : '❌'}`);
    console.log(`  Overall Status: ${results.overall ? '✅ PASSED' : '❌ FAILED'}`);

    process.exit(results.overall ? 0 : 1);
  } catch (error) {
    console.error('❌ Error during validation:', error.message);
    process.exit(1);
  }
}

// Export functions for use in other modules
module.exports = {
  validateHTMLTags,
  validateCSSClasses,
  validateCodeBlocks,
  validateTranslation
};

// Run main function if this script is executed directly
if (require.main === module) {
  main();
}