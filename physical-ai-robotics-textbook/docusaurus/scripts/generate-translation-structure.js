/**
 * Script to generate translation file structure from English files
 * This script copies the English translation structure to Urdu and Arabic directories
 * and creates placeholder files for missing translations
 */

const fs = require('fs');
const path = require('path');

// Define the base path for translations
const basePath = path.join(__dirname, '..', 'src', 'translations');

// Define the languages to process
const languages = ['ur', 'ar'];

// Get all directories in the English translations folder
const enPath = path.join(basePath, 'en');
const enDirs = fs.readdirSync(enPath, { withFileTypes: true })
  .filter(dirent => dirent.isDirectory())
  .map(dirent => dirent.name);

console.log('Starting translation structure generation...');

// For each language, create the directory structure and copy English files
languages.forEach(lang => {
  const langPath = path.join(basePath, lang);

  // Create language directory if it doesn't exist
  if (!fs.existsSync(langPath)) {
    fs.mkdirSync(langPath, { recursive: true });
    console.log(`Created directory: ${langPath}`);
  }

  // For each subdirectory in English, create it in the target language
  enDirs.forEach(dir => {
    const enDirPath = path.join(enPath, dir);
    const langDirPath = path.join(langPath, dir);

    // Create subdirectory if it doesn't exist
    if (!fs.existsSync(langDirPath)) {
      fs.mkdirSync(langDirPath, { recursive: true });
      console.log(`Created directory: ${langDirPath}`);
    }

    // Get all JSON files in the English subdirectory
    const enFiles = fs.readdirSync(enDirPath)
      .filter(file => path.extname(file) === '.json');

    // Copy each English file to the target language directory if it doesn't exist
    enFiles.forEach(file => {
      const enFilePath = path.join(enDirPath, file);
      const langFilePath = path.join(langDirPath, file);

      if (!fs.existsSync(langFilePath)) {
        // Read the English file
        const enContent = fs.readFileSync(enFilePath, 'utf8');
        const enJson = JSON.parse(enContent);

        // Create a placeholder translation with English text for reference
        const placeholderJson = createPlaceholderTranslation(enJson, lang);
        const placeholderContent = JSON.stringify(placeholderJson, null, 2);

        // Write the placeholder file
        fs.writeFileSync(langFilePath, placeholderContent);
        console.log(`Created placeholder: ${langFilePath}`);
      } else {
        console.log(`Skipped existing: ${langFilePath}`);
      }
    });
  });
});

console.log('Translation structure generation completed!');

/**
 * Creates a placeholder translation with English text as reference
 * @param {Object} enJson - The English translation object
 * @param {string} lang - The target language code
 * @returns {Object} - The placeholder translation object
 */
function createPlaceholderTranslation(enJson, lang) {
  const result = {};

  for (const [key, value] of Object.entries(enJson)) {
    if (typeof value === 'string') {
      // Add a comment indicating this needs translation
      result[key] = `PLACEHOLDER_${lang.toUpperCase()}: ${value}`;
    } else if (typeof value === 'object' && value !== null) {
      // Recursively process nested objects
      result[key] = createPlaceholderTranslation(value, lang);
    } else {
      // Keep non-string, non-object values as they are
      result[key] = value;
    }
  }

  return result;
}