# User Guide: Multi-Language Translation System

Welcome to the multi-language translation system for the Physical AI Robotics Textbook! This guide will help you navigate and use the translation features effectively.

## How to Change Language

1. **Locate the Language Toggle**: Look for the language toggle button in the top-right corner of the navigation bar. It displays the current language and has a globe icon (🌐).

2. **Login Required**: If you're not logged in, you'll see the toggle disabled with a message "Login required for translation". Please log in first.

3. **Open the Dropdown**: Once logged in, click on the language toggle button. You'll see a dropdown menu with three options:
   - 🇬🇧 English (default)
   - 🇵🇰 اردو (Urdu)
   - 🇸🇦 العربية (Arabic)

4. **Select Your Language**: Click on your preferred language from the dropdown. The page content will update immediately to display in your selected language.

5. **Your Preference is Saved**: Your language choice will be remembered across sessions using browser localStorage.

## Supported Languages

The system currently supports three languages:

| Language | Code | Direction | Status |
|----------|------|-----------|---------|
| English | en | Left-to-Right (LTR) | Default |
| Urdu | ur | Right-to-Left (RTL) | Complete |
| Arabic | ar | Right-to-Left (RTL) | Complete |

All textbook content has been professionally translated while preserving all formatting, code blocks, and diagrams.

## Login Requirement

**Authentication is required** to access the translation feature:

- The language toggle is disabled when not logged in
- You'll see a tooltip saying "Login required for translation"
- After logging in, the toggle becomes active
- Your authentication status is checked against the backend API at `http://localhost:8001/api/auth/session`

### Why Authentication is Required
- To maintain translation quality and prevent unauthorized changes
- To track usage and improve the service
- To provide personalized language preferences

## Troubleshooting

### Common Issues and Solutions

**Issue**: Language toggle is grayed out/disabled
- **Cause**: You're not logged in
- **Solution**: Log in to your account first, then try again

**Issue**: Content doesn't change after selecting a language
- **Cause**: Translation file may not be loaded
- **Solution**: Refresh the page or try a different language to verify functionality

**Issue**: Text appears garbled or with question marks
- **Cause**: Font loading issue
- **Solution**: Clear browser cache and reload the page

**Issue**: Arabic or Urdu text doesn't align properly
- **Cause**: RTL layout issue
- **Solution**: Refresh the page; the system should automatically apply correct RTL styling

**Issue**: Code blocks appear in the wrong language
- **Cause**: Content preservation issue
- **Solution**: This is expected behavior - code blocks remain in English to preserve technical accuracy

**Issue**: Translation takes too long to load
- **Cause**: Large content or slow network
- **Solution**: Wait for the content to load; translations are cached for faster subsequent access

## Screenshots

### Language Toggle in Navbar
```
┌─────────────────────────────────────────────────────────┐
│  Physical AI Robotics Textbook            🌐 English ▼  │
└─────────────────────────────────────────────────────────┘
```

### Dropdown Menu Open
```
┌─────────────────────────────────────────────────────────┐
│  Physical AI Robotics Textbook                          │
│                                            [▼]          │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ 🇬🇧 English                           ✓ Current    │ │
│  │ 🇵🇰 اردو                                          │ │
│  │ 🇸🇦 العربية                                        │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### RTL Language Display
When viewing Urdu or Arabic content, you'll notice:
- Text alignment shifts to the right
- Navigation elements adjust for RTL reading
- Code blocks remain left-aligned for readability
- All HTML structure and CSS classes are preserved

## Tips for Best Experience

1. **Use a modern browser** for best RTL support
2. **Ensure stable internet connection** for smooth translation loading
3. **Clear browser cache periodically** to get updated translations
4. **Report translation issues** to help improve quality
5. **Bookmark pages in your preferred language** - your language preference will be maintained

## Getting Help

If you encounter issues not covered in this guide:

1. Check that you're logged in to the system
2. Verify your internet connection
3. Try refreshing the page
4. Contact support if issues persist

---

*Last updated: December 2025*
*For technical documentation, see the Developer Documentation section.*