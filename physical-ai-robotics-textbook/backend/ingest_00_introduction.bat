@echo off
REM Save as: ingest_00_introduction.bat
REM Run: ingest_00_introduction.bat

echo ====================================
echo Processing 00-introduction folder
echo ====================================

cd "C:\new - Copy\physical-ai-robotics-textbook\backend"

echo.
echo [1/6] Processing index.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\index.md"
timeout /t 2 >nul

echo.
echo [2/6] Processing 01-welcome.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\01-welcome.md"
timeout /t 2 >nul

echo.
echo [3/6] Processing 02-prerequisites.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\02-prerequisites.md"
timeout /t 2 >nul

echo.
echo [4/6] Processing 03-hardware-requirements.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\03-hardware-requirements.md"
timeout /t 2 >nul

echo.
echo [5/6] Processing 04-how-to-use.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\04-how-to-use.md"
timeout /t 2 >nul

echo.
echo [6/6] Processing 05-syllabus.md...
python single_file_ingest.py "C:\new - Copy\physical-ai-robotics-textbook\docusaurus\docs\00-introduction\05-syllabus.md"

echo.
echo ====================================
echo DONE! All files processed
echo ====================================

pause