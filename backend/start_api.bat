@echo off
echo Starting RAG Agent API Server...
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo Python is not installed or not in PATH
    pause
    exit /b 1
)

REM Check if required packages are installed
echo Installing required packages...
pip install -r backend/requirements.txt

REM Start the API server
echo.
echo Starting FastAPI server on http://localhost:8000
echo Press Ctrl+C to stop the server
echo.
python run_api.py