@echo off
rem ---------------------------------------------------------------------------------------
rem sanitize.bat - run the full headless test suite under ASan + UBSan in native CLANG64.
rem
rem Normal Windows builds remain on UCRT64 through build.bat and mk.bat. This wrapper enters
rem the parallel MSYS2 CLANG64 environment solely for the sanitizer build and returns its
rem exit status. Install the opt-in dependencies with:
rem
rem   powershell -ExecutionPolicy Bypass -File tools\setup\setup_windows.ps1 -IncludeSanitizers
rem ---------------------------------------------------------------------------------------
setlocal enableextensions

cd /d "%~dp0"

if not defined MSYS2_ROOT set "MSYS2_ROOT=C:\msys64"

if not exist "%MSYS2_ROOT%\clang64\bin\clang.exe" (
    echo sanitize.bat: MSYS2 CLANG64 sanitizer toolchain not found at "%MSYS2_ROOT%". 1>&2
    echo Install with: powershell -ExecutionPolicy Bypass -File tools\setup\setup_windows.ps1 -IncludeSanitizers 1>&2
    exit /b 127
)

set "CIRCUIT_GIT_COMMIT="
set "CIRCUIT_GIT_BRANCH="
set "CIRCUIT_GIT_DIRTY="
where git >nul 2>&1
if not errorlevel 1 (
    for /f "delims=" %%i in ('git rev-parse --short^=12 HEAD 2^>nul') do set "CIRCUIT_GIT_COMMIT=%%i"
    for /f "delims=" %%i in ('git rev-parse --abbrev-ref HEAD 2^>nul') do set "CIRCUIT_GIT_BRANCH=%%i"
    git diff --quiet HEAD >nul 2>&1
    if errorlevel 1 (set "CIRCUIT_GIT_DIRTY=dirty") else (set "CIRCUIT_GIT_DIRTY=clean")
)

set "MSYS_REPO_FILE=%TEMP%\circuit_clang64_repo_%RANDOM%.txt"
"%MSYS2_ROOT%\usr\bin\cygpath.exe" -u "%CD%" > "%MSYS_REPO_FILE%"
if errorlevel 1 (
    echo sanitize.bat: cygpath failed for "%CD%". 1>&2
    del /q "%MSYS_REPO_FILE%" 2>nul
    exit /b 127
)
set /p MSYS_REPO=<"%MSYS_REPO_FILE%"
del /q "%MSYS_REPO_FILE%" 2>nul

"%MSYS2_ROOT%\usr\bin\env.exe" MSYSTEM=CLANG64 CHERE_INVOKING=1 MSYS2_PATH_TYPE=inherit ^
    "%MSYS2_ROOT%\usr\bin\bash.exe" --login -c "cd '%MSYS_REPO%' && make CIRCUIT_STRICT=1 sanitize %*"
set "STATUS=%ERRORLEVEL%"
exit /b %STATUS%
