@echo off
rem ---------------------------------------------------------------------------------------
rem build.bat - the hot-reload development build, for cmd.exe.
rem
rem Same contract as build.sh: rebuilds the game module every time, rebuilds drifty.exe only
rem when it is not already running, always terminates immediately, and returns the
rem compiler's exit status. It never launches drifty.exe.
rem
rem A failed compile leaves the previous working module untouched: the link output goes to a
rem temporary name and is only moved into place on success.
rem
rem   build.bat            build the module (and the exe if it is not running)
rem   build.bat release    single executable, no hot reload
rem   build.bat tests      headless test executable
rem   build.bat clean      remove generated artifacts
rem ---------------------------------------------------------------------------------------
setlocal enableextensions

cd /d "%~dp0"

rem ------------------------------------------------------------------------------ toolchain
if "%CC%"=="" set "CC=gcc"
where %CC% >nul 2>&1
if errorlevel 1 (
    for %%D in (
        "C:\ProgramData\chocolatey\lib\mingw\tools\install\mingw64\bin"
        "C:\msys64\ucrt64\bin"
        "C:\msys64\mingw64\bin"
        "C:\mingw64\bin"
    ) do (
        if exist "%%~D\gcc.exe" (
            set "PATH=%%~D;%PATH%"
            goto :toolchain_done
        )
    )
)
:toolchain_done
where %CC% >nul 2>&1
if errorlevel 1 (
    echo build.bat: no C compiler found ^(tried "%CC%"^). See README.md for prerequisites. 1>&2
    exit /b 127
)

rem --------------------------------------------------------------------------------- raylib
if "%RAYLIB_DIR%"=="" set "RAYLIB_DIR=vendor\raylib"
set "RAYLIB_CFLAGS=-I%RAYLIB_DIR%/include"
set "RAYLIB_LIBS=-L%RAYLIB_DIR%/lib -lraylib -lopengl32 -lgdi32 -lwinmm"
set "RAYLIB_RUNTIME=%RAYLIB_DIR%\bin\raylib.dll"

where pkg-config >nul 2>&1
if not errorlevel 1 (
    pkg-config --exists raylib
    if not errorlevel 1 (
        for /f "usebackq delims=" %%F in (`pkg-config --cflags raylib`) do set "RAYLIB_CFLAGS=%%F"
        for /f "usebackq delims=" %%F in (`pkg-config --libs raylib`)   do set "RAYLIB_LIBS=%%F"
        set "RAYLIB_RUNTIME="
    )
)

rem ---------------------------------------------------------------------------------- flags
set "CSTD=-std=c11 -Isrc"
set "WARNINGS=-Wall -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith"
set "DEBUG_FLAGS=-O0 -g"
set "RELEASE_FLAGS=-O2 -DNDEBUG"

set "SHARED_SRCS=src/input.c src/math_utils.c"
set "GAME_SRCS=src/game.c src/replay.c src/telemetry.c"
set "PLATFORM_SRCS=src/main.c src/timestep.c"
set "HOTRELOAD_SRC=src/hotreload_windows.c"
set "TEST_SRCS=tests/physics_tests.c src/timestep.c %GAME_SRCS% %SHARED_SRCS%"

set "MODULE=build\game.dll"
set "EXE=drifty.exe"
set "EXE_RELEASE=drifty_release.exe"
set "EXE_TESTS=drifty_tests.exe"

if not exist build mkdir build
if not exist telemetry mkdir telemetry

rem --------------------------------------------------------------------------------- action
set "MODE=dev"
if not "%~1"=="" set "MODE=%~1"

if /i "%MODE%"=="clean" goto :do_clean
if /i "%MODE%"=="tests" goto :do_tests
if /i "%MODE%"=="release" goto :do_release
if /i "%MODE%"=="dev" goto :do_dev
echo build.bat: unrecognised argument "%MODE%" 1>&2
exit /b 2

:do_clean
if exist build rmdir /s /q build
if exist "%EXE%" del /q "%EXE%"
if exist "%EXE_RELEASE%" del /q "%EXE_RELEASE%"
if exist "%EXE_TESTS%" del /q "%EXE_TESTS%"
if exist raylib.dll del /q raylib.dll
echo cleaned.
exit /b 0

:do_tests
%CC% %CSTD% %WARNINGS% %RELEASE_FLAGS% -DDRIFTY_HEADLESS %TEST_SRCS% -o "%EXE_TESTS%" %RAYLIB_CFLAGS% -lm
if errorlevel 1 exit /b %errorlevel%
echo Built %EXE_TESTS%.
exit /b 0

:do_release
if not "%RAYLIB_RUNTIME%"=="" if exist "%RAYLIB_RUNTIME%" copy /y "%RAYLIB_RUNTIME%" . >nul
%CC% %CSTD% %WARNINGS% %RELEASE_FLAGS% %PLATFORM_SRCS% %GAME_SRCS% %SHARED_SRCS% -o "%EXE_RELEASE%" %RAYLIB_CFLAGS% %RAYLIB_LIBS%
if errorlevel 1 exit /b %errorlevel%
echo Built %EXE_RELEASE% ^(no hot reload, no game module^).
exit /b 0

:do_dev
if not "%RAYLIB_RUNTIME%"=="" if exist "%RAYLIB_RUNTIME%" copy /y "%RAYLIB_RUNTIME%" . >nul

rem Link to a temporary name first: the linker leaves a zero-length file in place while it
rem works, and a running game polling for changes would load that. The move is atomic.
%CC% %CSTD% %WARNINGS% %DEBUG_FLAGS% -shared -fPIC -DDRIFTY_HOT_RELOAD -DDRIFTY_GAME_MODULE %GAME_SRCS% %SHARED_SRCS% -o build\game_tmp.tmp %RAYLIB_CFLAGS% %RAYLIB_LIBS%
if errorlevel 1 (
    if exist build\game_tmp.tmp del /q build\game_tmp.tmp
    echo build.bat: game module failed to compile; %MODULE% left untouched. 1>&2
    exit /b 1
)
move /y build\game_tmp.tmp "%MODULE%" >nul
if errorlevel 1 exit /b 1

rem If the game is already running, the module swap is all that is needed.
tasklist /fi "IMAGENAME eq drifty.exe" 2>nul | find /i "drifty.exe" >nul
if not errorlevel 1 (
    echo Hot reloading %MODULE%...
    exit /b 0
)

%CC% %CSTD% %WARNINGS% %DEBUG_FLAGS% -DDRIFTY_HOT_RELOAD %PLATFORM_SRCS% %SHARED_SRCS% %HOTRELOAD_SRC% -o "%EXE%" %RAYLIB_CFLAGS% %RAYLIB_LIBS%
if errorlevel 1 (
    echo build.bat: platform layer failed to compile. 1>&2
    exit /b 1
)

echo Built %MODULE% and %EXE% - run %EXE% and leave it running.
exit /b 0
