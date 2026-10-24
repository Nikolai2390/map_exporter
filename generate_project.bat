@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem This folder, with no trailing slash. The build directory is created here.
set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"

set "EXITCODE=0"

set "CMAKE="
for /f "delims=" %%C in ('where cmake 2^>nul') do (
    if not defined CMAKE set "CMAKE=%%C"
)
if not defined CMAKE if exist "%ProgramFiles%\CMake\bin\cmake.exe" set "CMAKE=%ProgramFiles%\CMake\bin\cmake.exe"
if not defined CMAKE (
    echo cmake was not found. Install CMake and run this again.
    set "EXITCODE=1"
    goto :finish
)

if not exist "%ROOT%\CMakeLists.txt" (
    echo CMakeLists.txt was not found next to this script.
    set "EXITCODE=1"
    goto :finish
)

rem A copied build folder still points at the directory where it was generated.
if exist "%ROOT%\build\CMakeCache.txt" (
    set "CACHED="
    for /f "usebackq tokens=1,* delims==" %%A in ("%ROOT%\build\CMakeCache.txt") do (
        if /i "%%A"=="CMAKE_HOME_DIRECTORY:INTERNAL" set "CACHED=%%B"
    )
    if defined CACHED (
        set "CACHED=!CACHED:/=\!"
        if /i not "!CACHED!"=="%ROOT%" (
            echo Removing build files generated for !CACHED!
            rmdir /s /q "%ROOT%\build"
        )
    )
)

"%CMAKE%" -S "%ROOT%" -B "%ROOT%\build" -G "Visual Studio 17 2022" -A x64
if errorlevel 1 (
    echo.
    echo Failed to generate the Visual Studio project.
    set "EXITCODE=1"
    goto :finish
)

echo.
echo Generated "%ROOT%\build\map_exporter.sln"

:finish
echo %cmdcmdline% | findstr /i /c:"/c " >nul && pause
exit /b %EXITCODE%
