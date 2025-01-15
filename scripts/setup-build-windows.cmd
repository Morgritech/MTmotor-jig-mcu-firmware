@echo off

rem User prompt.
echo:
echo ...Please run in Command Prompt (CMD)... 
echo ...Ignore if already running in CMD...
echo:

rem rem Check admin privileges.
rem net session >nul 2>&1
rem if NOT %errorLevel% == 0 (
rem     echo "Please run as admin."
rem     exit /b 1 rem Exit only the script (no the terminal); error code 1.
rem )

rem Command line arguments
set SCRIPT_NAME=%~nx0
set CMD_CLI=-cli
set ARG_PATH=--path
set CMD_DEPS=-deps
set CMD_BUILD=-build
set CMD_HELP=-help

rem Specify the directory where arduino-cli is/will be installed.
rem C:\Program Files\Arduino CLI
set ARDUINO_CLI_DIR="%ProgramFiles%\Arduino CLI"
set ARDUINO_CLI_DIR_FOR_PATH=%ProgramFiles%\Arduino CLI

if "%~1" == "%CMD_CLI%" GOTO cli
if "%~1" == "%CMD_DEPS%" GOTO deps
if "%~1" == "%CMD_BUILD%" GOTO build
if "%~1" == "%CMD_HELP%" GOTO help
GOTO help

:cli

echo ...Installing Arduino CLI...
echo:

winget install --accept-package-agreements -e ArduinoSA.CLI
echo:

if "%~2" == "%ARG_PATH%" (
  rem If not already added, add arduino CLI directory to windows environment path (current session) so it can be executed directly.
  echo ...Adding Arduino CLI to the Windows environment path...
  echo:
  set "PATH=%PATH%;%ARDUINO_CLI_DIR_FOR_PATH%;"
)

%ARDUINO_CLI_DIR%\arduino-cli version
echo:

GOTO end

:deps

echo ...Installing and setting up project dependencies...
echo:

echo ...Installing arduino platforms/cores...
echo:

rem Specify the directory to the file listing the required arduino cores.
rem The name of the cores will be extracted from the name of the boards.
set CORES=arduino-boards.txt

%ARDUINO_CLI_DIR%\arduino-cli core update-index

rem Read one line at a time. 
rem Split text which are separated by colon (:) into 1st (%%C) and 2nd (%%D) argument.
setlocal enabledelayedexpansion
for /f "usebackq tokens=1-2 delims=:" %%C in ("%CORES%") do (
  set CORE=%%C:%%D
  %ARDUINO_CLI_DIR%\arduino-cli core install !CORE!
)
endlocal
echo:

echo ...Installing arduino libraries...
echo:

rem Specify the directory to file listing the required arduino libraries.
set LIBS=arduino-libs.txt

%ARDUINO_CLI_DIR%\arduino-cli lib update-index

for /f "usebackq delims=" %%L in ("%LIBS%") do (
  %ARDUINO_CLI_DIR%\arduino-cli lib install %%L
)
echo:

GOTO end

:build

echo ...Compiling the project...
echo:

rem Specify the sketch directory.
set SKETCH_DIR=src
rem Specify the build directory.
set BUILD_DIR=build
rem Specify the directories to files listing the required arduino boards.
set BOARDS=arduino-boards.txt
rem Build for multiple boards.
rem Read one line at a time (No spaces or tabs as delimiters).
setlocal enabledelayedexpansion
for /f "usebackq delims=" %%B in ("%BOARDS%") do (
  rem Replace ":" with "-" to create the build sub-folder with the board name.
  set TEMP_DIR=%%B
  set BUILD_SUBDIR=!TEMP_DIR::=-!
  rem Split command line args (%*) which are separated by space (" ") into 1st argument (%%A) and 2nd onwards (%%B).
  for /f "tokens=1* delims= " %%A in ("%*") do (
      rem Ignore first command line argument.
      set BUILD_ARGS=%%B
  )
  rem Compile clean (no cache) and use command line args (2nd onwards) if passed to the script.
  %ARDUINO_CLI_DIR%\arduino-cli compile -b %%B %SKETCH_DIR% --clean --build-path %BUILD_DIR%\!BUILD_SUBDIR! !BUILD_ARGS!
)
endlocal
echo:

GOTO end

:help

echo ...Help...
echo:
echo Available commands are:
echo %CMD_CLI%    Install arduino-cli. Add optional flag %ARG_PATH% to add arduino-cli to the Windows environment path. 
echo %CMD_DEPS%   Install project dependencies (Arduino cores and libraries).
echo %CMD_BUILD%  Compile the project. Add optional arduino-cli flags if desired (e.g., to upload the programme to the board).
echo %CMD_HELP%   Show available commands.
echo Usage:
echo %SCRIPT_NAME% ^<command^>
echo %SCRIPT_NAME% %CMD_CLI% [optional flag]
echo %SCRIPT_NAME% %CMD_BUILD% [optional arduino-cli compile flags]
echo Examples:
echo %SCRIPT_NAME% %CMD_CLI% %ARG_PATH%
echo %SCRIPT_NAME% %CMD_BUILD% --port COM3 --upload
echo:

:end
echo ...End...
echo: