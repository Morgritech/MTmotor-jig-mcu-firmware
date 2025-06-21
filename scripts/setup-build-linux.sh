#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e 

echo

# Check sudo
#if [ "$EUID" -ne 0 ]
#  then echo "Please run as root."
#  exit
#fi

# Command line arguments
SCRIPT_NAME=$(basename "$0")
CMD_CLI="-cli"
ARG_PATH="--path"
CMD_DEPS="-deps"
CMD_BUILD="-build"
CMD_HELP="-help"

# Specify the directory where arduino-cli is/will be installed.
# home/<username>/bin
ARDUINO_CLI_DIR=~/bin

if [ "$1" == "$CMD_CLI" ] ; then
  echo ...Installing Arduino CLI...
  echo

  pushd $PWD # Save current working directory.
  cd ~ # Ensure we are in the "home/<username>" directory.
  sudo apt-get --no-install-recommends install -y curl
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
  echo
  popd > /dev/null # Restore the working directory + suppress the output of popd.

  if [ "$2" == "$ARG_PATH" ] ; then
    echo ...Adding arduino-cli to the Linux user environment path...
    echo
    PATH=$PATH:$ARDUINO_CLI_DIR
  fi

  #$ARDUINO_CLI_DIR/arduino-cli version
  #echo
fi

if [ "$1" == "$CMD_DEPS" ] ; then
  echo ...Installing and setting up project dependencies...
  echo

  echo ...Installing arduino platforms/cores...
  echo

  # Specify the directories to the file listing the required arduino cores.
  # The name of the cores will be extracted from the name of the boards.  
  CORES=arduino-boards.txt

  $ARDUINO_CLI_DIR/arduino-cli core update-index

  # Read one line at a time.
  for CORE in $(cat $CORES) ; do
    # Split text which are separated by colon (:) into 1st (first), 2nd (second) and 3rd (third) argument.
    # Carriage return '\r' must also be removed in-case file was created in Windows.
    IFS=':' read -r first second third <<< ${CORE%$'\r'}
    CORE="$first:$second"
    $ARDUINO_CLI_DIR/arduino-cli core install $CORE
  done
  unset IFS
  echo

  echo ...Installing arduino libraries...
  echo

  # Specify the directory to the file listing the required arduino libraries.
  LIBS=arduino-libs.txt

  $ARDUINO_CLI_DIR/arduino-cli lib update-index

  for LIB in $(cat $LIBS) ; do
    $ARDUINO_CLI_DIR/arduino-cli lib install ${LIB%$'\r'}
  done
  echo
fi

if [ "$1" == "$CMD_BUILD" ] ; then

  echo ...Compiling the project...
  echo

  # Specify the sketch directory.
  SKETCH_DIR=src #./src
  # Specify the build directory.
  BUILD_DIR=build
  # Specify the directories to files listing the required arduino boards.
  BOARDS=arduino-boards.txt
  # Build for multiple boards.
  # Read one line at a time (No spaces or tabs as delimiters).
  for BOARD in $(cat "$BOARDS") ; do
      # Replace ":" with "-" to create the build sub-folder with the board name.
      BUILD_SUBDIR="${BOARD//:/-}"
      # Compile clean (no cache) and use command line args (2nd onwards) if passed to the script.
      # Carriage return '\r' must be removed in-case file was created in Windows.
      $ARDUINO_CLI_DIR/arduino-cli compile -b ${BOARD%$'\r'} $SKETCH_DIR --clean --build-path $BUILD_DIR/$BUILD_SUBDIR ${@:2}
  done
  echo
fi

if [ "$1" == "$CMD_HELP" ] || ([ "$1" != "$CMD_CLI" ] && [ "$1" != "$CMD_DEPS" ] && [ "$1" != "$CMD_BUILD" ]) ; then
  echo \
  "...Help...

  Available commands are:
  $CMD_CLI    Install arduino-cli. Optional flag $ARG_PATH to add arduino-cli to the Linux environment path. 
  $CMD_DEPS   Install project dependencies (Arduino cores and libraries).
  $CMD_BUILD  Compile the project. Add optional arduino-cli flags if desired (e.g., to upload the programme to the board).
  $CMD_HELP   Show available commands.
  Usage:
  $SCRIPT_NAME <command>
  $SCRIPT_NAME $CMD_CLI [optional flag]
  $SCRIPT_NAME $CMD_BUILD [optional arduino-cli compile flags]
  EXAMPLES
  $SCRIPT_NAME $CMD_CLI $ARG_PATH
  $SCRIPT_NAME $CMD_BUILD --port /dev/ttyACM0 --upload
  "
fi

echo ...End...
echo