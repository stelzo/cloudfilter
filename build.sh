#!/bin/bash
set -e

### A script to build rust binaries and copy them to a catkin package bin folder.
### The script is intended to be used in a catkin package with a Cargo.toml file. It installs rust if it is not installed.
### Usage: ./build.sh DEBUG|RELEASE [binary1] [binary2] ...
###
### @author: Christopher Sieh <stelzo@steado.de>
### @date: 2023-04-25
### @license: MIT

# check first parameter case insensitive(!) debug or release
if [[ ! "$1" =~ ^(DEBUG|RELEASE|debug|release|Debug|Release)$ ]]; then
  echo "Usage: ./build.sh DEBUG|RELEASE [binary1] [binary2] ..."
  exit
fi

# set binaries except first argument
binaries=("${@:2}")

# check if given argument for DEBUG or RELEASE build
if [[ "$1" =~ ^(DEBUG|debug|Debug)$ ]]; then
  CARGO_BUILD_TYPE="debug"
else
  CARGO_BUILD_TYPE="release"
fi

# current dir name as package name
BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
DIR="$(basename "$BASE_DIR")"

# set dir name as default if no binaries are given
if [ "${#binaries[@]}" -eq 0 ]; then
  binaries=("$DIR")
fi

catkin_pkg_bin_path="$(eval "catkin locate -d")/lib/$DIR"

mkdir -p "$catkin_pkg_bin_path"

# install rust using curl or wget. If both are not installed, curl is installed but needs sudo.
install_rust() {
  if [ -f "$BASE_DIR"/.rustup.lock ]; then
    echo "rustup is already installing, please wait..."
    while [ -f "$BASE_DIR"/.rustup.lock ]; do
      sleep 1
    done
    return
  fi

  # set lock file
  touch "$BASE_DIR"/.rustup.lock

  if ! command -v curl &> /dev/null
  then
    if ! command -v wget &> /dev/null
      then
        echo "curl and wget could not be found, please install curl or wget and try again."
        echo "sudo apt install curl"
        exit 1
      else
        wget https://sh.rustup.rs -O - | sh -s -- -y
      fi
  else
    curl https://sh.rustup.rs -sSf | sh -s -- -y
  fi

  source "$HOME"/.cargo/env

  # remove lock file
  rm "$BASE_DIR"/.rustup.lock
}

# check if rust is installed
if ! command -v rustc &> /dev/null
then
  echo "rustc could not be found, installing rust..."
  install_rust

  # check if rust is installed again
  if ! command -v rustc &> /dev/null
  then
    echo "rustc could not be found"
    exit
  fi
fi

# check if cargo is installed
if ! command -v cargo &> /dev/null
then
  echo "cargo could not be found"
  install_rust

  # check if cargo is installed again
  if ! command -v cargo &> /dev/null
  then
    echo "cargo could not be found"
    exit
  fi
fi

### BUG WORKAROUND
# For generation, a few messages have problems so we reset to all default messages. Delete this part if you need messages from your workspace.
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
export CMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO
export ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share
### END BUG WORKAROUND

cd "$BASE_DIR"

# if debug, omit the release flag
if [ "$CARGO_BUILD_TYPE" = "debug" ]; then
  cargo build --bins
else
  cargo build --release --bins
fi

# copy binaries to catkin package bin folder
for binary in "${binaries[@]}"
do
  cp "target/$CARGO_BUILD_TYPE/$binary" "$catkin_pkg_bin_path"
done
