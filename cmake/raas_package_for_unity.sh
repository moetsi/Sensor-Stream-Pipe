#!/bin/bash

# Set the CMAKE_BINARY_DIR and CMAKE_SOURCE_DIR variables from the command line arguments
CMAKE_BINARY_DIR="$1"
CMAKE_SOURCE_DIR="$2"

# Delete the 'raas_unity_plugin' directory if it exists and create a new one
RAAS_UNITY_PLUGIN_DIR="${CMAKE_BINARY_DIR}/raas_unity_plugin"
if [ -d "$RAAS_UNITY_PLUGIN_DIR" ]; then
  rm -rf "$RAAS_UNITY_PLUGIN_DIR"
fi
mkdir -p "$RAAS_UNITY_PLUGIN_DIR"

# Determine the platform and shared library extension
case "$(uname -s)" in
    Linux*)     SHARED_LIB_EXT="so";;
    Darwin*)    SHARED_LIB_EXT="dylib";;
    CYGWIN*|MINGW*|MSYS*|*_NT*) SHARED_LIB_EXT="dll";;
    *)          echo "unknown platform"; exit 1;;
esac

# Recursively find and copy all libraries from the '3rdparty/' directory
# into the 'raas_unity_plugin' directory
find "${CMAKE_SOURCE_DIR}/3rdparty/" -type f \( -iname "*.so" -o -iname "*.a" -o -iname "*.dylib" -o -iname "*.dll" \) \
  -exec cp '{}' "$RAAS_UNITY_PLUGIN_DIR" ';'

# Determine the lib directory based on the platform
case "$(uname -s)" in
    CYGWIN*|MINGW*|MSYS*|*_NT*) LIB_DIR="${CMAKE_BINARY_DIR}/lib/Release";;
    *) LIB_DIR="${CMAKE_BINARY_DIR}/lib";;
esac

# Find and copy all libraries from the 'build_release/lib' directory
find "$LIB_DIR" -type f -iname "*.${SHARED_LIB_EXT}" -exec cp '{}' "$RAAS_UNITY_PLUGIN_DIR" ';'

# Determine the bin directory based on the platform
case "$(uname -s)" in
    CYGWIN*|MINGW*|MSYS*|*_NT*) BIN_DIR="${CMAKE_BINARY_DIR}/bin/Release";;
    *) BIN_DIR="${CMAKE_BINARY_DIR}/bin";;
esac

# Find and copy 'raas_plugin' from the 'bin' directory
find "$BIN_DIR" -type f -iname "raas_plugin*" -exec cp '{}' "$RAAS_UNITY_PLUGIN_DIR" ';'