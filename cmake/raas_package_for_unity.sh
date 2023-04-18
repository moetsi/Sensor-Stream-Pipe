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

# Find and copy all libraries from the 'build_release/lib' directory
find "${CMAKE_BINARY_DIR}/lib" -type f -iname "*.${SHARED_LIB_EXT}" \
  -exec cp '{}' "$RAAS_UNITY_PLUGIN_DIR" ';'
