#!/bin/bash

CONFIG=${1:-Release}

echo "Using ${CONFIG}"

rm -rf ssp_plugin_unity
mkdir ssp_plugin_unity
# Copy opencv2 and ffmpeg frameworks without the headers
cp -R _deps/ssp_dep-src/lib/* ssp_plugin_unity
rm -rf ssp_plugin_unity/opencv2.framework/Versions/A/Headers
rm -rf ssp_plugin_unity/opencv2.framework/Headers
rm -rf ssp_plugin_unity/FFmpeg.framework/Headers
find _deps -name "*.a" -print0 | xargs -0 -I % cp % ssp_plugin_unity
cp lib/${CONFIG}/*.a ssp_plugin_unity
