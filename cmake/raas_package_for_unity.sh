#!/bin/bash

CONFIG=${1:-Release}

echo "Using ${CONFIG}"

rm -rf raas_plugin_unity
mkdir raas_plugin_unity
# Copy opencv2 and ffmpeg frameworks without the headers
cp -R _deps/ssp_dep-src/lib/* raas_plugin_unity
rm -rf raas_plugin_unity/opencv2.framework/Versions/A/Headers
rm -rf raas_plugin_unity/opencv2.framework/Headers
rm -rf raas_plugin_unity/FFmpeg.framework/Headers
find _deps -name "*.a" -print0 | xargs -0 -I % cp % raas_plugin_unity
cp lib/${CONFIG}/*.a raas_plugin_unity
