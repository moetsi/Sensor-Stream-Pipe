#!/bin/bash

CONFIG=${1:-Release}

echo "Using ${CONFIG}"

rm -rf ssp_plugin_unity
mkdir ssp_plugin_unity
cp -R _deps/ssp_dep-src/lib/* ssp_plugin_unity
cp lib/${CONFIG}/*.a ssp_plugin_unity
