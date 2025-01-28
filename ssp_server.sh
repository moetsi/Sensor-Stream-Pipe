#!/bin/bash
source path-helpers-ubuntu20.sh
cd build_release/bin
./ssp_server "$@"
