#!/bin/bash
CMDLINE_SSP="$1 $2 $3 $4 $5 $6 $7 $8 $9"
source path-helpers-ubuntu20.sh
cd build_release/bin
./ssp_client_opencv $CMDLINE_SSP
