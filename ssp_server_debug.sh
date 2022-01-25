
#!/bin/bash
CMDLINE_SSP="$1 $2 $3 $4 $5 $6 $7 $8 $9"
source path-helpers-ubuntu20.sh
cd build_debug/bin
./ssp_server $CMDLINE_SSP
