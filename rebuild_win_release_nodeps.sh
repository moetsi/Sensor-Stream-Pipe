#!/bin/bash

##source ./path-helpers-ubuntu20.sh
#rm -Rvf build_release
#rm -Rvf build_debug
#cd 3rdparty
#./build_win_dep_xlink.sh || exit -1
#cd ..

mkdir build_release
cd build_release
cmake  -G "Visual Studio 16 2019" -A x64 -DCMAKE_BUILD_TYPE=Release -DXLINK_ENABLED=OFF -DLOCAL_DEP_BUILD_WINDOWS=ON .. || exit -1
cmake --build . --config Release
cd ..
#mkdir build_debug
#cd build_debug
#cmake -DCMAKE_BUILD_TYPE=Debug -DXLINK_ENABLED=ON -DLOCAL_DEP_BUILD_WINDOWS=ON .. || exit -1
#make -j24 || exit -1
