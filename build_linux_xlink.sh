#!/bin/bash

source ./path-helpers-ubuntu20.sh

rm -Rvf build_release
cd 3rdparty
./build_lin_dep_xlink.sh || exit -1
cd ..

mkdir build_release
cd build_release
cmake -DCMAKE_BUILD_TYPE=Release -DXLINK_ENABLED=ON -DLOCAL_DEP_BUILD_LINUX=ON .. || exit -1
make -j24 || exit -1
cd ..
mkdir build_debug
cd build_debug
cmake -DCMAKE_BUILD_TYPE=Debug -DXLINK_ENABLED=ON -DLOCAL_DEP_BUILD_LINUX=ON .. || exit -1
make -j24 || exit -1
