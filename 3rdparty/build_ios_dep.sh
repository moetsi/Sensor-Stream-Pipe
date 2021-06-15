#!/bin/bash

# Compile iOS dependencies needed to compile Sensor-Stream-Pipe on iOS
# This script should be run under Terminal
# Previously, you should install :
# - Xcode 12.4+
# - CMake 3.20+
#

function install_yasm {
    echo "Install yasm"
    curl -L -O \
        http://www.tortall.net/projects/yasm/releases/yasm-1.3.0.tar.gz
    tar zxf yasm-1.3.0.tar.gz
    pushd yasm-1.3.0
    ./configure --prefix=${LOCAL_DIR}/yasm
    make -j12
    make install
    popd
}

function install_gas_preprocessor {
    echo "Install gas-preprocessor"
    curl -L -o gas-preprocessor.pl \
  "https://git.libav.org/?p=gas-preprocessor.git;a=blob_plain;f=gas-preprocessor.pl;hb=HEAD"
    chmod +x gas-preprocessor.pl
    mkdir -p ${LOCAL_DIR}/bin
    mv gas-preprocessor.pl ${LOCAL_DIR}/bin
}

# Disable securetransport to be compatible with Apple AppStore
function build_ffmpeg {

    install_yasm
    install_gas_preprocessor

    echo "Building ffmpeg"
    git clone https://github.com/kewlbear/FFmpeg-iOS-build-script.git
    pushd FFmpeg-iOS-build-script
    git checkout 167e59bdd8d35ed
    
    # Use our version of ffmpeg-iOS-build
    patch -p1 < $SOURCE_DIR/ffmpeg-iOS-build.patch
    
    export PATH=${LOCAL_DIR}/bin:${LOCAL_DIR}/yasm/bin:$PATH
    IPHONEOS_DEPLOYMENT_TARGET="9.0" ./build-ffmpeg-iOS-framework.sh
    mv FFmpeg.framework ${LOCAL_DIR}/lib
    popd
}

# Build minimal OpenCV : core imgproc
function build_opencv {
    echo "Building opencv"
    git clone --depth 1 --branch 3.4.13 \
        https://github.com/opencv/opencv.git
    mkdir build_opencv
    pushd build_opencv
    python ../opencv/platforms/ios/build_framework.py \
        --iphoneos_archs arm64 \
        --iphonesimulator_archs x86_64 \
        --iphoneos_deployment_target "9.0" \
        --without apps \
        --without calib3d \
        --without dnn \
        --without features2d \
        --without flann \
        --without java_bindings_generator \
        --without js \
        --without js_bindings_generator \
        --without ml \
        --without objdetect \
        --without photo \
        --without python2 \
        --without python_bindings_generator \
        --without python_tests \
        --without shape \
        --without stitching \
        --without superres \
        --without ts \
        --without video \
        --without videoio \
        --without videostab \
        --without world \
        --disable JAVA \
        --disable QUIRC \
        --disable EIGEN \
        --disable FFMPEG \
        --disable PROTOBUF \
        ios
    mv ios/opencv2.framework ${LOCAL_DIR}/lib
    popd
}

function build_cereal {
    echo "Building Cereal"
    git clone --depth 1 --branch v1.3.0 \
        https://github.com/USCiLab/cereal.git
    pushd cereal
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cereal \
        -DJUST_INSTALL_CEREAL=ON \
        ..
    cmake --build . -j 12 --config Release --target install
    cd ..
    popd
}

# https://github.com/gabime/spdlog
function build_spdlog {
    echo "Building spdlog"
    git clone --depth 1 --branch v1.8.2 \
        https://github.com/gabime/spdlog.git
    pushd spdlog
    mkdir build && cd build
    cmake -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE=${SOURCE_DIR}/ios.toolchain.cmake \
        -DPLATFORM=OS64COMBINED -DENABLE_BITCODE=ON \
        -DDEPLOYMENT_TARGET="9.0" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/spdlog \
        -DSPDLOG_BUILD_SHARED=OFF \
        -DSPDLOG_BUILD_EXAMPLE=OFF \
        -DSPDLOG_INSTALL=ON \
        ..
    cmake --build . --config Release
    cmake --install . --config Release
    cd ..
    popd
}

# https://github.com/catid/Zdepth
function build_zdepth {
    echo "Building zdepth"
    git clone https://github.com/catid/Zdepth.git
    pushd Zdepth
    # Commit including our cmake patch
    git checkout 9b333d9aec520

    # Use our version of zdepth
    patch -p1 < $SOURCE_DIR/zdepth.patch

    mkdir build && cd build
    cmake -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE=${SOURCE_DIR}/ios.toolchain.cmake \
        -DPLATFORM=OS64COMBINED -DENABLE_BITCODE=ON \
        -DDEPLOYMENT_TARGET="9.0" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/zdepth \
        ..
    cmake --build . --config Release
    cmake --install . --config Release
    cd ..
    popd
}

# https://github.com/jbeder/yaml-cpp/
function build_yaml_cpp {
    echo "Building yaml cpp"
    git clone --depth 1 --branch yaml-cpp-0.6.3 \
        https://github.com/jbeder/yaml-cpp.git
    pushd yaml-cpp
    mkdir build && cd build
    cmake -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE=${SOURCE_DIR}/ios.toolchain.cmake \
        -DPLATFORM=OS64COMBINED -DENABLE_BITCODE=ON \
        -DDEPLOYMENT_TARGET="9.0" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/yaml-cpp \
        -DYAML_APPLE_UNIVERSAL_BIN=OFF \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DYAML_CPP_BUILD_CONTRIB=OFF \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_CPP_INSTALL=ON \
        ..
    cmake --build . --config Release
    cmake --install . --config Release
    cd ..
    popd
}

# https://github.com/zeromq/libzmq
function build_libzmq {
    echo "Building libzmq"
    git clone --depth 1 --branch v4.3.4 \
        https://github.com/zeromq/libzmq.git
    pushd libzmq
    mkdir build && cd build
    cmake -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE=${SOURCE_DIR}/ios.toolchain.cmake \
        -DPLATFORM=OS64COMBINED -DENABLE_BITCODE=ON \
        -DDEPLOYMENT_TARGET="9.0" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/libzmq \
        -DBUILD_SHARED=OFF -DBUILD_STATIC=ON \
        -DBUILD_TESTS=OFF -DWITH_TLS=OFF \
        -DWITH_LIBSODIUM=OFF \
        -DWITH_LIBSODIUM_STATIC=OFF \
        ..
    cmake --build . --config Release
    cmake --install . --config Release
    cd ..
    popd
}

# https://github.com/zeromq/cppzmq/
function build_cppzmq {
    echo "Building cppzmq"
    git clone --depth 1 --branch v4.7.1 \
        https://github.com/zeromq/cppzmq.git
    pushd cppzmq
    mkdir build && cd build
    cmake -G Xcode \
        -DCMAKE_TOOLCHAIN_FILE=${SOURCE_DIR}/ios.toolchain.cmake \
        -DPLATFORM=OS64COMBINED -DENABLE_BITCODE=ON \
        -DDEPLOYMENT_TARGET="9.0" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cppzmq \
        -DZeroMQ_DIR=${LOCAL_DIR}/libzmq/lib/cmake/ZeroMQ \
        -DCPPZMQ_BUILD_TESTS=OFF \
        ..
    cmake --build . --config Release
    cmake --install . --config Release
    cd ..
    popd
}

export SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd | sed -e 's,^/c/,c:/,')"

echo "Cleaning tmp directory"
rm -rf tmp
if [ $? -ne 0 ]; then
    echo "Unable to remove tmp directory"
    exit
fi
mkdir tmp
pushd tmp

# Where we install all our dependencies
export LOCAL_DIR=`pwd`/local.ssp
mkdir -p ${LOCAL_DIR}
mkdir -p ${LOCAL_DIR}/lib

build_ffmpeg
build_opencv
build_cereal
build_spdlog
build_zdepth
build_yaml_cpp
build_libzmq
build_cppzmq

version=$(git describe --dirty | sed -e 's/^v//' -e 's/g//' -e 's/[[:space:]]//g')
prefix=`date +%Y%m%d%H%M`
filename=${prefix}_${version}_ssp_iosdep

echo "Packing ${LOCAL_DIR} to ${filename}.tar"
tar -C ${LOCAL_DIR} -cf ${filename}.tar \
  cereal \
  cppzmq \
  lib \
  libzmq \
  spdlog \
  yaml-cpp \
  zdepth

echo "Compressing ${filename}.tar"
gzip ${filename}.tar
