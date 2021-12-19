#!/bin/bash

# Compile Linux dependencies needed to compile Sensor-Stream-Pipe on Ubuntu 18.04
# This script should be run under Terminal
# Previously, you should install :
# - dev tools
# - cmake 3.11+
# sudo apt-get update
# sudo apt-get install build-essential
# sudo apt-get install nasm
# sudo apt-get install libgtk2.0-dev
# sudo snap install --classic cmake

# For k4a
# - sudo apt-get install xorg-dev
# - sudo apt-get install uuid-dev
# - sudo apt-get install libudev-dev libusb-1.0-0-dev

# For zmq+ssl
# - sudo apt-get install libsodium-dev libsodium23

function install_nasm {
    echo "Verify nasm"
    nasm --version
    if [ $? -ne 0 ]; then
       echo "Unable to find nasm; do a sudo apt-get install nasm"
       exit
    fi
}

function build_openh264 {
    # https://github.com/AkillesAILimited/openh264
    echo "building libopenh264"
    git clone https://github.com/AkillesAILimited/openh264 || exit -1
    pushd openh264
    cp -fv ../../Makefile.libopenh264 Makefile
    ( export PREFIX=${LOCAL_DIR}/openh264; make -j16 OS=linux || exit -1)
    mkdir ${LOCAL_DIR}/openh264
    ( export PREFIX=${LOCAL_DIR}/openh264; make install || exit -1 )
    popd
}

# https://kochuns.blogspot.com/2018/09/ffmpegffmpeg-build-with-openh264.html for openh264 build instructions
function build_ffmpeg {
    echo "Building ffmpeg"
    git clone --depth 1 --branch release/4.3 \
         https://git.ffmpeg.org/ffmpeg.git ffmpeg || exit -1
    pushd ffmpeg
    ( export PKG_CONFIG_PATH=${LOCAL_DIR}/openh264/lib/pkgconfig; ./configure --prefix=${LOCAL_DIR}/ffmpeg \
        --disable-gpl \
        --enable-asm \
        --enable-libopenh264 \
        --extra-cflags='-I${LOCAL_DIR}/openh264/include' \
        --extra-ldflags='-L${LOCAL_DIR}/openh264/lib' \
        --disable-static \
        --enable-shared \
        --enable-rpath \
        --disable-programs \
        --disable-ffmpeg \
        --disable-ffplay \
        --disable-ffprobe \
        --disable-securetransport || exit -1 )
    make -j16 || exit -1
    make install || exit -1
    popd
}

# --exclude-libs
# core gapi highgui imgcodecs imgproc
# Build minimal OpenCV : core imgproc imgcodecs highgui 
# opencv/include/opencv4/opencv2/imgcodecs.hpp
function build_opencv {
    echo "Building opencv"
    wget -O opencv-4.5.3.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.5.3.tar.gz
    tar -xf opencv-4.5.3.tar.gz
    pushd opencv-4.5.3
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/opencv \
        -DOPENCV_GENERATE_PKGCONFIG=YES \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_opencv_apps:BOOL=ON \
        -DBUILD_opencv_calib3d:BOOL=OFF \
        -DBUILD_opencv_core:BOOL=ON \
        -DBUILD_opencv_dnn:BOOL=OFF \
        -DBUILD_opencv_features2d:BOOL=OFF \
        -DBUILD_opencv_flann:BOOL=OFF \
        -DBUILD_opencv_highgui:BOOL=ON \
        -DBUILD_opencv_imgcodecs:BOOL=ON \
        -DBUILD_opencv_imgproc:BOOL=ON \
        -DBUILD_opencv_java_bindings_generator:BOOL=OFF \
        -DBUILD_opencv_js:BOOL=OFF \
        -DBUILD_opencv_js_bindings_generator:BOOL=OFF \
        -DBUILD_opencv_ml:BOOL=OFF \
        -DBUILD_opencv_objdetect:BOOL=OFF \
        -DBUILD_opencv_photo:BOOL=OFF \
        -DBUILD_opencv_python2:BOOL=OFF \
        -DBUILD_opencv_python_bindings_generator:BOOL=OFF \
        -DBUILD_opencv_python_tests:BOOL=OFF \
        -DBUILD_opencv_shape:BOOL=OFF \
        -DBUILD_opencv_stitching:BOOL=OFF \
        -DBUILD_opencv_superres:BOOL=OFF \
        -DBUILD_opencv_ts:BOOL=OFF \
        -DBUILD_opencv_video:BOOL=OFF \
        -DBUILD_opencv_videoio:BOOL=OFF \
        -DBUILD_opencv_videostab:BOOL=OFF \
        -DBUILD_opencv_world:BOOL=OFF \
        -DBUILD_JAVA=OFF \
        -DBUILD_PACKAGE=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_PROTOBUF=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_JPEG=ON -DBUILD_PNG=ON -DBUILD_ZLIB=ON \
        -DWITH_EIGEN=OFF -DWITH_FFMPEG=OFF \
        -DWITH_QUIRC=OFF \
        -DWITH_LAPACK=NO \
        -DENABLE_PIC=ON \
        ..
    cmake --build . -j 16 --config Release --target install
    cd ..
    popd
}

function build_cereal {
    echo "Building Cereal"
    git clone --depth 1 --branch v1.3.0 \
        https://github.com/USCiLab/cereal.git || exit -1
    pushd cereal
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cereal \
        -DJUST_INSTALL_CEREAL=ON \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/gabime/spdlog
function build_spdlog {
    echo "Building spdlog"
    git clone --depth 1 --branch v1.8.2 \
        https://github.com/gabime/spdlog.git || exit -1
    pushd spdlog
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/spdlog \
        -DSPDLOG_BUILD_SHARED=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/catid/Zdepth
function build_zdepth {
    echo "Building zdepth"
    git clone https://github.com/catid/Zdepth.git || exit -1
    pushd Zdepth
    # Commit including our cmake patch
    git checkout 9b333d9aec520

    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/zdepth \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/jbeder/yaml-cpp/
function build_yaml_cpp {
    echo "Building yaml cpp"
    git clone --depth 1 --branch yaml-cpp-0.6.3 \
        https://github.com/jbeder/yaml-cpp.git || exit -1
    pushd yaml-cpp
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/yaml-cpp \
        -DYAML_APPLE_UNIVERSAL_BIN=OFF \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DYAML_CPP_BUILD_CONTRIB=OFF \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_CPP_INSTALL=ON \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/zeromq/libzmq
function build_libzmq {
    echo "Building libzmq"
    git clone --depth 1 --branch v4.3.4 \
        https://github.com/zeromq/libzmq.git || exit -1
    pushd libzmq
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/libzmq \
        -DBUILD_SHARED=OFF -DBUILD_STATIC=ON \
        -DBUILD_TESTS=OFF -DWITH_TLS=ON \
        -DWITH_LIBSODIUM=ON \
        -DWITH_LIBSODIUM_STATIC=ON \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/zeromq/cppzmq/
function build_cppzmq {
    echo "Building cppzmq"
    git clone --depth 1 --branch v4.7.1 \
        https://github.com/zeromq/cppzmq.git || exit -1
    pushd cppzmq
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cppzmq \
        -DZeroMQ_DIR=${LOCAL_DIR}/libzmq/lib/cmake/ZeroMQ \
        -DCPPZMQ_BUILD_TESTS=OFF \
        .. || exit -1
    cmake --build . -j 16 --config Release --target install || exit -1
    cd ..
    popd
}

# https://github.com/microsoft/Azure-Kinect-Sensor-SDK
function build_k4a {
    echo "Building Azure Kinect Sensor SDK"
    git clone --depth 1 --branch v1.4.1 \
        https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
    pushd Azure-Kinect-Sensor-SDK

    # Use our version of spdlog
    patch -p1 < $SOURCE_DIR/k4a.patch

    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/k4a \
        -Dspdlog_DIR=${LOCAL_DIR}/spdlog/lib/cmake/spdlog \
        -DBUILD_TESTING=OFF \
        ..
    cmake --build . -j 16 --config Release --target install
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

install_nasm || exit -1
build_openh264 || exit -1
build_ffmpeg || exit -1

build_opencv || exit -1
build_cereal || exit -1
build_spdlog || exit -1
build_zdepth || exit -1
build_yaml_cpp || exit -1
build_libzmq || exit -1
build_cppzmq || exit -1
#build_k4a
#build_depthai

version=$(git describe --dirty | sed -e 's/^v//' -e 's/g//' -e 's/[[:space:]]//g')
prefix=`date +%Y%m%d%H%M`
filename=${prefix}_${version}_ssp_lindep

echo "Packing ${LOCAL_DIR} to ${filename}.tar"
tar -C ${LOCAL_DIR} -cf ${filename}.tar \
  cereal \
  cppzmq \
  ffmpeg \
  libzmq \
  opencv \
  spdlog \
  yaml-cpp \
  openh264 \
  zdepth

##   depthai-core 

echo "Compressing ${filename}.tar"
gzip ${filename}.tar