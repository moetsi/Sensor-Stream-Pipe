#!/bin/bash

# Compile macOS dependencies needed to compile Sensor-Stream-Pipe on macOS
# This script should be run under Terminal
# Previously, you should install :
# - Xcode
# - CMake
#

function install_nasm {
    echo "Install nasm"
    curl -L -O \
      https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/macosx/nasm-2.15.05-macosx.zip
    unzip nasm-2.15.05-macosx.zip
    export PATH=`pwd`/nasm-2.15.05:$PATH
    nasm --version
}

# Disable securetransport to be compatible with Apple AppStore
function build_ffmpeg {
    echo "Building ffmpeg"
    git clone --depth 1 --branch release/4.3 \
         https://git.ffmpeg.org/ffmpeg.git ffmpeg
    pushd ffmpeg
    ./configure --prefix=${LOCAL_DIR}/ffmpeg \
        --disable-securetransport
    make -j12
    make install
    popd
}

# Build minimal OpenCV : core imgproc
function build_opencv {
    echo "Building opencv"
    git clone --depth 1 --branch 3.4.13 \
        https://github.com/opencv/opencv.git
    pushd opencv
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/opencv \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_opencv_apps:BOOL=OFF \
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
        -DWITH_EIGEN=OFF -DWITH_FFMPEG=OFF \
        -DWITH_QUIRC=OFF \
        ..
    cmake --build . -j 12 --config Release --target install
    cd ..
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
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/spdlog \
        -DSPDLOG_BUILD_SHARED=OFF \
        ..
    cmake --build . -j 12 --config Release --target install
    cd ..
    popd
}

# https://github.com/catid/Zdepth
function build_zdepth {
    echo "Building zdepth"
    git clone --depth 1 https://github.com/catid/Zdepth.git
    pushd Zdepth
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/zdepth \
        ..
    cmake --build . -j 12 --config Release --target install
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
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/yaml-cpp \
        -DYAML_APPLE_UNIVERSAL_BIN=OFF \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DYAML_CPP_BUILD_CONTRIB=OFF \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_CPP_INSTALL=ON \
        ..
    cmake --build . -j 12 --config Release --target install
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
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/libzmq \
        -DBUILD_SHARED=OFF -DBUILD_STATIC=ON \
        -DBUILD_TESTS=OFF -DWITH_TLS=OFF \
        -DWITH_LIBSODIUM=OFF \
        -DWITH_LIBSODIUM_STATIC=OFF \
        ..
    cmake --build . -j 12 --config Release --target install
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
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cppzmq \
        -DZeroMQ_DIR=${LOCAL_DIR}/libzmq/lib/cmake/ZeroMQ \
        -DCPPZMQ_BUILD_TESTS=OFF \
        ..
    cmake --build . -j 12 --config Release --target install
    cd ..
    popd
}

# Where we install all our dependencies
export LOCAL_DIR=$HOME/local.ssp
mkdir -p ${LOCAL_DIR}

export SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd | sed -e 's,^/c/,c:/,')"

echo "Cleaning tmp directory"
rm -rf tmp
if [ $? -ne 0 ]; then
    echo "Unable to remove tmp directory"
    exit
fi
mkdir tmp
pushd tmp

export MACOSX_DEPLOYMENT_TARGET="10.11"

install_nasm
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
filename=${prefix}_${version}_ssp_macdep

echo "Packing ${LOCAL_DIR} to ${filename}.tar"
tar -C ${LOCAL_DIR} -cf ${filename}.tar \
  cereal \
  cppzmq \
  ffmpeg \
  libzmq \
  opencv \
  spdlog \
  yaml-cpp \
  zdepth

echo "Compressing ${filename}.tar"
gzip ${filename}.tar
