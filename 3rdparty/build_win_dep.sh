#!/bin/bash

# Compile Windows dependencies needed to compile Sensor-Stream-Pipe on Windows
# This script should be run under `git bash` command prompt
# Previously, you should install :
# - Visual Studio 2017 Community Edition
# - CMake
#

function install_yasm {
    echo "Install yasm"
    curl -L -O \
      http://www.tortall.net/projects/yasm/releases/yasm-1.3.0-win64.exe

    mkdir bin
    mv yasm-1.3.0-win64.exe bin/yasm.exe
    export PATH=`pwd`/bin:$PATH
    yasm --version
}

function build_ffmpeg {
    echo "Building ffmpeg"
    #https://github.com/BtbN/FFmpeg-Builds/releases/download/autobuild-2021-09-18-12-22/ffmpeg-N-103679-g7bbad32d5a-win64-lgpl-shared.zip
    #export FFMPEG_NAME=ffmpeg-N-103679-g7bbad32d5a-win64-lgpl-shared
    #curl -L -O \
    #  https://github.com/BtbN/FFmpeg-Builds/releases/download/autobuild-2021-09-18-12-22/${FFMPEG_NAME}.zip
    export FFMPEG_NAME=ffmpeg-n4.4-188-g404c9331dd-win64-lgpl-shared-4.4
    cp ../$FFMPEG_NAME.zip .
    pwd
    FFMPEG_ZIP=$PWD/$FFMPEG_NAME.zip
    pushd ${LOCAL_DIR}
    pwd
    rm -Rvf ffmpeg-n4.4-188-g404c9331dd-win64-lgpl-shared-4.4/
    jar xvf $FFMPEG_ZIP
    pwd
    ls 
    popd
    unzip -uod ${LOCAL_DIR} ${FFMPEG_NAME}.zip 
    #unzip dll boom with cygwin libraries versions
    # /d/a/RaaSCL/RaaSCL/3rdparty/tmp/Sensor-Stream-Pipe/3rdparty
    # ffmpeg-N-103679-g7bbad32d5a-win64-lgpl-shared/
    # => /d/a/RaaSCL/RaaSCL/3rdparty/tmp/Sensor-Stream-Pipe/3rdparty/tmp/local.ssp/ffmpeg-N-103679-g7bbad32d5a-win64-lgpl-shared    
    mv -v ${LOCAL_DIR}/${FFMPEG_NAME} ${LOCAL_DIR}/ffmpeg
    return

    # a path of pain below ~
    git clone --depth 1 --branch release/4.3 \
         https://git.ffmpeg.org/ffmpeg.git ffmpeg 
    pushd ffmpeg

    ./configure --prefix=${LOCAL_DIR}/ffmpeg \
         --target-os=win64 --arch=x86_64 --toolchain=msvc \
         --disable-gpl \
         --enable-runtime-cpudetect \
         --enable-yasm \
         --enable-asm \
         --enable-w32threads \
         --disable-programs \
         --disable-ffmpeg \
         --disable-ffserver \
         --disable-ffplay \
         --disable-ffprobe
    make -j12
    make install
    popd
}


# Build minimal OpenCV : core imgproc
function build_opencv {
    echo "Building opencv"
    curl -L -O \
      https://github.com/opencv/opencv/archive/refs/tags/4.5.4.zip
    unzip 4.5.4.zip
        mv opencv-4.5.4 opencv

    pushd  opencv
    mkdir build && cd build
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A x64  \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/opencv \
        -DOPENCV_GENERATE_PKGCONFIG=YES \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_WITH_STATIC_CRT=OFF \
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
    cmake --build . --config Release --target install
    [ ${BUILD_DEBUG} ]  && cmake --build . --config Debug --target install
    cd ..
    popd
}

function build_cereal {
    echo "Building Cereal"
    git clone --depth 1 --branch v1.3.0 \
        https://github.com/USCiLab/cereal.git || exit -1
    pushd cereal
    mkdir build && cd build
    cmake -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cereal \
        -DJUST_INSTALL_CEREAL=ON \
        .. || exit -1
    cmake --build . --config Release --target install || exit -1
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
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/spdlog \
        -DSPDLOG_BUILD_SHARED=OFF \
        .. || exit -=1
    cmake --build . --config Release --target install
    [ ${BUILD_DEBUG} ]  && cmake --build . --config Debug --target install
    cd ..
    popd
}

# https://github.com/catid/Zdepth
function build_zdepth {
    echo "Building zdepth"
    git clone https://github.com/catid/Zdepth.git || exit -1
    pushd Zdepth

    # Commit including our cmake patch
    git checkout 9b333d9aec520 || exit -1

    mkdir build && cd build
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/zdepth \
        -DCMAKE_DEBUG_POSTFIX=d \
        .. || exit -1
    cmake --build . --config Release --target install
    [ ${BUILD_DEBUG} ]  && cmake --build . --config Debug --target install
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
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/yaml-cpp \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DYAML_CPP_BUILD_CONTRIB=OFF \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_CPP_INSTALL=ON \
        .. || exit -1
    cmake --build . --config Release --target install 
    [ ${BUILD_DEBUG} ]  && cmake --build . --config Debug --target install 
    cd ..
    popd
}

# https://github.com/zeromq/libzmq
function build_libzmq {
    echo "Building libzmq"
    git clone --depth 1 --branch v4.3.4 \
        https://github.com/zeromq/libzmq.git
    #git clone https://github.com/zeromq/libzmq.git        
    pushd libzmq
    #git checkout f13f891c911b4e007efd0bf5bd1412874aebd24a 
    mkdir build && cd build
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/libzmq \
        -DBUILD_SHARED=OFF -DBUILD_STATIC=ON -DZMQ_HAVE_IPC=OFF \
        -DBUILD_TESTS=OFF \
        -DWITH_LIBSODIUM=OFF \
        .. || exit -1
    cmake --build . --config Release --target install 
    [ ${BUILD_DEBUG} ]  && cmake --build . --config Debug --target install 
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
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/cppzmq \
        -DZeroMQ_DIR=${LOCAL_DIR}/libzmq/CMake \
        -DCPPZMQ_BUILD_TESTS=OFF \
        .. || exit -1
    cmake --build . --config Release --target install 
    cd ..
    popd
}

# https://github.com/microsoft/Azure-Kinect-Sensor-SDK
function build_k4a {
    echo "Building Azure Kinect Sensor SDK"
    git clone --depth 1 --branch v1.4.1 \
        https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git k4a
    pushd k4a

    # Use our version of spdlog
    patch -p1 < $SOURCE_DIR/k4a.patch

    mkdir build && cd build
    CFLAGS="-MP" CXXFLAGS="-MP" cmake \
        -G "Visual Studio 16 2019" -A "x64" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${LOCAL_DIR}/k4a \
        -Dspdlog_DIR=${LOCAL_DIR}/spdlog/lib/cmake/spdlog \
        -DBUILD_TESTING=OFF \
        ..
    cmake --build . --config Release --target install
    cmake --build . --target install
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

#export BUILD_DEBUG=1

[ ${BUILD_DEBUG} ] && echo "Build Release+Debug version" || echo "Build only Release version"

install_yasm || exit -1
build_ffmpeg || exit -1
build_opencv || exit -1
build_cereal || exit -1
build_spdlog || exit -1
build_zdepth || exit -1
build_yaml_cpp || exit -1
build_libzmq || exit -1
build_cppzmq || exit -1
##build_k4a

prefix=`date +%Y%m%d%H%M`
filename=${prefix}_ssp_windep

echo "Packing ${LOCAL_DIR} to ${filename}.tar"
#tar -C ${LOCAL_DIR} -cf ${filename}.tar \
#  cereal \
#  cppzmq \
#  ffmpeg \
#  k4a \
#  libzmq \
#  opencv \
#  spdlog \
#  yaml-cpp \
#  zdepth

tar -C ${LOCAL_DIR} -cf ${filename}.tar \
  spdlog \
  cereal \
  cppzmq \
  ffmpeg \
  libzmq \
  opencv \
  yaml-cpp \
  zdepth

echo "Compressing ${filename}.tar"
gzip ${filename}.tar
