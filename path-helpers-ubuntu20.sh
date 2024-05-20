#!/bin/bash
# Convert all shell script files from DOS to UNIX format in the current directory and subdirectories
dos2unix $(find . -name "*.sh")
# The following line is commented out; it's used to source environment variables for OpenVINO toolkit
# source  /opt/intel/openvino_2021/bin/setupvars.sh 
# Set the CMake module path to include OpenVINO's nGraph CMake files
export CMAKE_MODULE_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/cmake
# Set the library path to include OpenVINO's nGraph and Inference Engine libraries
export LD_LIBRARY_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/lib:/opt/intel/openvino_2021/deployment_tools/inference_engine/lib/intel64/:/opt/intel/openvino_2021/deployment_tools/inference_engine/external/tbb/lib
# The following line is commented out; it's used to set the DEPTHAI_LEVEL environment variable to debug
# export DEPTHAI_LEVEL=debug
# Set the CMake prefix path to include OpenVINO's nGraph and Inference Engine CMake files
export CMAKE_PREFIX_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/cmake:/opt/intel/openvino_2021/deployment_tools/inference_engine/share/
# Update the library path to include local SSP dependencies for ffmpeg and openh264
export LD_LIBRARY_PATH=$PWD/3rdparty/tmp/local.ssp/ffmpeg/lib/:$PWD/3rdparty/tmp/local.ssp/openh264/lib/:$LD_LIBRARY_PATH