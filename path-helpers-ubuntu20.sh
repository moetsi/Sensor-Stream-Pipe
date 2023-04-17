#!/bin/bash
dos2unix $(find . -name "*.sh")
# source  /opt/intel/openvino_2021/bin/setupvars.sh 
export CMAKE_MODULE_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/cmake
export LD_LIBRARY_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/lib:/opt/intel/openvino_2021/deployment_tools/inference_engine/lib/intel64/:/opt/intel/openvino_2021/deployment_tools/inference_engine/external/tbb/lib
# export DEPTHAI_LEVEL=trace
export CMAKE_PREFIX_PATH=/opt/intel/openvino_2021/deployment_tools/ngraph/cmake:/opt/intel/openvino_2021/deployment_tools/inference_engine/share/
export LD_LIBRARY_PATH=$PWD/3rdparty/tmp/local.ssp/ffmpeg/lib/:$PWD/3rdparty/tmp/local.ssp/openh264/lib/:$LD_LIBRARY_PATH
