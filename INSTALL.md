# Sensor Stream Pipe Instalation


[Linux instructions](#linux)

[Windows instructions](#windows)


## Linux

To get our Sensor Stream Pipe up and running, you will require the following:

The following steps were tested on Ubuntu 18.04. Installing on other recent Linux distributions should be pretty similar, but please check the installation instructions for OpenCV and Kinect DK on your respective platform first. 
Installation instructions for Windows should be ready soon.
If you encounter any problems or have any suggestions, please let us know by emailing contact@moetsi.com or post on our [forum](https://moetsi.com/pages/community).

### Dependencies

To get our Sensor Stream Pipe up and running, you will require the following:

* [OpenCV](https://opencv.org/) 3.2.0 (tested on version available on Ubuntu 18.04 repo) is used for image processing.
* [libav](https://github.com/libav/libav/) 3.4.6 (tested on version available on Ubuntu 18.04 repo) encodes, decodes and processes image frames.
* [Cereal](https://uscilab.github.io/cereal/) 1.2.2 (headers only) serializes data for network transmission.
* [ZeroMQ](http://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq/) (libzmq3 4.3.1, cppzmq 4.3.0) perform network and low-level I/O operations.
* [spdlog](https://github.com/gabime/spdlog/) 1.4.1 Logging library.
* [yaml-cpp](https://github.com/jbeder/yaml-cpp/) 0.6.0 reads server configuration files.
* [Zdepth](https://github.com/catid/Zdepth.git): compresses depth data.
* [NvPipe](https://github.com/NVIDIA/NvPipe/) (*optional*, but **recommended if you have an NVidia GPU** ) encodes and decodes frames. This is optional, but recommended for users with Nvidia GPUs.
* [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/) 1.2 (*optional*) accesses Kinect DK data.
* [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/bs-cyrl-ba/azure/Kinect-dk/body-sdk-download/) 0.9.3 (*optional*) SSP Body Tracking client.

### Download and install repo libraries

#### OpenCV 3.2.0

```
sudo apt install libopencv-dev libopencv-core-dev uuid-dev
```
#### Libav 3.4.6

```
sudo apt install libavformat-dev libavutil-dev libavcodec-dev libavfilter-dev
```

### Download and extract "out-of-repo" libraries


First, create a folder where local libs are to be installed:

```
mkdir ~/libs
mkdir ~/libs/srcOriginal
```

#### Cereal 1.2.2

```
cd ~/libs/srcOriginal
wget https://codeload.github.com/USCiLab/cereal/tar.gz/v1.2.2
tar xf v1.2.2
cp -r cereal-1.2.2/include ~/libs
```

#### ZeroMQ

If you want to take advantage of ZMQ pulling support to check for new frames, compile ZMQ and CPPZMQ with the [draft API suppport](https://github.com/zeromq/cppzmq/issues/175).
Also, you must set SSP_WITH_ZMQ_POLLING at SSP build time.
Discussion available [here](https://github.com/moetsi/Sensor-Stream-Pipe/pull/7)


##### libzmq3 4.3.1

```
cd ~/libs/srcOriginal
wget https://github.com/zeromq/libzmq/releases/download/v4.3.1/zeromq-4.3.1.tar.gz
tar xf zeromq-4.3.1.tar.gz
cd zeromq-4.3.1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make install -j4
```

##### cppzmq 4.3.0

```
cd ~/libs/srcOriginal
wget https://github.com/zeromq/cppzmq/archive/v4.3.0.tar.gz
tar xf v4.3.0.tar.gz
cd cppzmq-4.3.0
cp *.hpp ~/libs/include
```

#### yaml-cpp 0.6.0

```
cd ~/libs/srcOriginal
wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.0.tar.gz
tar xf yaml-cpp-0.6.0.tar.gz
cd yaml-cpp-yaml-cpp-0.6.0
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make install
```

#### Zdepth

```
cd ~/libs/srcOriginal
git clone https://github.com/catid/Zdepth.git
cd Zdepth
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make install
cp libzdepth.a ~/libs/lib/
cp zstd/libzstd.a ~/libs/lib/
```

#### spdlog

```
cd ~/libs/srcOriginal
wget https://github.com/gabime/spdlog/archive/v1.4.1.tar.gz
tar xf v1.4.1.tar.gz
cd spdlog-1.4.1 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make -j
make install
```

##### NVPipe (optional, recommended for users with Nvidia GPU)

```
cd ~/libs/srcOriginal
git clone https://github.com/NVIDIA/NvPipe.git
cd NvPipe/
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make
make install
```

##### Azure Kinect SDK 1.2 (optional)

*Note: to avoid getting a password prompt, run any command as sudo before starting this section of the tutorial*

1) Add the Linux Software Repository for Microsoft Products.
```
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```

2) Install Azure Kinect SDK 1.2
```
sudo apt install libk4a1.2 libk4a1.2-dev k4a-tools
```

3) To be able to use the Kinect as non-root, please run the following:
```
wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules
sudo cp 99-k4a.rules /etc/udev/rules.d/
```

4) In the current package, the link to the canonical version of the depth lib is missing.
You can create it by running the following command:

```
sudo ln -s /usr/lib/x86_64-linux-gnu/libdepthengine.so.2.0 /usr/lib/x86_64-linux-gnu/libdepthengine.so
```

##### Azure Kinect Body Tracking SDK (optional)

Check instructions above to add the Linux Software Repository for Microsoft Products and then do:


```
sudo apt install libk4abt0.9-dev
```


#### Building Sensor Stream Pipe

Download and build the project (the ssp_server, ssp_client and ssp_tester):

```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
cd Sensor-Stream-Pipe
mkdir build
cd build
cmake .. -DSSP_WITH_KINECT_SUPPORT=OFF -DSSP_WITH_K4A_BODYTRACK=OFF -DSSP_WITH_NVPIPE_SUPPORT=OFF
make
```


You can turn on Kinect, Bodytrack and NVPipe support by adding the following to the ```cmake ..``` line respectively:
 
```
-DSSP_WITH_KINECT_SUPPORT=ON 
-DSSP_WITH_K4A_BODYTRACK=ON
-DSSP_WITH_NVPIPE_SUPPORT=ON
```


## Windows


Windows installation process was performed using [vcpkg](https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=msvc-160) to install most dependencies.
Tested on Windows 10 Build 19041, Visual Studio 2019 Community Edition (VS).

This process may also work for Linux, but this was not tested.

### Install vcpkg

Follow vcpkg installation instructions available [here](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows)

### Install dependencies available on vcpkg

Install dependencies using vcpkg.

```
vcpkg install azure-kinect-sensor-sdk:x64-windows cereal:x64-windows cppzmq:x64-windows ffmpeg:x64-windows opencv3:x64-windows spdlog:x64-windows yaml-cpp:x64-windows zeromq:x64-windows
```

### Build and install remaining dependecies 

Prepare a directory to place the remaining dependecies lib and include files (refered henceforth as `$LIBS`).
This directory should have a `lib` and `include` subfolders with the corresponding `.lib` and headers respectively.

#### Zdepth

Clone Zdepth repo

```
git clone https://github.com/catid/Zdepth.git
```

Open CMakeLists file in VS and build accorcing to your desired profile (x86 or x64; Debug or Release).

If you did not specify an install dir during the CMake configuration, copy the `Zdepth\include` and output lib folders (e.g. `ZDepth\out\*`) to `$LIBS`.

#### Azure Kinect Body Tracking SDK (optional)

Install Azure Body Tracker SDK from the instructions available [here](https://www.microsoft.com/en-us/download/details.aspx?id=100942).

Copy the SDK include and lib files from the SDK install list to `$LIBS`, or add the SDK path to SSP CMakeLists (see below)

#### Building Sensor Stream Pipe

Clone the SSP repo


```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
```

Due to the diferences in the build process, the Windows CMake file is named CMakeListsWindows.txt at the root of the SSP repo.

Thus, you shoud delete CMakeLists.txt and rename CMakeListsWindows.txt to CMakeLists.txt.

Open CMakeLists.txt in VS.

Replace/Add the include ("C://Users//Andre//source//repos//vcpkg//installed//x64-windows//include") and link paths ("C://Users//Andre//source//repos//vcpkg//installed//x64-windows//lib") at the top of the file with your `$LIBS` paths

```
include_directories("C://Users//Andre//source//repos//vcpkg//installed//x64-windows//include")
link_directories("C://Users//Andre//source//repos//vcpkg//installed//x64-windows//lib")
```

You can also add your `vcpkg//installed//` dir to the include and link paths.

After replacing the paths, set the desired compile options (SSP_WITH_KINECT_SUPPORT, SSP_WITH_K4A_BODYTRACK, ...), regenerate CMakeCache and build the project.

#### Linking errors?


if you have linking errors (missing .lib files), try replacing the short lib name with the full lib path in CMake: 
"libzmq" -> "C:/Users/Andre/source/repos/vcpkg/installed/x64-windows/lib/libzmq.lib"