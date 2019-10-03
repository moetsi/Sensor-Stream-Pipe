# Sensor Stream Pipe Instalation

Sensor Stream Pipe depends on the following libraries:

* [OpenCV](https://opencv.org/) 3.2.0 (tested on version available on Ubuntu 18.04 repo): Image processing;
* [libav](https://github.com/libav/libav/) 3.4.6 (tested on version available on Ubuntu 18.04 repo): Encode, decode and process image frames;
* [Zdepth](https://github.com/catid/Zdepth.git): compress depth data;
* [Cereal](https://uscilab.github.io/cereal/) 1.2.2 (headers only): Data serialization for network transmission;
* [spdlog](https://github.com/gabime/spdlog/) 1.4.1: logging library;
* [yaml-cpp](https://github.com/jbeder/yaml-cpp/) 0.6.0: Reading server configuration file;
* [ZeroMQ](http://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq/) (libzmq3 4.3.1, cppzmq 4.3.0):  Network and low-level I/O;
* [NvPipe](https://github.com/NVIDIA/NvPipe/) (*optional*, but **recommended if you have an NVidia GPU** ): Encode and decode frames;
* [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/) 1.2 (*optional*): Access Kinect DK data.
* [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/bs-cyrl-ba/azure/Kinect-dk/body-sdk-download/) 0.9.3 (*optional*): SSP Body Tracking client.

## Download and install repo libraries

### OpenCV 3.2.0

```
sudo apt install libopencv-dev libopencv-core-dev uuid-dev
```
### Libav 3.4.6

```
sudo apt install libavformat-dev libavutil-dev libavcodec-dev libavfilter-dev
```

## Download and extract "out-of-repo" libraries


First, create a folder where local libs are to be installed:

```
mkdir ~/libs
mkdir ~/libs/srcOriginal
```

### Cereal 1.2.2

```
cd ~/libs/srcOriginal
wget https://codeload.github.com/USCiLab/cereal/tar.gz/v1.2.2
tar xf v1.2.2
cp -r cereal-1.2.2/include ~/libs
```

### ZeroMQ


#### libzmq3 4.3.1

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

#### cppzmq 4.3.0

```
cd ~/libs/srcOriginal
wget https://github.com/zeromq/cppzmq/archive/v4.3.0.tar.gz
tar xf v4.3.0.tar.gz
cd cppzmq-4.3.0
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make install
```

### yaml-cpp 0.6.0

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

### Zdepth

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

### spdlog

```
cd ~/libs/srcOriginal
wget https://github.com/gabime/spdlog/archive/v1.4.1.tar.gz
tar xf v1.4.1.tar.gz
cd spdlog-1.4.1 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make -j
make install
```

#### NVPipe (optional, recommended for users with Nvidia GPU)

```
cd ~/libs/srcOriginal
git clone https://github.com/NVIDIA/NvPipe.git
cd NvPipe/
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/libs
make
make install
```

#### Azure Kinect SDK 1.2 (optional)

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

#### Azure Kinect Body Tracking SDK (optional)

Check instructions above to add the Linux Software Repository for Microsoft Products and then do:


```
sudo apt install libk4abt0.9-dev
```


### Building Sensor Stream Pipe

1) Add header libraries to search path

```
export CPATH=~/libs/include:$CPATH
export C_INCLUDE_PATH=~/libs/include:$C_INCLUDE_PATH
```

2) Download and build the project (server, clients and tester)

```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
cd Sensor-Stream-Pipe
mkdir build
cd build
cmake .. -DSSP_WITH_KINECT_SUPPORT=OFF -DSSP_WITH_K4A_BODYTRACK=OFF -DSSP_WITH_NVPIPE_SUPPORT=OFF
make
```


You can turn off Kinect, Bodytrack and NVPipe support by adding the following to the ```cmake ..``` line respectively:
 
```
-DSSP_WITH_KINECT_SUPPORT=OFF 
-DSSP_WITH_K4A_BODYTRACK=OFF
-DSSP_WITH_NVPIPE_SUPPORT=OFF
```