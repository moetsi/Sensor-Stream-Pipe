# Sensor Stream Pipe

The Sensor Stream Pipe is a C++ API that efficiently compresses and sends multiple types of video streams (color, depth, ...) over the network in real time.
It's designed to simplify remote data rendering and processing of color and depth data.
It supports the Azure Kinect DK RGB-D camera and existing datasets (Bundle Fusion, MS RGB-D 7 scenes).

Frame data can be send in it's raw form (JPG/PNG frames), or compressed using a myriad of codecs, leveraged on FFmpeg/LibAV and NVPipe.

![ NVPipe example ](https://github.com/moetsi/Sensor-Stream-Pipe/raw/master/examples/example.png)Here's a view from the client size, receiving data in real time from a Kinect DK and showing it on screen.
Data was compressed from 400 Mbits to 20 Mbits (PSNR ~36), with a processing overhead of ~30 ms.

# Getting started

The Sensor Stream Pipe is designed to take over the encoding and processing of frames on the server and the decoding on the client, allowing to use the decoded frames as desired.
Currently, the example client converts the received frames into an OpenCV matrix and shows them on the screen.

## Installation

The following steps were tested on Ubuntu 18.04 (both client and the servers).

### Dependencies

* [OpenCV](https://opencv.org/) 3.2.0 (tested on version available on Ubuntu 18.04 repo): Image processing;
* [libav](https://github.com/libav/libav/) 3.4.6 (tested on version available on Ubuntu 18.04 repo): Encode, decode and process image frames;
* [Cereal](https://uscilab.github.io/cereal/) 1.2.2 (headers only): Data serialization for network transmission;
* [ZeroMQ](http://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq/) (libzmq3 4.3.1, cppzmq 4.3.0):  Network and low-level I/O;
* [yaml-cpp](https://github.com/jbeder/yaml-cpp/) 0.6.0: Reading server configuration file;
* [NvPipe](https://github.com/NVIDIA/NvPipe/) (*optional*, but **recommended if you have an NVidia GPU** ): Encode and decode frames;
* [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/) 1.2 (*optional*): Access Kinect DK data.

TODO: the remainder of this section may be better suited for an INSTALL.md file

#### Download and install repo libraries

#### OpenCV 3.2.0

```
sudo apt install libopencv-dev libopencv-core-dev uuid-dev
```
#### Libav 3.4.6

```
sudo apt install libavformat-dev libavutil-dev libavcodec-dev libavfilter-dev
```


#### Download and extract "out-of-repo" libraries


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

#### NVPipe (optional, recommended for users with Nvidia GPU)

```
cd ~/libs/srcOriginal
git clone git@github.com:NVIDIA/NvPipe.git
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
### Building Sensor Stream Pipe

1) Add header libraries to search path

```
export CPATH=~/libs/include:$CPATH
export C_INCLUDE_PATH =~/libs/include:$C_INCLUDE_PATH
```

2) Download and build the project (server and client)

```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
cd Sensor-Stream-Pipe
cmake .
make
```

## Usage

The project is divided into two components:
* **server**: read frames on disk and sends them over the network
* **client**: receive network frames and decodes them into cv::Mat

### Server

The server can stream three types of data:
* **frames**: sends frames over the network compressed or in their original formats (png, jpg);
* **video**: sends encoded frames (with or without reencoding) from a previously encoded video. Supports all video types supported by FFmpeg;
* **kinect**: encodes Kinect DK frame data to video in real time.

For all these data types, the data can be sent raw (no compression, very high bandwidth requirements) or compressed (high quality, 20-50x lower bandwidth requirements), using libav or NVCodec (though NVPipe).


#### Running the server

```
./bin/ssp_server <configuration file>
```

By default, the server will start streaming frame data, even if no client is connected.
If no client joins, it'll store 1000 frame packets in a buffer, and I'll stop reading frames from the selected input (image files, video or Kinect).
When a client connects, packets in buffer will be sent first, and, as the buffer empties, the server will resume reading frames from the selected input.

#### Server configuration format

Server configuration (client host and port, input data configuration and encoding/encoding configuration) is stored in a YAML file.
The format of the file (encoding Kinect DK frame data with the Nvidia encoder) is as follows:

```
general:
  host: "192.168.1.64"
  port: 9999
  frame_source:
    type: "kinect"
    parameters:
        stream_color_video: True
        stream_depth_video: True
        stream_ir_video: True
        streaming_color_format: "K4A_IMAGE_FORMAT_COLOR_BGRA32"
        streaming_color_resolution: "K4A_COLOR_RESOLUTION_720P"
        streaming_depth_mode: "K4A_DEPTH_MODE_NFOV_UNBINNED"
        wired_sync_mode: "K4A_WIRED_SYNC_MODE_STANDALONE"
        streaming_rate: "K4A_FRAMES_PER_SECOND_30"
        absoluteExposureValue: 0
video_encoder:
  0: #color
    type: "nvenc"
    codec_name: "NVPIPE_HEVC"
    input_format: "NVPIPE_RGBA32"
    bit_rate: 4000000
  1: #depth
    type: "nvenc"
    codec_name: "NVPIPE_HEVC"
    input_format: "NVPIPE_UINT16"
    bit_rate: 15000000
  2: #ir
    type: "nvenc"
    codec_name: "NVPIPE_HEVC"
    input_format: "NVPIPE_UINT16"
    bit_rate: 15000000

```

The ```configs/``` folder includes a set of examples for all types of data using multiple encoders, codecs and parameters.

#### Generating frame list file

To stream image frame data, you need to generate a frame list file.
The ```examples/``` folder includes a set of example frame files for Bundle Fusion and MS RGB-D datasets.
See the [ssp-data-scripts](https://github.com/moetsi/ssp-data-scripts) to see how to generate the frame files.

```
wget http://download.microsoft.com/download/2/8/5/28564B23-0828-408F-8631-23B1EFF1DAC8/stairs.zip
unzip stairs.zip
cd strais
unzip seq-01.zip
```

After extracting the paths, change the paths in the ```examples/stairs-seq-01-frames-color.txt``` and ```examples/stairs-seq-01-frames-depth.txt``` to match your images location.

### Client

The client application receives network packets from the server, de-serializes and decodes them, and makes them available for processing.
In this example, it'll convert the encoded frames into OpenCV matrices and show them to the user.
The goal is for users to adapt this code for they final processing goal.

```
./bin/ssp_client <port>
```
Due to the queuing process described in the server section, it is recommended you start the client application first.

#### Parallel processing

By default, the client can receive frames from multiple servers in parallel, and will process the input using a [fair queuing method](http://zguide.zeromq.org/page:all#Divide-and-Conquer).

## System Architecture

### Server

```
//start zeromq socket
zmq::context_t context(1);
zmq::socket_t socket(context, ZMQ_PUSH);

//reading parameters and preparing Reader and Encoder
std::string codec_parameters_file = std::string(argv[1]);
YAML::Node codec_parameters = YAML::LoadFile(codec_parameters_file);

...

socket.connect("tcp://" + host + ":" + std::to_string(port));

while (1) {

  while (v.empty()) {
    ... // get frame from reader
  }

  if (!v.empty()) {
    // serialize and send message reader
    std::string message = cerealStructToString(v);

    zmq::message_t request(message.size());
    memcpy(request.data(), message.c_str(), message.size());
    socket.send(request);
  }
}
```

### Client

```
// prepare socket
...
zmq::context_t context(1);
zmq::socket_t socket(context, ZMQ_PULL);
socket.bind("tcp://*:" + std::string(argv[1]));
...

for (;;) {
  // Receive frame set
  zmq::message_t request;

  socket.recv(&request);
  ...
  // Unserialize frame set
  std::string result =
      std::string(static_cast<char *>(request.data()), request.size());

  std::vector<FrameStruct> f_list =
      parseCerealStructFromString<std::vector<FrameStruct>>(result);
  ...

  for (FrameStruct f : f_list) {
    // Decode frame
    ...

    if (imgChanged && !img.empty()) {
      // Prepare frame for display
      ...

      // The frame is ready in img, your application can plug here for additional processing
      cv::namedWindow(decoder_id);
      cv::imshow(decoder_id, img);
      cv::waitKey(1);
      imgChanged = false;
    }
  }
}
```

### Encoding color frames

TODO: add details on how color and depth frames are encoded and the tradeoffs between methods.

### Encoding depth frames


## Benchmark:

TODO: add compression ratio, PSNR (color frames), MSE (depth frames) and latency

#### MS RGB-D 7-Scenes

#### BundleFusion

#### Kinect DK data

## Roadmap

TODO: do you think this is a good idea

## Authors

* **André Mourão** - [amourao](https://github.com/amourao)
* **Adam Polak** - [adammpolak](https://github.com/adammpolak)
