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

Check the install instructions ![ here ](https://github.com/moetsi/Sensor-Stream-Pipe/blob/master/INSTALL.md)

### Building Sensor Stream Pipe

1) Download and build the project (server and client)

```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
cd Sensor-Stream-Pipe
mkdir build
cd build
cmake ..
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
