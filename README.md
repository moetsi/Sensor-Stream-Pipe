# Sensor Stream Pipe

Moetsi's Sensor Stream Pipe (SSP) is the first open-source C++ modular kit-of-parts that compresses, streams, and processes sensor data (RGB-D).
It does this by efficiently compressing raw data streams, allowing developers to send multiple video types over the network in real time. 
Frame data can be sent in its raw form (JPG/PNG frames), or compressed using a myriad of codecs, leveraged on FFmpeg/LibAV and NV Codec to considerably reduce bandwidth strain.

SSP is designed to help overcome the limitations of on-device sensor data processing. 

In its current incarnation, Moetsi’s pipe supports the [Azure Kinect DK](https://azure.microsoft.com/en-us/services/kinect-dk/) RGB-D camera and existing datasets (e.g. [BundleFusion](https://graphics.stanford.edu/projects/bundlefusion/), [MS RGB-D 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/) and [VSFS](http://graphics.stanford.edu/projects/vsfs/)), but we’re planning to support other cameras and devices (e.g. Kinect v2 and Structure Core) in the near future. 
At the same time, we’ve made the Moetsi SSP totally open source so that you can build out support for any devices that you want to too!

## Component parts

The ssp_server is the frame encoder and sender for the Moetsi SSP. 
It gets frames from a device or datasets, encodes and compresses each frame, and then sends them over the internet or your local network (whether 4G, wired or wireless) for remote processing.

The ssp_clients are the frame receiver and decoder. 
They run on the remote processing server and gets the frames from the ssp_server for further processing.

Here’s a screenshot of what the pipe looks like from the client side:

![ NVPipe example ](https://github.com/moetsi/Sensor-Stream-Pipe/raw/master/examples/example.png)
You can see it’s receiving real-time data from a Kinect DK and rendering it for on-screen display. 
In this scenario we achieved a substantial 20x data compression, reducing the stream size from 400 Mbps to just 20 Mbps, along with a PSNR of ~39 dB and a processing overhead of ~10-15 ms.

## Feedback
   
Moetsi's Sensor Stream Pipe is currently in alpha. Features will probably change, bugs will probably be found. 
It's a work in progress after all! 
That said, we welcome both feedback and pull requests.
   
We would also love to hear more about how you plan to use the Moetsi Sensor Stream Pipe! 
So if you have any problems, questions, feature requests, or ideas for improvement, please feel free to reach out at [olenka@moetsi.com](mailto:olenka@moetsi.com) or post on our [forum](https://moetsi.com/pages/community)
   
The better we understand how you’re using the Moetsi SSP, the better we can plan future developments!


## Getting started

The Moetsi Sensor Stream Pipe is designed to overcome the limitations of on-device sensor data processing. 
It does this by encoding and compressing your device’s color or depth frames, and transmitting them to a remote server where they can be decoded and processed at scale.

The Moetsi SSP can be used on two platforms:
* edge devices that need to send frames off device to a remote server, and;
* processing servers that receive frames from multiple sources.

By taking data processing off device, you will be able to run far more powerful computations on your sensor data and make the most of the tools at your disposal.

#### Features include:

- Synchronized streaming of color, depth and IR frames
- Support for Azure Kinect DK (live and recorded video streaming) and image datasets (e.g. [BundleFusion](https://graphics.stanford.edu/projects/bundlefusion/), [MS RGB-D 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/) and [VSFS](http://graphics.stanford.edu/projects/vsfs/))
- Hardware-accelerated encoding (e.g. Nvidia codec), providing you with the lowest possible latency and bandwidth without compromising on quality
- Interoperability with Libav and FFmpeg creates a hyperflexible framework for all the use cases you brilliant developers can come up with!
- Access to the calibration data for each of the sensors on the Kinect, enabling you to build a point cloud from the color and depth images, perform body tracking, bundle fusion, etc. 


### Installation

SSP installation was tested on Ubuntu 18.04. Installing on other recent Linux distributions should be pretty similar, but please check the installation instructions for OpenCV and Kinect DK on your respective platform first. 

Installation instructions for Windows 10 are available on the [ INSTALL ](https://github.com/moetsi/Sensor-Stream-Pipe/blob/master/INSTALL.md) file.

If you encounter any problems or have any suggestions, please let us know by emailing [contact@moetsi.com](mailto:contact@moetsi.com) or post on our [forum](https://moetsi.com/pages/community).

#### Dependencies

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

For more detailed instructions, check the [ INSTALL ](https://github.com/moetsi/Sensor-Stream-Pipe/blob/master/INSTALL.md) file.

#### Building Sensor Stream Pipe

Download and build the project (the ssp_server, ssp_client and ssp_tester):

```
git clone git@github.com:moetsi/Sensor-Stream-Pipe.git
cd Sensor-Stream-Pipe
mkdir build
cd build
cmake .. -DSSP_WITH_KINECT_SUPPORT=ON -DSSP_WITH_K4A_BODYTRACK=ON -DSSP_WITH_NVPIPE_SUPPORT=ON
make
```


You can turn off Kinect, Bodytrack and NVPipe support by changing the following to the ```cmake ..``` options to off:
 
```
-DSSP_WITH_KINECT_SUPPORT=OFF
-DSSP_WITH_K4A_BODYTRACK=OFF
-DSSP_WITH_NVPIPE_SUPPORT=OFF
```

## Usage

The project is broken down into three components:
* **ssp_server**: reads frames on disk and sends them over the network.
* **ssp_client**: receives network frames and decodes them into cv::Mat.
* **ssp_tester**: a reproducible tester for measuring SSP compression and quality.

### SSP Server
    
The ssp_server can stream three types of data:  
* **Images**: frames from images stored on the disk.
* **Video**: encoded frames that have been captured using a color/depth camera, such as the Kinect. All video types supported by FFmpeg/Libav can be processed.
* **Kinect**: Live Kinect DK frame data.

For all three data types, the data can be sent losslessly (very high bandwidth requirements), or compressed (20-50x lower bandwidth requirements), using Libav or NVCodec through the NVPipe. 

As any compression will affect quality, we recommend first experimenting with the ssp_tester to figure out the optimal levels for your use case.

#### Running the ssp_server

```
./bin/ssp_server <configuration file>
```

The ssp_server will start streaming frame data by default, even if no client is connected. 
If no client joins, it will store 1000 frame packets in a buffer before it stops reading frames.

When an ssp_client connects, the packets in the buffer will be sent first. 
After the buffer has emptied, the ssp_server will resume reading frames from the selected input in order, ensuring that no frames are dropped.

#### ssp_server configuration format

The ssp_server configuration is stored in a YAML file. It includes ssp_client host and port, input data configuration and encoding configuration.

The format of the file (encoding Kinect DK frame data with the Nvidia encoder) is as follows:

```
general:
  host: "192.168.1.64"
  port: 9999
  log_level: "debug"
  log_file: "ssp_server.log"
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
    type: "zdepth"
  2: #ir
    type: "nvenc"
    codec_name: "NVPIPE_HEVC"
    input_format: "NVPIPE_UINT16"
    bit_rate: 15000000

```
 
The ```config/``` folder includes a set of examples for all types of data using multiple encoders, codecs and parameters.

#### Generating A Frame List File

To stream image frame data, you need to generate a frame list file. 
The ```data/``` folder includes an example frame list file each for the BundleFusion and MS RGB-D datasets. 
These frame files will work with many other datasets.

See the [ssp-data-scripts](https://github.com/moetsi/ssp-data-scripts) for information on how to generate the frame files.

After extracting the paths, change the paths in the ```examples/stairs-seq-01-frames-color.txt``` and ```examples/stairs-seq-01-frames-depth.txt``` to match your images location.


### SSP Client

The ssp_client application receives network packets from the ssp_server, de-serializes and decodes them, and makes them available for processing.

To provide an example, **ssp_client_opencv** converts the encoded frames into displayable OpenCV, whereas **ssp_client_k4a** connects with remote Azure Kinect body tracking and **ssp_client_pointcloud** generates a pointcloud from your Kinect DK data. 
These three different clients provide three examples of output format, but you can adapt this code for final processing however you like. 

Have a look at ssp_client_template for a bare-bones template that you can play around with. 
For example, you could use the code with BundleFusion or another, comparable algorithm. 
This is incredibly beneficial if you’re ingesting multiple sensor streams (e.g. surveillance/security), and want to run computer vision algorithms such as skeleton tracking. 
And also lends itself perfectly to projects involving sensors in multiple locations, i.e. if you want to combine streams from a variety of drones operating in different locations into one stream.

```
./bin/ssp_client_opencv <port> (<log level>) (<log file>)
./bin/ssp_client_k4a <port> (<log level>) (<log file>)
./bin/ssp_client_pointcloud <port> (<output_folder>) (<log level>) (<log file>)
```

Due to the enqueuing process as described in the ssp_server section, it is recommended that you start the ssp_client application first.

#### Parallel processing

By default, the ssp_client can receive frames from multiple servers in parallel, and will process the input using a [fair queuing method](http://zguide.zeromq.org/page:all#Divide-and-Conquer).

### SSP Tester

The Moetsi SSP has been designed to stream your sensor data in real-time. 
However, as streaming data in real-time is not reproducible, it is hard to test which parameters will work best for you. 
You need a way to "record results," so to speak. 
Thus, we built the Moetsi SSP Tester, which enables you to test how various settings and parameters affect latency, bandwidth and quality.

The SSP Tester runs the full encoding process for an existing video or image dataset, and returns metrics for **MSE**, **PSNR**, **MSSIM**. 
It uses the same configuration file format as the Moetsi ssp_server and supports the same input data and parameters to ensure that the results are comparable.

```
./bin/ssp_tester <configuration file> (<test time for live data>)
```

Here is an example of the output for a Kinect DK video (color and depth data) with 20x compression:

```
...
[statistics];[0]
    [time];[0];32.4667;seconds
    [original_size];[0];240544293;bytes
    [compressed_size];[0];14909024;bytes
    [original_bandwidth];[0];59.2717;Mbps
    [compressed_bandwidth];[0];3.67368;Mbps
    [compression ratio];[0];16.1341;x
    [latency];[0];10.2854;ms
    [PSNR];[0];39.1411
    [MSSIM];[0];0.938965;0.951397;0.93559;0.0
[statistics];[1]
    [time];[1];32.4667;seconds
    [original_size];[1];718110720;bytes
    [compressed_size];[1];51234739;bytes
    [original_bandwidth];[1];176.947;Mbps
    [compressed_bandwidth];[1];12.6246;Mbps
    [compression ratio];[1];14.0161;x
    [latency];[1];8.25051;ms
    [MSE];[1];2.0558
    [MSE_4096];[1];2.05479
...
```
The key results here are a **PSNR** of just **39 dB** and **latency** of only **8.25 ms**.

## Sensor Stream Pipe Development

### About Moetsi
At Moetsi we are super excited about the idea of digitizing reality. Creating a seamless interface between the world as we know it, and a world augmented, improved and expressed through new technologies is plain cool. But we think there’s a problem. On-device computation is limited, platform-specific frameworks are restrictive, and sorting raw depth data is seriously challenging.

To address the first problem, we've created the Moetsi Sensor Stream Pipe; to make it easier to process off-device without throttling bandwidth. It means you are no longer confined to the computational limits of your local device, and you don’t have to make a massive trade-off on time-to-computation because our pipeline is super fast (latency is less than 30 ms for Kinect data). 

But it doesn’t end here. 

Our pipeline is just one of the first pieces of the puzzle. To develop a robust enough infrastructure to support a true digital twin of the physical world, a lot more needs to be done. This includes creating algorithms that can turn this raw depth data into real, usable applications. 

### How to Contribute

We’re always excited to work with like-minded people, and invite you to experiment with our pipeline however you like! If you enjoy our work and think you can help take this project to the next level, feel free to drop us a message on [olenka@moetsi.com](mailto:olenka@moetsi.com) to get involved.

If you happen to discover any bugs in our code, we’d really appreciate knowing about them. Please just create an issue here on GitHub. 

In terms of related projects that fall outside of this repo’s scope, we’d be super excited to see, and think the community could benefit from development on:

#### More devices

Working with the Kinect v2, and other sensors such as the Structure Core sensor.

#### Integrations

Any other sort of output that you can imagine!

#### Encoding

Improve encoding performance on AMD/Intel graphic cards, by using the AMD Media Codec/Intel Quick Sync Video instead of relying on libav (VAAPI or OpenCK) for hardware accelerated encoding. Feel free to do the same for Intel cards using Intel Quick Sync Video too! 

## Moetsi’s Permissive License

Moetsi’s Sensor Stream Pipe is licensed under the MIT license. That means that we don’t require attribution, but we’d really like to know what cool things you’re using our pipe for. Drop us a message on [olenka@moetsi.com](mailto:olenka@moetsi.com) or post on our [forum](https://moetsi.com/pages/community) to tell us all about it!

## Support Moetsi!

Our Sensor Stream Pipe is always going to be free, but it has taken a lot of blood, sweat and tears to get to this point. If you love what we’ve made, please consider reaching out to [olenka@moetsi.com](mailto:olenka@moetsi.com).


## Authors

* **André Mourão** - [amourao](https://github.com/amourao)
* **Olenka Polak** - [olenkapolak](https://github.com/olenkapolak)
* **Adam Polak** - [adammpolak](https://github.com/adammpolak)
