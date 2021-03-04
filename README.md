# [Checkout the Sensor Stream Pipe gitbook for full documentation](https://moetsi.gitbook.io/sensor-stream-pipe/)
![image](https://user-images.githubusercontent.com/12940507/109909944-06152900-7c75-11eb-91c6-fd52b347617b.png)

# What is Sensor Stream Pipe?

Moetsi's Sensor Stream Pipe \(SSP\) is the first open-source C++ modular kit-of-parts that compresses, streams, and processes sensor data \(RGB-D\). It does this by efficiently compressing raw data streams, allowing developers to send multiple video types over the network in real time. Frame data can be sent in its raw form \(JPG/PNG frames\), or compressed using a myriad of codecs, leveraged on FFmpeg/LibAV and NV Codec to considerably reduce bandwidth strain.

SSP is designed to help overcome the limitations of on-device sensor data processing. By taking data processing off device, you will be able to run far more powerful computations on your sensor data and make the most of the tools at your disposal.

The Moetsi Sensor Stream Pipe is designed to overcome the limitations of on-device sensor data processing. It does this by encoding and compressing your device’s color or depth frames, and transmitting them to a remote server where they can be decoded and processed at scale.

Currently, Moetsi’s Sensor Stream Pipe supports:

* .mkv \(matroska\) RGB-D recordings
* [Azure Kinect DK](https://azure.microsoft.com/en-us/services/kinect-dk/) RGB-D camera 
* seminal computer vision/spatial computing datasets \(e.g. [BundleFusion](https://graphics.stanford.edu/projects/bundlefusion/), [MS RGB-D 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/) and [VSFS](http://graphics.stanford.edu/projects/vsfs/)\)

We’re planning to support other cameras and devices \(e.g. Kinect v2 and Structure Core\) in the near future. At the same time, we’ve made the Moetsi SSP totally open source so that anyone can build out support for any device.

[Checkout the Sensor Stream Pipe gitbook for full documentation](https://moetsi.gitbook.io/sensor-stream-pipe/)

**Features include:**

* Synchronized streaming of color, depth and IR frames
* Support for Azure Kinect DK \(live and recorded video streaming\) and image datasets \(e.g. [BundleFusion](https://graphics.stanford.edu/projects/bundlefusion/), [MS RGB-D 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/) and [VSFS](http://graphics.stanford.edu/projects/vsfs/)\) and .mkv \(matroska\) files
* Hardware-accelerated encoding \(e.g. Nvidia codec\), providing you with the lowest possible latency and bandwidth without compromising on quality
* Interoperability with Libav and FFmpeg creates a hyperflexible framework for all the use cases you brilliant developers can come up with!
* Access to the calibration data for each of the sensors on the Kinect, enabling you to build a point cloud from the color and depth images, perform body tracking, bundle fusion, etc.

### But why though...?

* If you have 4 sensor streams and want to do an environment reconstruction using their data feeds
* If you have a couple of sensors and want to find where they are relative to each other
* You want to run pose detection algorithms on a dozen sensors and synthesize the results into a single 3D model
* Basically if you want to do any spatial computing/computer vision on multiple incoming data streams

You can use Sensor Stream Server to send compressed sensor data to reduce bandwidth requirements and Sensor Stream Client to receive these streams as an ingestion step for a computer vision/spatial computing pipeline.

If you want to synthesize RGB-D+ data from multiple feeds in real-time, you will probably need something like Sensor Stream Pipe.

## Component parts

[Checkout the Sensor Stream Pipe gitbook for full documentation](https://moetsi.gitbook.io/sensor-stream-pipe/)

### [Sensor Stream Server](https://moetsi.gitbook.io/sensor-stream-pipe/components-overview/sensor-stream-server)

The ssp\_server is the frame encoder and sender.

"Frames" are a sample of data from a frame source. For example, the Azure Kinect collects: RGB \(color\), depth, and IR data. If we want to stream RGB-D and IR, we sample our frame source \(the Azure Kinect\), and create 3 frames, one for each frame type: 1 for color data, 1 for depth data, and 1 for ir data. We then package these 3 frames as a zmq message and send through a zmq socket.

Sensor Stream Server reads its configurations from a yaml file \(examples in /configs\). The config file provides Sensor Stream Server: a destination for its frames, the frame source \(video, Azure Kinect, or dataset\), and how each frame type should be encoded.

### [Sensor Stream Client](https://moetsi.gitbook.io/sensor-stream-pipe/components-overview/sensor-stream-client)

The ssp\_clients are the frame receiver and decoder. They run on the remote processing server and receive the frames from the ssp\_server for further processing.

There are a few templates for how you can use Sensor Stream Client in 

#### Sensor Stream Client with OpenCV processing

If you run Sensor Stream Client with OpenCV visualization:

![ NVPipe example ](https://github.com/moetsi/Sensor-Stream-Pipe/raw/master/examples/example.png)

 You can see it’s receiving real-time data from a Kinect DK and rendering it for on-screen display. In this scenario we achieved a substantial 20x data compression, reducing the stream size from 400 Mbps to just 20 Mbps, along with a PSNR of ~39 dB and a processing overhead of ~10-15 ms 😱.

Sensor Stream Client is built so it can be an ingestion step for a spatial computing/computer vision pipeline.

### [Sensor Stream Tester](https://moetsi.gitbook.io/sensor-stream-pipe/components-overview/sensor-stream-tester)

A reproducible tester for measuring SSP compression and quality. You can use this to measure how different encodings and settings affect bandwidth/compression.

## Getting started

We recommend going through [Streaming a Video from our Gitbook](https://moetsi.gitbook.io/sensor-stream-pipe/streaming-a-video)

to get up to speed quickly. You will stream using Sensor Stream Server and receive on Sensor Stream Client a pre-recorded RGB-D+ stream to get a quick feel of what Sensor Stream Pipe does.

## Sensor Stream Pipe Development

### Feedback

Moetsi's Sensor Stream Pipe is currently in alpha. Features will probably change, bugs will probably be found. It's a work in progress after all! That said, we welcome both feedback and pull requests.

We would also love to hear more about how you plan to use the Moetsi Sensor Stream Pipe! So if you have any problems, questions, feature requests, or ideas for improvement, please feel free to reach out at [olenka@moetsi.com](mailto:olenka@moetsi.com).

The better we understand how you’re using the Moetsi SSP, the better we can plan future developments!

### About Moetsi

At Moetsi we are super excited about the idea of digitizing reality. Creating a seamless interface between the world as we know it, and a world augmented, improved and expressed through new technologies is plain cool. But we think there’s a problem. On-device computation is limited, platform-specific frameworks are restrictive, and sorting raw depth data is seriously challenging.

To address the first problem, we've created the Moetsi Sensor Stream Pipe; to make it easier to process off-device without throttling bandwidth. It means you are no longer confined to the computational limits of your local device, and you don’t have to make a massive trade-off on time-to-computation because our pipeline is super fast \(latency is less than 30 ms for Kinect data\).

But it doesn’t end here.

Our pipeline is just one of the first pieces of the puzzle. To develop a robust enough infrastructure to support a true digital twin of the physical world, a lot more needs to be done. This includes creating algorithms that can turn this raw depth data into real, usable applications.

### How to Contribute

We’re always excited to work with like-minded people, and invite you to experiment with our pipeline however you like! If you enjoy our work and think you can help take this project to the next level, feel free to drop us a message on [olenka@moetsi.com](mailto:olenka@moetsi.com) to get involved.

If you happen to discover any bugs in our code, we’d really appreciate knowing about them. Please just create an issue here on GitHub.

In terms of related projects that fall outside of this repo’s scope, we’d be super excited to see, and think the community could benefit from development on:

#### **More devices**

Working with the Kinect v2, and other sensors such as the Structure Core sensor.

#### **Integrations**

Any other sort of output that you can imagine!

#### **Encoding**

Improve encoding performance on AMD/Intel graphic cards, by using the AMD Media Codec/Intel Quick Sync Video instead of relying on libav \(VAAPI or OpenCK\) for hardware accelerated encoding. Feel free to do the same for Intel cards using Intel Quick Sync Video too!

### Moetsi’s Permissive License

Moetsi’s Sensor Stream Pipe is licensed under the MIT license. That means that we don’t require attribution, but we’d really like to know what cool things you’re using our pipe for. Drop us a message on [olenka@moetsi.com](mailto:olenka@moetsi.com) or post on our [forum](https://moetsi.com/pages/community) to tell us all about it!

### Support Moetsi!

Our Sensor Stream Pipe is always going to be free, but it has taken a lot of blood, sweat and tears to get to this point. If you love what we’ve made, please consider reaching out to [olenka@moetsi.com](mailto:olenka@moetsi.com).

### Authors

* **André Mourão** - [amourao](https://github.com/amourao)
* **Olenka Polak** - [olenkapolak](https://github.com/olenkapolak)
* **Adam Polak** - [adammpolak](https://github.com/adammpolak)

