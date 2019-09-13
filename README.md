# DAVP: Distributed Architecture for Video-stream Processing

The goal of DAVP is to efficiently aggregate and process multiple of video streams from a set of diverse set of devices (smartphones, cameras, ...).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* OpenCV 3.x, probably 4.x (tested on version available on 18.04 repo, 3.2.0)
* Asio 1.13 (headers only lib)
* cereal 1.2.2 (headers only lib)
* zeroMQ (libzmq3 4.3.1, cppzmq 4.3.0)
* yaml-cpp 0.6.0

Tested on Ubuntu 18.04.2

### Installing dependencies from repo

#### OpenCV 3.2.0

```
sudo apt install libopencv-dev libopencv-core-dev uuid-dev
```

#### Azure Kinect SDK 1.2


Add the Linux Software Repository for Microsoft Products.

Note: Run any command as sudo to avoid getting a password prompt
```
curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
```

Install Azure Kinect SDK 1.2
```
sudo apt install libk4a1.2 libk4a1.2-dev k4a-tools
```

To be able to use the Kinect as non-root, please run the following:
```
wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules
sudo cp 99-k4a.rules /etc/udev/rules.d/
```

On my installation, the link to the canonical version of the depth lib was missing.
You can create it by running the following command:

```
sudo ln -s /usr/lib/x86_64-linux-gnu/libdepthengine.so.1.0 /usr/lib/x86_64-linux-gnu/libdepthengine.so
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

### Building


Add header libraries to search path

```
export CPATH=~/libs/include:$CPATH
export C_INCLUDE_PATH =~/libs/include:$C_INCLUDE_PATH 
```


Download and build the project

```
git clone git@github.com:moetsi/davp-baseline.git
cd davp-baseline
cmake .
make
```

## Running

The project is divided into two types of components:
* servers: read frames on disk and sends them over the network
* clients: receive network frames and decodes them into cv::Mat

### Client

The client can handle all types for frames (color, depth) for all formats (images and packets):

```
client <port>
```


### Servers:


The project is divided into two types of components:
* frame_server: sends frames over the network in their original format (png, jpg)
* video_file_server: sends encoded frames from a previously encoded video
* video_encoder_server: encodes RAW png or jpg images into video live and sends them over the network


```
./bin/frame_server <host> <port> <frame list file (see below how to generate a file)> <stop after (optional, number of times to stream the full frame set, defaults to MAX_INT)>
./bin/video_file_server <host> <port> <video file> <stop after (optional, number of times to stream the full frame set, defaults to MAX_INT)>
./bin/video_encoder_server <host> <port> <frame list file (see below how to generate a file)> <video parameters (see below)> <stop after (optional, number of times to stream the full frame set, defaults to MAX_INT)>
```

#### Video parametrization format

This is a [YAML](https://en.wikipedia.org/wiki/YAML) file that contains the parameters for libAV live video encoding (codec, bitrate, pixel format, I-B-P frame ratio, ...):

```
codec_name: "libx264"
bitrate: 1960000
gop_size: 10
max_b_frames: 1
pix_fmt: "yuv420p"
```

### Example 1: Stream six parallel raw frame streams to a client

The output for the server and client follows this format:

```
<device id>;<sensor id>;<frame id> sent/received, took <time to frame> ms; size <packet size in bytes>; avg <avg fps since first frame sent/received> fps; <avg bandwidth since first frame sent/received> Mbps
```


**Terminal 1:**

Run the frame processing client 

```
./bin/client 9999
``` 

**Terminal 2:**

Start the frame streaming devices:

```
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-01-frames-color.txt &
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-02-frames-color.txt &
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-03-frames-color.txt &
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-04-frames-color.txt &
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-05-frames-color.txt &
./bin/frame_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-06-frames-color.txt &
```

**Real frame_server output running on a 4 core i5-2500k at 4.4 GHz**


Server sending a stream.

```
# ./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-01-frames.txt
1;0;0 sent, took 2 ms; size 494433; avg 500 fps; 1977.73 Mbps
1;0;1 sent, took 32 ms; size 493917; avg 58 fps; 232.553 Mbps
1;0;2 sent, took 33 ms; size 493629; avg 44 fps; 176.953 Mbps
1;0;3 sent, took 33 ms; size 492412; avg 40 fps; 157.951 Mbps
1;0;4 sent, took 33 ms; size 493412; avg 37 fps; 148.439 Mbps
1;0;5 sent, took 33 ms; size 493059; avg 36 fps; 142.692 Mbps
1;0;6 sent, took 34 ms; size 493019; avg 35 fps; 138.155 Mbps
1;0;7 sent, took 33 ms; size 492996; avg 34 fps; 135.515 Mbps
1;0;8 sent, took 34 ms; size 494131; avg 33 fps; 133.064 Mbps
1;0;9 sent, took 34 ms; size 493278; avg 33 fps; 131.144 Mbps
1;0;10 sent, took 32 ms; size 493279; avg 33 fps; 130.392 Mbps
.....
```

Client receiving six parallel streams.

```
# ./bin/client 9999
.....
2;0;232 received, took 13 ms; size 504917; avg 77 fps; 312.452 Mbps
3;0;232 received, took 11 ms; size 458153; avg 77 fps; 312.448 Mbps
4;0;232 received, took 12 ms; size 490571; avg 77 fps; 312.439 Mbps
6;0;232 received, took 13 ms; size 478814; avg 77 fps; 312.444 Mbps
5;0;232 received, took 13 ms; size 501322; avg 77 fps; 312.423 Mbps
1;0;213 received, took 13 ms; size 537060; avg 77 fps; 312.436 Mbps
2;0;233 received, took 13 ms; size 503478; avg 77 fps; 312.469 Mbps
3;0;233 received, took 11 ms; size 457584; avg 77 fps; 312.464 Mbps
4;0;233 received, took 12 ms; size 489986; avg 77 fps; 312.456 Mbps
.....
```

### Example 2: Stream six parallel raw frame streams to a client


**Terminal 1:**

Run the frame processing client

```
./bin/client 9999
```

**Terminal 2:**

Start the frame streaming devices:

```
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-01-frames-color.txt config/color_h264.yaml &
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-02-frames-color.txt config/color_h264.yaml &
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-03-frames-color.txt config/color_h264.yaml &
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-04-frames-color.txt config/color_h264.yaml &
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-05-frames-color.txt config/color_h264.yaml &
./bin/video_encoder_server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-06-frames-color.txt config/color_h264.yaml &
```

### Generating frame list file

See the [davp-data-scripts](https://github.com/moetsi/davp-data-scripts) to see how to generate the frame files.

## Notes:

### Frame Client

#### MS RGB-D 7-Scenes

Initial results show that a single thread takes **6-7 ms** to transform image bytes into a cv::Mat.

This means it takes about **12-14 ms** per FrameStruct, restricting to about **75-85 fps** per thread just to transform two images into cv::Mat.  

**75-85 fps** restricts it to a little below **2.5-2.8** streams per thread.

#### BundleFusion

BundleFusion results are much better.
Initial results show that a single thread takes **2-3 ms** to transform image bytes into a cv::Mat.

This means it takes about **4-5 ms** per FrameStruct, restricting to about **200 fps** per thread to transform two images into cv::Mat.  

**200 fps** allows about **6.6** streams per thread.

This difference in performance is cause by:
 * Frame format: BundleFusion uses **JPG** for color frames, while MS RGB-D 7-Scenes uses **PNG**. 
 * Bandwidth requirements: **PNG** images are much larger, meaning that the amount of data pushed is much higher: **33 Mbps** vs **125 Mbps**.
 
 Additional experiments must be performed to check which factor is more important and how they can be mitigated.

### Additional notes

Note that the thread is also dealing with network communication, meaning that there may be some slight boosts by decomposing the client app into more threads.

In addition, network requirements are also high: **33 Mbps** for the Bundle Fusion dataset and **125 Mbps** for the MS RGB-D 7-Scenes at **30Hz**.  

## Built With

* [ZeroMQ](http://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq) - Network and low-level I/O programming
* [Cereal](https://uscilab.github.io/cereal/) - Serialization library
* [OpenCV](https://opencv.org/) - Turn frames to cv::Mat
* [libav](https://github.com/libav/libav/) - Encodes and decodes images into video
* [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) - Access Azure Kinect DK

## Authors

* **André Mourão** - [amourao](https://github.com/amourao)



