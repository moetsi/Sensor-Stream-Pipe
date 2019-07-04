# DAVP: Distributed Architecture for Video-stream Processing

The goal of DAVP is to efficiently aggregate and process multiple of video streams from a set of diverse set of devices (smartphones, cameras, ...).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

OpenCV 3.x, probably 4.x (tested on version available on 18.04 repo, 3.2.0)

Asio 1.13 (headers only lib)

cereal 1.2.2 (headers only lib)

zeroMQ (libzmq3 4.3.1, cppzmq 4.3.0)

Tested on Ubuntu 18.04.2

### Installing dependencies

Install OpenCV

```
sudo apt install libopencv-dev libopencv-core-dev uuid-dev
```

## Download and extract "out-of-repo" libraries


First, create a folder where local libs are to be installed:

```
mkdir ~/libs
mkdir ~/libs/srcOriginal
```

### ~~ASIO (outdated, now using ZeroMQ)~~

```
cd ~/libs/srcOriginal
wget https://codeload.github.com/chriskohlhoff/asio/zip/asio-1-13-0
unzip asio-1-13-0
cp -r asio-asio-1-13-0/asio/include/ ~/libs
```

### Cereal

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

### Building


~~Add header libraries to search path~~ (not necessary; part of the CMakeFile)

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

The project is divided into two components:
» server: reads frames on disk and sends them over the network
» client: receives network frames and decodes them into cv::Mat

```
server <host> <port> <frame list file (see below how to generate a file)> <stop after (optional, number of times to stream the full frame set, defaults to MAX_INT)>
client <port>
```

The output for the server and client follows this format:

```
<device id>;<sensor id>;<frame id> sent/received, took <time to frame> ms; size <packet size in bytes>; avg <avg fps since first frame sent/received> fps; <avg bandwidth since first frame sent/received> Mbps
```

### Example: Stream six parallel frame streams to a client

**Terminal 1:**

Run the frame processing client 

```
./client 9999
``` 

**Terminal 2:**

Start the frame streaming devices:

```
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-01-frames.txt &
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-02-frames.txt &
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-03-frames.txt &
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-04-frames.txt &
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-05-frames.txt &
./server localhost 9999 ~/data/ms_rgbd_7s/stairs-seq-06-frames.txt &
```

**Real output running on a 4 core i5-2500k at 4.4 GHz**


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
# ./client 9999
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


### Generating frame list file

See the [davp-data-scripts](https://github.com/moetsi/davp-data-scripts) to see how to generate files for the 

## Notes


Current network protocol is fast, resilient to network failures and can deal with parallel streams.

### MS RGB-D 7-Scenes

Initial results show that a single thread takes **6-7 ms** to transform image bytes into a cv::Mat.

This means it takes about **12-14 ms** per FrameStruct, restricting to about **75-85 fps** per thread just to transform two images into cv::Mat.  

**75-85 fps** restricts it to a little below **2.5-2.8** streams per server.

### BundleFusion

BundleFusion results are much better.
Initial results show that a single thread takes **2-3 ms** to transform image bytes into a cv::Mat.

This means it takes about **4-5 ms** per FrameStruct, restricting to about **200 fps** per thread to transform two images into cv::Mat.  

**200 fps** allows about **6.6** streams per server.

This difference in performance is cause by:
 * Frame format: BundleFusion uses **JPG** for color frames, while MS RGB-D 7-Scenes uses **PNG**. 
 * Bandwidth requirements: **PNG** images are much larger, meaning that the amount of data pushed is much higher: **33 Mbps** vs **125 Mbps**.
 
 Additional experiments must be performed to check which factor is more important and how they can be mitigated.

### Additional notes

Note that the thread is also dealing with network communication, meaning that there may be some slight boosts by decomposing the client app into more threads.

In addition, network requirements are also high: **33 Mbps** for the Bundle Fusion dataset and **125 Mbps** for the MS RGB-D 7-Scenes at **30Hz**.  

## Built With

* [ASIO](https://think-async.com/Asio/) - ~~Network and low-level I/O programming~~ (replaced by **ZeroMQ**)
* [ZeroMQ](http://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq) - Network and low-level I/O programming
* [Cereal](https://uscilab.github.io/cereal/) - Serialization library
* [OpenCV](https://opencv.org/) - Turn frames to cv::Mat

## Authors

* **André Mourão** - *Initial work* - [amourao](https://github.com/amourao)

