# DAVP: Distributed Architecture for Video-stream Processing

The goal of DAVP is to efficiently aggregate and process multiple of video streams from a set of diverse set of devices (smartphones, cameras, ...).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

OpenCV 3.x, probably 4.x (tested on version available on 18.04 repo, 3.2.0)

Asio 1.13 (headers only lib)

cereal 1.2.2 (headers only lib)

Tested on Ubuntu 18.04.2

### Installing dependencies

Install OpenCV

```
sudo apt install libopencv-dev libopencv-core-dev
```

Download and extract header libraries

```
wget https://codeload.github.com/chriskohlhoff/asio/zip/asio-1-13-0
unzip asio-1-13-0
cp -r asio-asio-1-13-0/asio(include/ ~/libs
```

```
wget https://codeload.github.com/USCiLab/cereal/tar.gz/v1.2.2
tar xf v1.2.2
cp -r cereal-1.2.2/include ~/libs
```

## ZeroMQ


### libzmq3 4.3.1

```
wget https://github.com/zeromq/libzmq/releases/download/v4.3.1/zeromq-4.3.1.tar.gz
tar xf zeromq-4.3.1.tar.gz
cd zeromq-4.3.1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/amourao/libs
make install
```

### cppzmq 4.3.0

```
wget https://github.com/zeromq/cppzmq/archive/v4.3.0.tar.gz
tar xf v4.3.0.tar.gz
cd cppzmq-4.3.0
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/amourao/libs
make install
```

### Building


Add header libraries to search path

```
export CPATH=/home/amourao/libs/include:$CPATH
export C_INCLUDE_PATH =/home/amourao/libs/include:$C_INCLUDE_PATH 
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

To stream frames on loop on a single node, run the following commands on two terminal windows:

```
# ./server localhost 9999 apt-0.txt
Frame 	0 sent, took 0 ms; size 161087; avg inf fps; inf Mbps
Frame 	1 sent, took 35 ms; size 162913; avg 57 fps; 74.0571 Mbps
Frame 	2 sent, took 32 ms; size 159411; avg 44 fps; 57.7207 Mbps
Frame 	3 sent, took 33 ms; size 162808; avg 40 fps; 51.6975 Mbps
Frame 	4 sent, took 34 ms; size 162200; avg 37 fps; 48.2638 Mbps
Frame 	5 sent, took 33 ms; size 160750; avg 35 fps; 46.4273 Mbps
Frame 	6 sent, took 33 ms; size 159583; avg 35 fps; 45.1501 Mbps
Frame 	7 sent, took 34 ms; size 159952; avg 34 fps; 44.0583 Mbps
Frame 	8 sent, took 33 ms; size 162306; avg 33 fps; 43.476 Mbps
Frame 	9 sent, took 34 ms; size 161530; avg 33 fps; 42.8582 Mbps
Frame 	10 sent, took 33 ms; size 161508; avg 32 fps; 42.4922 Mbps
.....
```

Client receiving multiple parallel streams.

```
# ./client 9999
Frame 907 received, took 23 ms; size 171181; avg 49 fps
Frame 967 received, took 11 ms; size 156648; avg 50 fps
Frame 908 received, took 22 ms; size 171891; avg 49 fps
Frame 968 received, took 10 ms; size 153730; avg 50 fps
Frame 909 received, took 22 ms; size 166073; avg 50 fps
Frame 969 received, took 11 ms; size 152208; avg 50 fps
Frame 910 received, took 21 ms; size 165463; avg 50 fps
Frame 970 received, took 13 ms; size 152834; avg 50 fps
Frame 911 received, took 21 ms; size 168747; avg 50 fps
Frame 971 received, took 11 ms; size 150640; avg 50 fps
Frame 912 received, took 22 ms; size 167307; avg 50 fps
Frame 972 received, took 11 ms; size 150720; avg 50 fps
Frame 913 received, took 23 ms; size 165487; avg 50 fps
Frame 973 received, took 11 ms; size 149879; avg 50 fps
Frame 914 received, took 22 ms; size 164491; avg 50 fps
Frame 974 received, took 10 ms; size 147178; avg 50 fps
Frame 915 received, took 23 ms; size 168506; avg 50 fps
```

### Generating frame list file

See the [davp-data-scripts](https://github.com/moetsi/davp-data-scripts) to see how to generate files for the 

## Notes

Current protocol is very fast and resilient to network failures.
The main problem are the network requirements: 40 Mbps for the Bundle Fusion dataset at 30Hz.

## Built With

* [ASIO](https://think-async.com/Asio/) - Network and low-level I/O programming
* [Cereal](https://uscilab.github.io/cereal/) - Serialization library
* [OpenCV](https://opencv.org/) - Turn frames to cv::Mat

## Authors

* **André Mourão** - *Initial work* - [amourao](https://github.com/amourao)

