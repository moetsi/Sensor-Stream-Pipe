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
cp -r asio-asio-1-13-0/asio/include/ ~/libs
```

```
wget https://codeload.github.com/USCiLab/cereal/tar.gz/v1.2.2
tar xf v1.2.2
cp -r cereal-1.2.2/include ~/libs
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
server <port> <frame list file (see below how to generate a file)> <stop after (optional, number of times to stream the full frame set, defaults to MAX_INT)>
client <host> <port>
```

To stream frames on loop on a single node, run the following commands on two terminal windows:

```
# ./server 9999 apt-0.txt
Frame 0 sent, took 1 ms; size 161087; avg 1000 fps
Frame 1 sent, took 37 ms; size 162913; avg 52 fps
Frame 2 sent, took 34 ms; size 159411; avg 41 fps
Frame 3 sent, took 34 ms; size 162808; avg 37 fps
Frame 4 sent, took 34 ms; size 162200; avg 35 fps
Frame 5 sent, took 34 ms; size 160750; avg 34 fps
Frame 6 sent, took 34 ms; size 159583; avg 33 fps
Frame 7 sent, took 34 ms; size 159952; avg 32 fps
Frame 8 sent, took 35 ms; size 162306; avg 32 fps
Frame 9 sent, took 35 ms; size 161530; avg 31 fps
.....
```

```
# ./client localhost 9999
Frame 0 received, took 2 ms; size 161087; avg 500 fps
Frame 1 received, took 36 ms; size 162913; avg 52 fps
Frame 2 received, took 34 ms; size 159411; avg 41 fps
Frame 3 received, took 34 ms; size 162808; avg 37 fps
Frame 4 received, took 34 ms; size 162200; avg 35 fps
Frame 5 received, took 35 ms; size 160750; avg 34 fps
Frame 6 received, took 35 ms; size 159583; avg 33 fps
Frame 7 received, took 33 ms; size 159952; avg 32 fps
Frame 8 received, took 35 ms; size 162306; avg 32 fps
Frame 9 received, took 41 ms; size 161530; avg 31 fps
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

