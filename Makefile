
all:
	g++ -o bin/reader FrameReader.cpp
	g++ -pthread -o bin/client FrameClient.cpp
	g++ -pthread -o bin/server FrameServer.cpp

