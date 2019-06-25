
all:
	g++ -o reader FrameReader.cpp
	g++ -pthread -o client FrameClient.cpp
	g++ -pthread -o server FrameServer.cpp

