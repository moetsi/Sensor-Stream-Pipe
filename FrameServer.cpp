#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h> 

#include <asio.hpp>

#include "FrameStruct.hpp"
#include "FrameReader.cpp"

using asio::ip::tcp;

std::string make_daytime_string()
{
  using namespace std;  
  time_t now = time(0);
  return ctime(&now);
}


std::string get_frame_message()
{
    return getExampleFrameStructBytes();
}

int main(int argc, char* argv[])
{

  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: server <port>" << std::endl;
      return 1;
    }

    asio::io_context io_context;

    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), atoi(argv[1])));

    for (;;)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000/30));
      tcp::socket socket(io_context);
      acceptor.accept(socket);

      std::string message = get_frame_message();
      std::cout << "Sent " << message.size() << " bytes" << std::endl;

      asio::error_code ignored_error;
      asio::write(socket, asio::buffer(message), ignored_error);
      socket.close();

    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
