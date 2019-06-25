#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h> 

#include <asio.hpp>

#include "FrameStruct.hpp"

using asio::ip::tcp;

std::string make_daytime_string()
{
  using namespace std;  
  time_t now = time(0);
  return ctime(&now);
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
      tcp::socket socket(io_context);
      acceptor.accept(socket);

      std::string message = make_daytime_string();

      asio::error_code ignored_error;
      asio::write(socket, asio::buffer(message), ignored_error);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
