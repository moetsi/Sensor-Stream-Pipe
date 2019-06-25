//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2019 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <asio.hpp>

#include <chrono>
#include <thread>

#include "FrameStruct.hpp"

using asio::ip::tcp;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: client <host> <port>" << std::endl;
      return 1;
    }

    asio::io_context io_context;

    tcp::resolver resolver(io_context);
    tcp::resolver::results_type endpoints =
      resolver.resolve(argv[1], argv[2]);

    tcp::socket socket(io_context);

    for (;;)
    {
      asio::connect(socket, endpoints);

      for (;;)
      {
        std::vector<unsigned char> buf(256);
        asio::error_code error;

        size_t len = socket.read_some(asio::buffer(buf), error);

        if (error == asio::error::eof)
          break; // Connection closed cleanly by peer.
        else if (error)
          throw asio::system_error(error); // Some other error.

        for (uint i = 0; i < buf.size(); i++)
          std::cout << buf[i];
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
