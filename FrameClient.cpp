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
#include "FrameReader.cpp"

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
      asio::error_code error = asio::error::host_unreachable;
      asio::streambuf streamBuffer;
      asio::streambuf::mutable_buffers_type mutableBuffer =
              streamBuffer.prepare(1610610);
    for (;;)
    {
        while(error)
            asio::connect(socket, endpoints, error);

      for (;;) {
          size_t len = socket.read_some(asio::buffer(mutableBuffer), error);
          streamBuffer.commit(len);

          if (error == asio::error::eof) {
              std::cout << "EOF, Received " << len << " bytes " << streamBuffer.size() << std::endl;
              FrameStruct f = parseFrameStruct(streamBuffer);
              //streamBuffer.consume(streamBuffer.in_avail());
              break; // Connection closed cleanly by peer.
          } else if (error)
          {
              throw asio::system_error(error); // Some other error.
          }
      }


    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
