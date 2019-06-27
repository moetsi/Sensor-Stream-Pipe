//
// Created by amourao on 27-06-2019.
//

#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <asio.hpp>
#include <cereal/archives/binary.hpp>

#include "FrameStruct.hpp"

std::vector<unsigned char> readFile(const char *filename);

FrameStruct createFrameStruct(const char *filename1, const char *filename2);

std::string getExampleFrameStructBytes();

FrameStruct parseFrameStruct(std::string &data);
FrameStruct parseFrameStruct(std::vector<unsigned char> &data, size_t dataSize);
FrameStruct parseFrameStruct(asio::streambuf &data);

