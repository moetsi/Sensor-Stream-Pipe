#include <iostream>
#include <fstream>
#include <vector>

#include <cereal/archives/binary.hpp>
#include "FrameStruct.hpp"

std::vector<unsigned char> readFile(const char* filename)
{
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}



int main (int argc, char *argv[])
{

    if (argc != 3)
    {
        std::cerr << "Usage: reader <file1> <file2>" << std::endl;
        return 1;
    }

    std::vector<unsigned char> colorFileData = readFile(argv[1]);
    std::vector<unsigned char> depthFileData = readFile(argv[2]);
    FrameStruct frame = FrameStruct();
    frame.colorFrame = colorFileData;
    frame.depthFrame = depthFileData;
    std::cout << sizeof(frame) << std::endl;

    std::ofstream os("out.cereal", std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive(frame);
}

