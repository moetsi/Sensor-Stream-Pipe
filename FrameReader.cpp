//
// Created by amourao on 26-06-2019.
//

#include "FrameReader.h"

std::vector<unsigned char> readFile(const char *filename) {
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> fileData(fileSize);
    file.read((char *) &fileData[0], fileSize);
    return fileData;
}

FrameStruct createFrameStruct(const char *filename1, const char *filename2) {
    std::vector<unsigned char> colorFileData = readFile(filename1);
    std::vector<unsigned char> depthFileData = readFile(filename2);
    FrameStruct frame = FrameStruct();
    frame.colorFrame = colorFileData;
    frame.depthFrame = depthFileData;
    return frame;
}

std::string getExampleFrameStructBytes() {
    FrameStruct frame = createFrameStruct("/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg",
                                          "/home/amourao/data/bundle_fusion/apt0/frame-000000.depth.png");
    std::ostringstream os(std::ios::binary);

    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(frame);
    }

    return os.str();

}

FrameStruct parseFrameStruct(std::string &data) {
    FrameStruct frameIn;
    std::istringstream is(data, std::ios::binary);
    {
        cereal::BinaryInputArchive iarchive(is);
        iarchive(frameIn);
    }
    return frameIn;
}


FrameStruct parseFrameStruct(std::vector<unsigned char> &data, size_t dataSize) {
    FrameStruct frameIn;
    std::istringstream is(std::string(data.begin(), data.begin() + dataSize), std::ios::binary);
    {
        cereal::BinaryInputArchive iarchive(is);
        iarchive(frameIn);
    }
    return frameIn;
}


FrameStruct parseFrameStruct(asio::streambuf &data) {
    FrameStruct frameIn;

    std::istream is(&data);
    {
        cereal::BinaryInputArchive iarchive(is);
        iarchive(frameIn);
    }
    return frameIn;
}
