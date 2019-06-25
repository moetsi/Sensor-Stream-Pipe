#include <iostream>
#include <fstream>
#include <vector>

std::vector<unsigned char> readFile(const char* filename)
{
    // open the file
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    // get its size
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data
    std::vector<unsigned char> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}

int main (int argc, char *argv[]){
    std::vector<unsigned char> fileData = readFile(argv[1]);
    std::cout << fileData.size() << std::endl;
}

