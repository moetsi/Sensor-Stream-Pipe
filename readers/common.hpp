// Copyright (C) 2018-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

/**
 * @brief a header file with common samples functionality
 * @file common.hpp
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

using std::setprecision;

// clang-format off
#include "openvino/openvino.hpp"
#include "slog.hpp"
// clang-format on

// @brief performance counters sort
static constexpr char pcSort[] = "sort";
static constexpr char pcNoSort[] = "no_sort";
static constexpr char pcSimpleSort[] = "simple_sort";

#ifndef UNUSED
#    if defined(_MSC_VER) && !defined(__clang__)
#        define UNUSED
#    else
#        define UNUSED __attribute__((unused))
#    endif
#endif

/**
 * @brief Unicode string wrappers
 */
#if defined(ENABLE_UNICODE_PATH_SUPPORT) && defined(_WIN32)
#    define tchar                wchar_t
#    define tstring              std::wstring
#    define tmain                wmain
#    define TSTRING2STRING(tstr) wstring2string(tstr)
#else
#    define tchar                char
#    define tstring              std::string
#    define tmain                main
#    define TSTRING2STRING(tstr) tstr
#endif

#if defined(ENABLE_UNICODE_PATH_SUPPORT) && defined(_WIN32)

/**
 * @brief Convert wstring to string
 * @param ref on wstring
 * @return string
 */
inline std::string wstring2string(const std::wstring& wstr) {
    std::string str;
    for (auto&& wc : wstr)
        str += static_cast<char>(wc);
    return str;
}
#endif

/**
 * @brief trim from start (in place)
 * @param s - string to trim
 */
inline void ltrim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int c) {
                return !std::isspace(c);
            }));
}

/**
 * @brief trim from end (in place)
 * @param s - string to trim
 */
inline void rtrim(std::string& s) {
    s.erase(std::find_if(s.rbegin(),
                         s.rend(),
                         [](int c) {
                             return !std::isspace(c);
                         })
                .base(),
            s.end());
}

/**
 * @brief trim from both ends (in place)
 * @param s - string to trim
 */
inline std::string& trim(std::string& s) {
    ltrim(s);
    rtrim(s);
    return s;
}
/**
 * @brief Gets filename without extension
 * @param filepath - full file name
 * @return filename without extension
 */
inline std::string fileNameNoExt(const std::string& filepath) {
    auto pos = filepath.rfind('.');
    if (pos == std::string::npos)
        return filepath;
    return filepath.substr(0, pos);
}

/**
 * @brief Get extension from filename
 * @param filename - name of the file which extension should be extracted
 * @return string with extracted file extension
 */
inline std::string fileExt(const std::string& filename) {
    auto pos = filename.rfind('.');
    if (pos == std::string::npos)
        return "";
    return filename.substr(pos + 1);
}

inline slog::LogStream& operator<<(slog::LogStream& os, const ov::Version& version) {
    os << "Build ................................. ";
    os << version.buildNumber << slog::endl;

    return os;
}

inline slog::LogStream& operator<<(slog::LogStream& os, const std::map<std::string, ov::Version>& versions) {
    for (auto&& version : versions) {
        os << version.first << slog::endl;
        os << version.second << slog::endl;
    }

    return os;
}

/**
 * @class Color
 * @brief A Color class stores channels of a given color
 */
class Color {
private:
    unsigned char _r;
    unsigned char _g;
    unsigned char _b;

public:
    /**
     * A default constructor.
     * @param r - value for red channel
     * @param g - value for green channel
     * @param b - value for blue channel
     */
    Color(unsigned char r, unsigned char g, unsigned char b) : _r(r), _g(g), _b(b) {}

    inline unsigned char red() {
        return _r;
    }

    inline unsigned char blue() {
        return _b;
    }

    inline unsigned char green() {
        return _g;
    }
};

// TODO : keep only one version of writeOutputBMP

/**
 * @brief Writes output data to image
 * @param name - image name
 * @param data - output data
 * @param classesNum - the number of classes
 * @return false if error else true
 */
static UNUSED void writeOutputBmp(std::vector<std::vector<size_t>> data, size_t classesNum, std::ostream& outFile) {
    unsigned int seed = (unsigned int)time(NULL);
    // Known colors for training classes from Cityscape dataset
    static std::vector<Color> colors = {
        {128, 64, 128}, {232, 35, 244}, {70, 70, 70},   {156, 102, 102}, {153, 153, 190}, {153, 153, 153},
        {30, 170, 250}, {0, 220, 220},  {35, 142, 107}, {152, 251, 152}, {180, 130, 70},  {60, 20, 220},
        {0, 0, 255},    {142, 0, 0},    {70, 0, 0},     {100, 60, 0},    {90, 0, 0},      {230, 0, 0},
        {32, 11, 119},  {0, 74, 111},   {81, 0, 81}};

    while (classesNum > colors.size()) {
        static std::mt19937 rng(seed);
        std::uniform_int_distribution<int> dist(0, 255);
        Color color(dist(rng), dist(rng), dist(rng));
        colors.push_back(color);
    }

    unsigned char file[14] = {
        'B',
        'M',  // magic
        0,
        0,
        0,
        0,  // size in bytes
        0,
        0,  // app data
        0,
        0,  // app data
        40 + 14,
        0,
        0,
        0  // start of data offset
    };
    unsigned char info[40] = {
        40,   0,    0, 0,  // info hd size
        0,    0,    0, 0,  // width
        0,    0,    0, 0,  // height
        1,    0,           // number color planes
        24,   0,           // bits per pixel
        0,    0,    0, 0,  // compression is none
        0,    0,    0, 0,  // image bits size
        0x13, 0x0B, 0, 0,  // horz resolution in pixel / m
        0x13, 0x0B, 0, 0,  // vert resolution (0x03C3 = 96 dpi, 0x0B13 = 72 dpi)
        0,    0,    0, 0,  // #colors in palette
        0,    0,    0, 0,  // #important colors
    };

    auto height = data.size();
    auto width = data.at(0).size();

    OPENVINO_ASSERT(
        height < (size_t)std::numeric_limits<int32_t>::max && width < (size_t)std::numeric_limits<int32_t>::max,
        "File size is too big: ",
        height,
        " X ",
        width);

    int padSize = static_cast<int>(4 - (width * 3) % 4) % 4;
    int sizeData = static_cast<int>(width * height * 3 + height * padSize);
    int sizeAll = sizeData + sizeof(file) + sizeof(info);

    file[2] = (unsigned char)(sizeAll);
    file[3] = (unsigned char)(sizeAll >> 8);
    file[4] = (unsigned char)(sizeAll >> 16);
    file[5] = (unsigned char)(sizeAll >> 24);

    info[4] = (unsigned char)(width);
    info[5] = (unsigned char)(width >> 8);
    info[6] = (unsigned char)(width >> 16);
    info[7] = (unsigned char)(width >> 24);

    int32_t negativeHeight = -(int32_t)height;
    info[8] = (unsigned char)(negativeHeight);
    info[9] = (unsigned char)(negativeHeight >> 8);
    info[10] = (unsigned char)(negativeHeight >> 16);
    info[11] = (unsigned char)(negativeHeight >> 24);

    info[20] = (unsigned char)(sizeData);
    info[21] = (unsigned char)(sizeData >> 8);
    info[22] = (unsigned char)(sizeData >> 16);
    info[23] = (unsigned char)(sizeData >> 24);

    outFile.write(reinterpret_cast<char*>(file), sizeof(file));
    outFile.write(reinterpret_cast<char*>(info), sizeof(info));

    unsigned char pad[3] = {0, 0, 0};

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            unsigned char pixel[3];
            size_t index = data.at(y).at(x);
            pixel[0] = colors.at(index).red();
            pixel[1] = colors.at(index).green();
            pixel[2] = colors.at(index).blue();
            outFile.write(reinterpret_cast<char*>(pixel), 3);
        }
        outFile.write(reinterpret_cast<char*>(pad), padSize);
    }
}

/**
 * @brief Writes output data to BMP image
 * @param name - image name
 * @param data - output data
 * @param height - height of the target image
 * @param width - width of the target image
 * @return false if error else true
 */
static UNUSED bool writeOutputBmp(std::string name, unsigned char* data, size_t height, size_t width) {
    std::ofstream outFile;
    outFile.open(name, std::ofstream::binary);
    if (!outFile.is_open()) {
        return false;
    }

    unsigned char file[14] = {
        'B',
        'M',  // magic
        0,
        0,
        0,
        0,  // size in bytes
        0,
        0,  // app data
        0,
        0,  // app data
        40 + 14,
        0,
        0,
        0  // start of data offset
    };
    unsigned char info[40] = {
        40,   0,    0, 0,  // info hd size
        0,    0,    0, 0,  // width
        0,    0,    0, 0,  // height
        1,    0,           // number color planes
        24,   0,           // bits per pixel
        0,    0,    0, 0,  // compression is none
        0,    0,    0, 0,  // image bits size
        0x13, 0x0B, 0, 0,  // horz resolution in pixel / m
        0x13, 0x0B, 0, 0,  // vert resolution (0x03C3 = 96 dpi, 0x0B13 = 72 dpi)
        0,    0,    0, 0,  // #colors in palette
        0,    0,    0, 0,  // #important colors
    };

    OPENVINO_ASSERT(
        height < (size_t)std::numeric_limits<int32_t>::max && width < (size_t)std::numeric_limits<int32_t>::max,
        "File size is too big: ",
        height,
        " X ",
        width);

    int padSize = static_cast<int>(4 - (width * 3) % 4) % 4;
    int sizeData = static_cast<int>(width * height * 3 + height * padSize);
    int sizeAll = sizeData + sizeof(file) + sizeof(info);

    file[2] = (unsigned char)(sizeAll);
    file[3] = (unsigned char)(sizeAll >> 8);
    file[4] = (unsigned char)(sizeAll >> 16);
    file[5] = (unsigned char)(sizeAll >> 24);

    info[4] = (unsigned char)(width);
    info[5] = (unsigned char)(width >> 8);
    info[6] = (unsigned char)(width >> 16);
    info[7] = (unsigned char)(width >> 24);

    int32_t negativeHeight = -(int32_t)height;
    info[8] = (unsigned char)(negativeHeight);
    info[9] = (unsigned char)(negativeHeight >> 8);
    info[10] = (unsigned char)(negativeHeight >> 16);
    info[11] = (unsigned char)(negativeHeight >> 24);

    info[20] = (unsigned char)(sizeData);
    info[21] = (unsigned char)(sizeData >> 8);
    info[22] = (unsigned char)(sizeData >> 16);
    info[23] = (unsigned char)(sizeData >> 24);

    outFile.write(reinterpret_cast<char*>(file), sizeof(file));
    outFile.write(reinterpret_cast<char*>(info), sizeof(info));

    unsigned char pad[3] = {0, 0, 0};

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            unsigned char pixel[3];
            pixel[0] = data[y * width * 3 + x * 3];
            pixel[1] = data[y * width * 3 + x * 3 + 1];
            pixel[2] = data[y * width * 3 + x * 3 + 2];

            outFile.write(reinterpret_cast<char*>(pixel), 3);
        }
        outFile.write(reinterpret_cast<char*>(pad), padSize);
    }
    return true;
}

/**
 * @brief Adds colored rectangles to the image
 * @param data - data where rectangles are put
 * @param height - height of the rectangle
 * @param width - width of the rectangle
 * @param rectangles - vector points for the rectangle, should be 4x compared to num classes
 * @param classes - vector of classes
 * @param thickness - thickness of a line (in pixels) to be used for bounding boxes
 */
static UNUSED void addRectangles(unsigned char* data,
                                 size_t height,
                                 size_t width,
                                 std::vector<int> rectangles,
                                 std::vector<int> classes,
                                 int thickness = 1) {
    std::vector<Color> colors = {// colors to be used for bounding boxes
                                 {128, 64, 128},  {232, 35, 244}, {70, 70, 70},  {156, 102, 102}, {153, 153, 190},
                                 {153, 153, 153}, {30, 170, 250}, {0, 220, 220}, {35, 142, 107},  {152, 251, 152},
                                 {180, 130, 70},  {60, 20, 220},  {0, 0, 255},   {142, 0, 0},     {70, 0, 0},
                                 {100, 60, 0},    {90, 0, 0},     {230, 0, 0},   {32, 11, 119},   {0, 74, 111},
                                 {81, 0, 81}};

    if (rectangles.size() % 4 != 0 || rectangles.size() / 4 != classes.size()) {
        return;
    }

    for (size_t i = 0; i < classes.size(); i++) {
        int x = rectangles.at(i * 4);
        int y = rectangles.at(i * 4 + 1);
        int w = rectangles.at(i * 4 + 2);
        int h = rectangles.at(i * 4 + 3);

        int cls = classes.at(i) % colors.size();  // color of a bounding box line

        if (x < 0)
            x = 0;
        if (y < 0)
            y = 0;
        if (w < 0)
            w = 0;
        if (h < 0)
            h = 0;

        if (static_cast<std::size_t>(x) >= width) {
            x = static_cast<int>(width - 1);
            w = 0;
            thickness = 1;
        }
        if (static_cast<std::size_t>(y) >= height) {
            y = static_cast<int>(height - 1);
            h = 0;
            thickness = 1;
        }

        if (static_cast<std::size_t>(x + w) >= width) {
            w = static_cast<int>(width - x - 1);
        }
        if (static_cast<std::size_t>(y + h) >= height) {
            h = static_cast<int>(height - y - 1);
        }

        thickness = std::min(std::min(thickness, w / 2 + 1), h / 2 + 1);

        size_t shift_first;
        size_t shift_second;
        for (int t = 0; t < thickness; t++) {
            shift_first = (y + t) * width * 3;
            shift_second = (y + h - t) * width * 3;
            for (int ii = x; ii < x + w + 1; ii++) {
                data[shift_first + ii * 3] = colors.at(cls).red();
                data[shift_first + ii * 3 + 1] = colors.at(cls).green();
                data[shift_first + ii * 3 + 2] = colors.at(cls).blue();
                data[shift_second + ii * 3] = colors.at(cls).red();
                data[shift_second + ii * 3 + 1] = colors.at(cls).green();
                data[shift_second + ii * 3 + 2] = colors.at(cls).blue();
            }
        }

        for (int t = 0; t < thickness; t++) {
            shift_first = (x + t) * 3;
            shift_second = (x + w - t) * 3;
            for (int ii = y; ii < y + h + 1; ii++) {
                data[shift_first + ii * width * 3] = colors.at(cls).red();
                data[shift_first + ii * width * 3 + 1] = colors.at(cls).green();
                data[shift_first + ii * width * 3 + 2] = colors.at(cls).blue();
                data[shift_second + ii * width * 3] = colors.at(cls).red();
                data[shift_second + ii * width * 3 + 1] = colors.at(cls).green();
                data[shift_second + ii * width * 3 + 2] = colors.at(cls).blue();
            }
        }
    }
}

/**
 * Write output data to image
 * \param name - image name
 * \param data - output data
 * \param classesNum - the number of classes
 * \return false if error else true
 */

static UNUSED bool writeOutputBmp(unsigned char* data, size_t height, size_t width, std::ostream& outFile) {
    unsigned char file[14] = {
        'B',
        'M',  // magic
        0,
        0,
        0,
        0,  // size in bytes
        0,
        0,  // app data
        0,
        0,  // app data
        40 + 14,
        0,
        0,
        0  // start of data offset
    };
    unsigned char info[40] = {
        40,   0,    0, 0,  // info hd size
        0,    0,    0, 0,  // width
        0,    0,    0, 0,  // height
        1,    0,           // number color planes
        24,   0,           // bits per pixel
        0,    0,    0, 0,  // compression is none
        0,    0,    0, 0,  // image bits size
        0x13, 0x0B, 0, 0,  // horz resolution in pixel / m
        0x13, 0x0B, 0, 0,  // vert resolution (0x03C3 = 96 dpi, 0x0B13 = 72 dpi)
        0,    0,    0, 0,  // #colors in palette
        0,    0,    0, 0,  // #important colors
    };

    OPENVINO_ASSERT(
        height < (size_t)std::numeric_limits<int32_t>::max && width < (size_t)std::numeric_limits<int32_t>::max,
        "File size is too big: ",
        height,
        " X ",
        width);

    int padSize = static_cast<int>(4 - (width * 3) % 4) % 4;
    int sizeData = static_cast<int>(width * height * 3 + height * padSize);
    int sizeAll = sizeData + sizeof(file) + sizeof(info);

    file[2] = (unsigned char)(sizeAll);
    file[3] = (unsigned char)(sizeAll >> 8);
    file[4] = (unsigned char)(sizeAll >> 16);
    file[5] = (unsigned char)(sizeAll >> 24);

    info[4] = (unsigned char)(width);
    info[5] = (unsigned char)(width >> 8);
    info[6] = (unsigned char)(width >> 16);
    info[7] = (unsigned char)(width >> 24);

    int32_t negativeHeight = -(int32_t)height;
    info[8] = (unsigned char)(negativeHeight);
    info[9] = (unsigned char)(negativeHeight >> 8);
    info[10] = (unsigned char)(negativeHeight >> 16);
    info[11] = (unsigned char)(negativeHeight >> 24);

    info[20] = (unsigned char)(sizeData);
    info[21] = (unsigned char)(sizeData >> 8);
    info[22] = (unsigned char)(sizeData >> 16);
    info[23] = (unsigned char)(sizeData >> 24);

    outFile.write(reinterpret_cast<char*>(file), sizeof(file));
    outFile.write(reinterpret_cast<char*>(info), sizeof(info));

    unsigned char pad[3] = {0, 0, 0};

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            unsigned char pixel[3];
            pixel[0] = data[y * width * 3 + x * 3];
            pixel[1] = data[y * width * 3 + x * 3 + 1];
            pixel[2] = data[y * width * 3 + x * 3 + 2];
            outFile.write(reinterpret_cast<char*>(pixel), 3);
        }
        outFile.write(reinterpret_cast<char*>(pad), padSize);
    }

    return true;
}

static UNUSED void printPerformanceCounts(const std::map<std::string, ov::ProfilingInfo>& performanceMap,
                                          std::ostream& stream,
                                          std::string deviceName,
                                          bool bshowHeader = true) {
    std::chrono::microseconds totalTime = std::chrono::microseconds::zero();
    // Print performance counts
    if (bshowHeader) {
        stream << std::endl << "performance counts:" << std::endl << std::endl;
    }
    std::ios::fmtflags fmt(std::cout.flags());

    for (const auto& it : performanceMap) {
        std::string toPrint(it.first);
        const int maxLayerName = 30;

        if (it.first.length() >= maxLayerName) {
            toPrint = it.first.substr(0, maxLayerName - 4);
            toPrint += "...";
        }

        stream << std::setw(maxLayerName) << std::left << toPrint;
        switch (it.second.status) {
        case ov::ProfilingInfo::Status::EXECUTED:
            stream << std::setw(15) << std::left << "EXECUTED";
            break;
        case ov::ProfilingInfo::Status::NOT_RUN:
            stream << std::setw(15) << std::left << "NOT_RUN";
            break;
        case ov::ProfilingInfo::Status::OPTIMIZED_OUT:
            stream << std::setw(15) << std::left << "OPTIMIZED_OUT";
            break;
        }
        stream << std::setw(30) << std::left << "layerType: " + std::string(it.second.node_type) + " ";
        stream << std::setw(20) << std::left << "realTime: " + std::to_string(it.second.real_time.count());
        stream << std::setw(20) << std::left << "cpu: " + std::to_string(it.second.cpu_time.count());
        stream << " execType: " << it.second.exec_type << std::endl;
        if (it.second.real_time.count() > 0) {
            totalTime += it.second.real_time;
        }
    }
    stream << std::setw(20) << std::left << "Total time: " + std::to_string(totalTime.count()) << " microseconds"
           << std::endl;
    std::cout << std::endl;
    std::cout << "Full device name: " << deviceName << std::endl;
    std::cout << std::endl;
    std::cout.flags(fmt);
}

/**
 * @brief This class represents an object that is found by an object detection net
 */
class DetectedObject {
public:
    int objectType;
    float xmin, xmax, ymin, ymax, prob;
    bool difficult;

    DetectedObject(int _objectType,
                   float _xmin,
                   float _ymin,
                   float _xmax,
                   float _ymax,
                   float _prob,
                   bool _difficult = false)
        : objectType(_objectType),
          xmin(_xmin),
          xmax(_xmax),
          ymin(_ymin),
          ymax(_ymax),
          prob(_prob),
          difficult(_difficult) {}

    DetectedObject(const DetectedObject& other) = default;

    static float ioU(const DetectedObject& detectedObject1_, const DetectedObject& detectedObject2_) {
        // Add small space to eliminate empty squares
        float epsilon = 0;  // 1e-5f;

        DetectedObject detectedObject1(detectedObject1_.objectType,
                                       (detectedObject1_.xmin - epsilon),
                                       (detectedObject1_.ymin - epsilon),
                                       (detectedObject1_.xmax - epsilon),
                                       (detectedObject1_.ymax - epsilon),
                                       detectedObject1_.prob);
        DetectedObject detectedObject2(detectedObject2_.objectType,
                                       (detectedObject2_.xmin + epsilon),
                                       (detectedObject2_.ymin + epsilon),
                                       (detectedObject2_.xmax),
                                       (detectedObject2_.ymax),
                                       detectedObject2_.prob);

        if (detectedObject1.objectType != detectedObject2.objectType) {
            // objects are different, so the result is 0
            return 0.0f;
        }

        if (detectedObject1.xmax < detectedObject1.xmin)
            return 0.0;
        if (detectedObject1.ymax < detectedObject1.ymin)
            return 0.0;
        if (detectedObject2.xmax < detectedObject2.xmin)
            return 0.0;
        if (detectedObject2.ymax < detectedObject2.ymin)
            return 0.0;

        float xmin = (std::max)(detectedObject1.xmin, detectedObject2.xmin);
        float ymin = (std::max)(detectedObject1.ymin, detectedObject2.ymin);
        float xmax = (std::min)(detectedObject1.xmax, detectedObject2.xmax);
        float ymax = (std::min)(detectedObject1.ymax, detectedObject2.ymax);

        // Caffe adds 1 to every length if the box isn't normalized. So do we...
        float addendum;
        if (xmax > 1 || ymax > 1)
            addendum = 1;
        else
            addendum = 0;

        // intersection
        float intr;
        if ((xmax >= xmin) && (ymax >= ymin)) {
            intr = (addendum + xmax - xmin) * (addendum + ymax - ymin);
        } else {
            intr = 0.0f;
        }

        // union
        float square1 = (addendum + detectedObject1.xmax - detectedObject1.xmin) *
                        (addendum + detectedObject1.ymax - detectedObject1.ymin);
        float square2 = (addendum + detectedObject2.xmax - detectedObject2.xmin) *
                        (addendum + detectedObject2.ymax - detectedObject2.ymin);

        float unn = square1 + square2 - intr;

        return static_cast<float>(intr) / unn;
    }

    DetectedObject scale(float scale_x, float scale_y) const {
        return DetectedObject(objectType,
                              xmin * scale_x,
                              ymin * scale_y,
                              xmax * scale_x,
                              ymax * scale_y,
                              prob,
                              difficult);
    }
};

class ImageDescription {
public:
    const std::list<DetectedObject> alist;
    const bool check_probs;

    explicit ImageDescription(const std::list<DetectedObject>& _alist, bool _check_probs = false)
        : alist(_alist),
          check_probs(_check_probs) {}

    static float ioUMultiple(const ImageDescription& detectedObjects, const ImageDescription& desiredObjects) {
        const ImageDescription *detectedObjectsSmall, *detectedObjectsBig;
        bool check_probs = desiredObjects.check_probs;

        if (detectedObjects.alist.size() < desiredObjects.alist.size()) {
            detectedObjectsSmall = &detectedObjects;
            detectedObjectsBig = &desiredObjects;
        } else {
            detectedObjectsSmall = &desiredObjects;
            detectedObjectsBig = &detectedObjects;
        }

        std::list<DetectedObject> doS = detectedObjectsSmall->alist;
        std::list<DetectedObject> doB = detectedObjectsBig->alist;

        float fullScore = 0.0f;
        while (doS.size() > 0) {
            float score = 0.0f;
            std::list<DetectedObject>::iterator bestJ = doB.end();
            for (auto j = doB.begin(); j != doB.end(); j++) {
                float curscore = DetectedObject::ioU(*doS.begin(), *j);
                if (score < curscore) {
                    score = curscore;
                    bestJ = j;
                }
            }

            float coeff = 1.0;
            if (check_probs) {
                if (bestJ != doB.end()) {
                    float mn = std::min((*bestJ).prob, (*doS.begin()).prob);
                    float mx = std::max((*bestJ).prob, (*doS.begin()).prob);

                    coeff = mn / mx;
                }
            }

            doS.pop_front();
            if (bestJ != doB.end())
                doB.erase(bestJ);
            fullScore += coeff * score;
        }
        fullScore /= detectedObjectsBig->alist.size();

        return fullScore;
    }

    ImageDescription scale(float scale_x, float scale_y) const {
        std::list<DetectedObject> slist;
        for (auto& dob : alist) {
            slist.push_back(dob.scale(scale_x, scale_y));
        }
        return ImageDescription(slist, check_probs);
    }
};

struct AveragePrecisionCalculator {
private:
    enum MatchKind { TruePositive, FalsePositive };

    /**
     * Here we count all TP and FP matches for all the classes in all the images.
     */
    std::map<int, std::vector<std::pair<double, MatchKind>>> matches;

    std::map<int, int> N;

    double threshold;

    static bool SortBBoxDescend(const DetectedObject& bbox1, const DetectedObject& bbox2) {
        return bbox1.prob > bbox2.prob;
    }

    static bool SortPairDescend(const std::pair<double, MatchKind>& p1, const std::pair<double, MatchKind>& p2) {
        return p1.first > p2.first;
    }

public:
    explicit AveragePrecisionCalculator(double _threshold) : threshold(_threshold) {}

    // gt_bboxes -> des
    // bboxes -> det

    void consumeImage(const ImageDescription& detectedObjects, const ImageDescription& desiredObjects) {
        // Collecting IoU values
        std::vector<bool> visited(desiredObjects.alist.size(), false);
        std::vector<DetectedObject> bboxes{std::begin(detectedObjects.alist), std::end(detectedObjects.alist)};
        std::sort(bboxes.begin(), bboxes.end(), SortBBoxDescend);

        for (auto&& detObj : bboxes) {
            // Searching for the best match to this detection
            // Searching for desired object
            float overlap_max = -1;
            int jmax = -1;
            auto desmax = desiredObjects.alist.end();

            int j = 0;
            for (auto desObj = desiredObjects.alist.begin(); desObj != desiredObjects.alist.end(); desObj++, j++) {
                double iou = DetectedObject::ioU(detObj, *desObj);
                if (iou > overlap_max) {
                    overlap_max = static_cast<float>(iou);
                    jmax = j;
                    desmax = desObj;
                }
            }

            MatchKind mk;
            if (overlap_max >= threshold) {
                if (!desmax->difficult) {
                    if (!visited[jmax]) {
                        mk = TruePositive;
                        visited[jmax] = true;
                    } else {
                        mk = FalsePositive;
                    }
                    matches[detObj.objectType].push_back(std::make_pair(detObj.prob, mk));
                }
            } else {
                mk = FalsePositive;
                matches[detObj.objectType].push_back(std::make_pair(detObj.prob, mk));
            }
        }

        for (auto desObj = desiredObjects.alist.begin(); desObj != desiredObjects.alist.end(); desObj++) {
            if (!desObj->difficult) {
                N[desObj->objectType]++;
            }
        }
    }

    std::map<int, double> calculateAveragePrecisionPerClass() const {
        /**
         * Precision-to-TP curve per class (a variation of precision-to-recall curve without
         * dividing into N)
         */
        std::map<int, std::map<int, double>> precisionToTP;

        std::map<int, double> res;

        for (auto m : matches) {
            // Sorting
            std::sort(m.second.begin(), m.second.end(), SortPairDescend);

            int clazz = m.first;
            int TP = 0, FP = 0;

            std::vector<double> prec;
            std::vector<double> rec;

            for (auto mm : m.second) {
                // Here we are descending in a probability value
                MatchKind mk = mm.second;
                if (mk == TruePositive)
                    TP++;
                else if (mk == FalsePositive)
                    FP++;

                double precision = static_cast<double>(TP) / (TP + FP);
                double recall = 0;
                if (N.find(clazz) != N.end()) {
                    recall = static_cast<double>(TP) / N.at(clazz);
                }

                prec.push_back(precision);
                rec.push_back(recall);
            }

            int num = static_cast<int>(rec.size());

            // 11point from Caffe
            double ap = 0;
            std::vector<float> max_precs(11, 0.);
            int start_idx = num - 1;
            for (int j = 10; j >= 0; --j) {
                for (int i = start_idx; i >= 0; --i) {
                    if (rec[i] < j / 10.) {
                        start_idx = i;
                        if (j > 0) {
                            max_precs[j - 1] = max_precs[j];
                        }
                        break;
                    } else {
                        if (max_precs[j] < prec[i]) {
                            max_precs[j] = static_cast<float>(prec[i]);
                        }
                    }
                }
            }
            for (int j = 10; j >= 0; --j) {
                ap += max_precs[j] / 11;
            }
            res[clazz] = ap;
        }

        return res;
    }
};

/**
 * @brief Adds colored rectangles to the image
 * @param data - data where rectangles are put
 * @param height - height of the rectangle
 * @param width - width of the rectangle
 * @param detectedObjects - vector of detected objects
 */
static UNUSED void addRectangles(unsigned char* data,
                                 size_t height,
                                 size_t width,
                                 std::vector<DetectedObject> detectedObjects) {
    std::vector<Color> colors = {{128, 64, 128},  {232, 35, 244}, {70, 70, 70},  {156, 102, 102}, {153, 153, 190},
                                 {153, 153, 153}, {30, 170, 250}, {0, 220, 220}, {35, 142, 107},  {152, 251, 152},
                                 {180, 130, 70},  {60, 20, 220},  {0, 0, 255},   {142, 0, 0},     {70, 0, 0},
                                 {100, 60, 0},    {90, 0, 0},     {230, 0, 0},   {32, 11, 119},   {0, 74, 111},
                                 {81, 0, 81}};

    for (size_t i = 0; i < detectedObjects.size(); i++) {
        int cls = detectedObjects[i].objectType % colors.size();

        int xmin = static_cast<int>(detectedObjects[i].xmin * width);
        int xmax = static_cast<int>(detectedObjects[i].xmax * width);
        int ymin = static_cast<int>(detectedObjects[i].ymin * height);
        int ymax = static_cast<int>(detectedObjects[i].ymax * height);

        size_t shift_first = ymin * width * 3;
        size_t shift_second = ymax * width * 3;
        for (int x = xmin; x < xmax; x++) {
            data[shift_first + x * 3] = colors.at(cls).red();
            data[shift_first + x * 3 + 1] = colors.at(cls).green();
            data[shift_first + x * 3 + 2] = colors.at(cls).blue();
            data[shift_second + x * 3] = colors.at(cls).red();
            data[shift_second + x * 3 + 1] = colors.at(cls).green();
            data[shift_second + x * 3 + 2] = colors.at(cls).blue();
        }

        shift_first = xmin * 3;
        shift_second = xmax * 3;
        for (int y = ymin; y < ymax; y++) {
            data[shift_first + y * width * 3] = colors.at(cls).red();
            data[shift_first + y * width * 3 + 1] = colors.at(cls).green();
            data[shift_first + y * width * 3 + 2] = colors.at(cls).blue();
            data[shift_second + y * width * 3] = colors.at(cls).red();
            data[shift_second + y * width * 3 + 1] = colors.at(cls).green();
            data[shift_second + y * width * 3 + 2] = colors.at(cls).blue();
        }
    }
}

inline void showAvailableDevices() {
    ov::Core core;
    std::vector<std::string> devices = core.get_available_devices();

    std::cout << std::endl;
    std::cout << "Available target devices:";
    for (const auto& device : devices) {
        std::cout << "  " << device;
    }
    std::cout << std::endl;
}

/**
 * @brief Parse text config file. The file must have the following format (with space a delimeter):
 * CONFIG_NAME1 CONFIG_VALUE1
 * CONFIG_NAME2 CONFIG_VALUE2
 *
 * @param configName - filename for a file with config options
 * @param comment - lines starting with symbol `comment` are skipped
 */
std::map<std::string, std::string> parseConfig(const std::string& configName, char comment = '#');

inline std::string getFullDeviceName(ov::Core& core, std::string device) {
    try {
        return core.get_property(device, ov::device::full_name);
    } catch (ov::Exception&) {
        return {};
    }
}

static UNUSED void printPerformanceCounts(std::vector<ov::ProfilingInfo> performanceData,
                                          std::ostream& stream,
                                          std::string deviceName,
                                          bool bshowHeader = true) {
    std::chrono::microseconds totalTime = std::chrono::microseconds::zero();
    // Print performance counts
    if (bshowHeader) {
        stream << std::endl << "performance counts:" << std::endl << std::endl;
    }
    std::ios::fmtflags fmt(std::cout.flags());
    for (const auto& it : performanceData) {
        std::string toPrint(it.node_name);
        const int maxLayerName = 30;

        if (it.node_name.length() >= maxLayerName) {
            toPrint = it.node_name.substr(0, maxLayerName - 5);
            toPrint += "...";
        }

        stream << std::setw(maxLayerName) << std::left << toPrint << " ";
        switch (it.status) {
        case ov::ProfilingInfo::Status::EXECUTED:
            stream << std::setw(15) << std::left << "EXECUTED ";
            break;
        case ov::ProfilingInfo::Status::NOT_RUN:
            stream << std::setw(15) << std::left << "NOT_RUN ";
            break;
        case ov::ProfilingInfo::Status::OPTIMIZED_OUT:
            stream << std::setw(15) << std::left << "OPTIMIZED_OUT ";
            break;
        }
        stream << std::setw(30) << std::left << "layerType: " + std::string(it.node_type) + " ";
        stream << std::setw(30) << std::left << "execType: " + std::string(it.exec_type) + " ";
        stream << std::setw(25) << std::left << "realTime (ms): " + std::to_string(it.real_time.count() / 1000.0) + " ";
        stream << std::setw(25) << std::left << "cpuTime (ms): " + std::to_string(it.cpu_time.count() / 1000.0) + " ";
        stream << std::endl;
        if (it.real_time.count() > 0) {
            totalTime += it.real_time;
        }
    }
    stream << std::setw(25) << std::left << "Total time: " + std::to_string(totalTime.count() / 1000.0)
           << " milliseconds" << std::endl;
    std::cout << std::endl;
    std::cout << "Full device name: " << deviceName << std::endl;
    std::cout << std::endl;
    std::cout.flags(fmt);
}

static UNUSED void printPerformanceCounts(ov::InferRequest request,
                                          std::ostream& stream,
                                          std::string deviceName,
                                          bool bshowHeader = true) {
    auto performanceMap = request.get_profiling_info();
    printPerformanceCounts(performanceMap, stream, deviceName, bshowHeader);
}

static inline std::string double_to_string(const double number) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << number;
    return ss.str();
}

template <typename T>
using uniformDistribution = typename std::conditional<
    std::is_floating_point<T>::value,
    std::uniform_real_distribution<T>,
    typename std::conditional<std::is_integral<T>::value, std::uniform_int_distribution<T>, void>::type>::type;

template <typename T, typename T2>
static inline void fill_random(ov::Tensor& tensor,
                               T rand_min = std::numeric_limits<uint8_t>::min(),
                               T rand_max = std::numeric_limits<uint8_t>::max()) {
    std::mt19937 gen(0);
    size_t tensor_size = tensor.get_size();
    if (0 == tensor_size) {
        throw std::runtime_error(
            "Models with dynamic shapes aren't supported. Input tensors must have specific shapes before inference");
    }
    T* data = tensor.data<T>();
    uniformDistribution<T2> distribution(rand_min, rand_max);
    for (size_t i = 0; i < tensor_size; i++) {
        data[i] = static_cast<T>(distribution(gen));
    }
}

static inline void fill_tensor_random(ov::Tensor tensor) {
    switch (tensor.get_element_type()) {
    case ov::element::f32:
        fill_random<float, float>(tensor);
        break;
    case ov::element::f64:
        fill_random<double, double>(tensor);
        break;
    case ov::element::f16:
        fill_random<short, short>(tensor);
        break;
    case ov::element::i32:
        fill_random<int32_t, int32_t>(tensor);
        break;
    case ov::element::i64:
        fill_random<int64_t, int64_t>(tensor);
        break;
    case ov::element::u8:
        // uniform_int_distribution<uint8_t> is not allowed in the C++17
        // standard and vs2017/19
        fill_random<uint8_t, uint32_t>(tensor);
        break;
    case ov::element::i8:
        // uniform_int_distribution<int8_t> is not allowed in the C++17 standard
        // and vs2017/19
        fill_random<int8_t, int32_t>(tensor, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
        break;
    case ov::element::u16:
        fill_random<uint16_t, uint16_t>(tensor);
        break;
    case ov::element::i16:
        fill_random<int16_t, int16_t>(tensor);
        break;
    case ov::element::boolean:
        fill_random<uint8_t, uint32_t>(tensor, 0, 1);
        break;
    default:
        throw ov::Exception("Input type is not supported for a tensor");
    }
}

static UNUSED void printPerformanceCountsNoSort(std::vector<ov::ProfilingInfo> performanceData,
                                                std::ostream& stream,
                                                std::string deviceName,
                                                bool bshowHeader = true) {
    std::chrono::microseconds totalTime = std::chrono::microseconds::zero();
    // Print performance counts
    if (bshowHeader) {
        stream << std::endl << "performance counts:" << std::endl << std::endl;
    }
    std::ios::fmtflags fmt(std::cout.flags());

    for (const auto& it : performanceData) {
        if (it.real_time.count() > 0) {
            totalTime += it.real_time;
        }
    }
    if (totalTime.count() != 0) {
        for (const auto& it : performanceData) {
            std::string toPrint(it.node_name);
            const int maxLayerName = 30;

            if (it.node_name.length() >= maxLayerName) {
                toPrint = it.node_name.substr(0, maxLayerName - 5);
                toPrint += "...";
            }

            stream << std::setw(maxLayerName) << std::left << toPrint << " ";
            switch (it.status) {
            case ov::ProfilingInfo::Status::EXECUTED:
                stream << std::setw(15) << std::left << "EXECUTED ";
                break;
            case ov::ProfilingInfo::Status::NOT_RUN:
                stream << std::setw(15) << std::left << "NOT_RUN ";
                break;
            case ov::ProfilingInfo::Status::OPTIMIZED_OUT:
                stream << std::setw(15) << std::left << "OPTIMIZED_OUT ";
                break;
            }
            stream << std::setw(30) << std::left << "layerType: " + std::string(it.node_type) + " ";
            stream << std::setw(30) << std::left << "execType: " + std::string(it.exec_type) + " ";
            stream << std::setw(25) << std::left
                   << "realTime (ms): " + std::to_string(it.real_time.count() / 1000.0) + " ";
            stream << std::setw(25) << std::left
                   << "cpuTime (ms): " + std::to_string(it.cpu_time.count() / 1000.0) + " ";

            double opt_proportion = it.real_time.count() * 100.0 / totalTime.count();
            std::stringstream opt_proportion_ss;
            opt_proportion_ss << std::fixed << std::setprecision(2) << opt_proportion;
            std::string opt_proportion_str = opt_proportion_ss.str();
            if (opt_proportion_str == "0.00") {
                opt_proportion_str = "N/A";
            }
            stream << std::setw(20) << std::left << "proportion: " + opt_proportion_str + "%";

            stream << std::endl;
        }
    }
    stream << std::setw(25) << std::left << "Total time: " + std::to_string(totalTime.count() / 1000.0)
           << " milliseconds" << std::endl;
    std::cout << std::endl;
    std::cout << "Full device name: " << deviceName << std::endl;
    std::cout << std::endl;
    std::cout.flags(fmt);
}

static UNUSED bool sort_pc_descend(const ov::ProfilingInfo& profiling1, const ov::ProfilingInfo& profiling2) {
    return profiling1.real_time > profiling2.real_time;
}

static UNUSED void printPerformanceCountsDescendSort(std::vector<ov::ProfilingInfo> performanceData,
                                                     std::ostream& stream,
                                                     std::string deviceName,
                                                     bool bshowHeader = true) {
    std::chrono::microseconds totalTime = std::chrono::microseconds::zero();
    // Print performance counts
    if (bshowHeader) {
        stream << std::endl << "performance counts:" << std::endl << std::endl;
    }
    std::ios::fmtflags fmt(std::cout.flags());

    for (const auto& it : performanceData) {
        if (it.real_time.count() > 0) {
            totalTime += it.real_time;
        }
    }
    if (totalTime.count() != 0) {
        // sort perfcounter
        std::vector<ov::ProfilingInfo> sortPerfCounts{std::begin(performanceData), std::end(performanceData)};
        std::sort(sortPerfCounts.begin(), sortPerfCounts.end(), sort_pc_descend);

        for (const auto& it : sortPerfCounts) {
            std::string toPrint(it.node_name);
            const int maxLayerName = 30;

            if (it.node_name.length() >= maxLayerName) {
                toPrint = it.node_name.substr(0, maxLayerName - 5);
                toPrint += "...";
            }

            stream << std::setw(maxLayerName) << std::left << toPrint << " ";
            switch (it.status) {
            case ov::ProfilingInfo::Status::EXECUTED:
                stream << std::setw(15) << std::left << "EXECUTED ";
                break;
            case ov::ProfilingInfo::Status::NOT_RUN:
                stream << std::setw(15) << std::left << "NOT_RUN ";
                break;
            case ov::ProfilingInfo::Status::OPTIMIZED_OUT:
                stream << std::setw(15) << std::left << "OPTIMIZED_OUT ";
                break;
            }
            stream << std::setw(30) << std::left << "layerType: " + std::string(it.node_type) + " ";
            stream << std::setw(30) << std::left << "execType: " + std::string(it.exec_type) + " ";
            stream << std::setw(25) << std::left
                   << "realTime (ms): " + std::to_string(it.real_time.count() / 1000.0) + " ";
            stream << std::setw(25) << std::left
                   << "cpuTime (ms): " + std::to_string(it.cpu_time.count() / 1000.0) + " ";

            double opt_proportion = it.real_time.count() * 100.0 / totalTime.count();
            std::stringstream opt_proportion_ss;
            opt_proportion_ss << std::fixed << std::setprecision(2) << opt_proportion;
            std::string opt_proportion_str = opt_proportion_ss.str();
            if (opt_proportion_str == "0.00") {
                opt_proportion_str = "N/A";
            }
            stream << std::setw(20) << std::left << "proportion: " + opt_proportion_str + "%";

            stream << std::endl;
        }
    }
    stream << std::setw(25) << std::left << "Total time: " + std::to_string(totalTime.count() / 1000.0)
           << " milliseconds" << std::endl;
    std::cout << std::endl;
    std::cout << "Full device name: " << deviceName << std::endl;
    std::cout << std::endl;
    std::cout.flags(fmt);
}

static UNUSED void printPerformanceCountsSimpleSort(std::vector<ov::ProfilingInfo> performanceData,
                                                    std::ostream& stream,
                                                    std::string deviceName,
                                                    bool bshowHeader = true) {
    std::chrono::microseconds totalTime = std::chrono::microseconds::zero();
    // Print performance counts
    if (bshowHeader) {
        stream << std::endl << "performance counts:" << std::endl << std::endl;
    }
    std::ios::fmtflags fmt(std::cout.flags());

    for (const auto& it : performanceData) {
        if (it.real_time.count() > 0) {
            totalTime += it.real_time;
        }
    }
    if (totalTime.count() != 0) {
        // sort perfcounter
        std::vector<ov::ProfilingInfo> sortPerfCounts{std::begin(performanceData), std::end(performanceData)};
        std::sort(sortPerfCounts.begin(), sortPerfCounts.end(), sort_pc_descend);

        for (const auto& it : sortPerfCounts) {
            if (it.status == ov::ProfilingInfo::Status::EXECUTED) {
                std::string toPrint(it.node_name);
                const int maxLayerName = 30;

                if (it.node_name.length() >= maxLayerName) {
                    toPrint = it.node_name.substr(0, maxLayerName - 5);
                    toPrint += "...";
                }

                stream << std::setw(maxLayerName) << std::left << toPrint << " ";
                stream << std::setw(15) << std::left << "EXECUTED ";
                stream << std::setw(30) << std::left << "layerType: " + std::string(it.node_type) + " ";
                stream << std::setw(30) << std::left << "execType: " + std::string(it.exec_type) + " ";
                stream << std::setw(25) << std::left
                       << "realTime (ms): " + std::to_string(it.real_time.count() / 1000.0) + " ";
                stream << std::setw(25) << std::left
                       << "cpuTime (ms): " + std::to_string(it.cpu_time.count() / 1000.0) + " ";

                double opt_proportion = it.real_time.count() * 100.0 / totalTime.count();
                std::stringstream opt_proportion_ss;
                opt_proportion_ss << std::fixed << std::setprecision(2) << opt_proportion;
                std::string opt_proportion_str = opt_proportion_ss.str();
                if (opt_proportion_str == "0.00") {
                    opt_proportion_str = "N/A";
                }
                stream << std::setw(20) << std::left << "proportion: " + opt_proportion_str + "%";

                stream << std::endl;
            }
        }
    }
    stream << std::setw(25) << std::left << "Total time: " + std::to_string(totalTime.count() / 1000.0)
           << " milliseconds" << std::endl;
    std::cout << std::endl;
    std::cout << "Full device name: " << deviceName << std::endl;
    std::cout << std::endl;
    std::cout.flags(fmt);
}

static UNUSED void printPerformanceCountsSort(std::vector<ov::ProfilingInfo> performanceData,
                                              std::ostream& stream,
                                              std::string deviceName,
                                              std::string sorttype,
                                              bool bshowHeader = true) {
    if (sorttype == pcNoSort) {
        printPerformanceCountsNoSort(performanceData, stream, deviceName, bshowHeader);
    } else if (sorttype == pcSort) {
        printPerformanceCountsDescendSort(performanceData, stream, deviceName, bshowHeader);
    } else if (sorttype == pcSimpleSort) {
        printPerformanceCountsSimpleSort(performanceData, stream, deviceName, bshowHeader);
    }
}
