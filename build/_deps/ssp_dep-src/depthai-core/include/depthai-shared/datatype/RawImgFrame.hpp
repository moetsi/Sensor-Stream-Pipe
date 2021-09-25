#pragma once

#include "RawBuffer.hpp"
#include "depthai-shared/common/Timestamp.hpp"
namespace dai {

/// RawImgFrame structure
struct RawImgFrame : public RawBuffer {
    enum class Type {
        YUV422i,    // interleaved 8 bit
        YUV444p,    // planar 4:4:4 format
        YUV420p,    // planar 4:2:0 format
        YUV422p,    // planar 8 bit
        YUV400p,    // 8-bit greyscale
        RGBA8888,   // RGBA interleaved stored in 32 bit word
        RGB161616,  // Planar 16 bit RGB data
        RGB888p,    // Planar 8 bit RGB data
        BGR888p,    // Planar 8 bit BGR data
        RGB888i,    // Interleaved 8 bit RGB data
        BGR888i,    // Interleaved 8 bit BGR data
        LUT2,       // 1 bit  per pixel, Lookup table (used for graphics layers)
        LUT4,       // 2 bits per pixel, Lookup table (used for graphics layers)
        LUT16,      // 4 bits per pixel, Lookup table (used for graphics layers)
        RAW16,      // save any raw type (8, 10, 12bit) on 16 bits
        RAW14,      // 14bit value in 16bit storage
        RAW12,      // 12bit value in 16bit storage
        RAW10,      // 10bit value in 16bit storage
        RAW8,
        PACK10,  // 10bit packed format
        PACK12,  // 12bit packed format
        YUV444i,
        NV12,
        NV21,
        BITSTREAM,  // used for video encoder bitstream
        HDR,
        RGBF16F16F16p,  // Planar FP16 RGB data
        BGRF16F16F16p,  // Planar FP16 BGR data
        RGBF16F16F16i,  // Interleaved FP16 RGB data
        BGRF16F16F16i,  // Interleaved FP16 BGR data
        GRAY8,          // 8 bit grayscale (1 plane)
        GRAYF16,        // FP16 grayscale (normalized)
        NONE
    };
    struct Specs {
        Type type;
        unsigned int width;     // width in pixels
        unsigned int height;    // height in pixels
        unsigned int stride;    // defined as distance in bytes from pix(y,x) to pix(y+1,x)
        unsigned int bytesPP;   // bytes per pixel (for LUT types 1)
        unsigned int p1Offset;  // Offset to first plane
        unsigned int p2Offset;  // Offset to second plane
        unsigned int p3Offset;  // Offset to third plane

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Specs, type, width, height, stride, bytesPP, p1Offset, p2Offset, p3Offset);
    };

    Specs fb;
    uint32_t category;     //
    uint32_t instanceNum;  // Which source created this frame (color, mono, ...)
    int sequenceNum;       // increments for each frame
    Timestamp ts;          // generation timestamp, synced to host time
    Timestamp tsDevice;    // generation timestamp, direct device monotonic clock

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::ImgFrame;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawImgFrame, fb, category, instanceNum, sequenceNum, ts, tsDevice);
};

}  // namespace dai
