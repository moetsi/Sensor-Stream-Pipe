#pragma once

#include "RawBuffer.hpp"

namespace dai {

/// TensorInfo structure
struct TensorInfo {
    enum class StorageOrder : int {
        NHWC = 0x4213,
        NHCW = 0x4231,
        NCHW = 0x4321,
        HWC = 0x213,
        CHW = 0x321,
        WHC = 0x123,
        HCW = 0x231,
        WCH = 0x132,
        CWH = 0x312,
        NC = 0x43,
        CN = 0x34,
        C = 0x3,
        H = 0x2,
        W = 0x1,
    };

    enum class DataType : int {
        FP16 = 0,  // Half precision floating point
        U8F = 1,   // Unsigned byte
        INT = 2,   // Signed integer (4 byte)
        FP32 = 3,  // Single precision floating point
        I8 = 4,    // Signed byte
    };

    StorageOrder order;
    DataType dataType;
    unsigned int numDimensions;
    std::vector<unsigned> dims;
    std::vector<unsigned> strides;
    std::string name;
    unsigned int offset;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TensorInfo, order, dataType, numDimensions, dims, strides, name, offset);
};

/// RawNNData structure
struct RawNNData : public RawBuffer {
    // NNData data is in PoBuf
    std::vector<TensorInfo> tensors;
    unsigned int batchSize;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::NNData;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawNNData, tensors, batchSize);
};

}  // namespace dai
