#pragma once

#include <depthai-shared/common/optional.hpp>
#include <nlohmann/json.hpp>

namespace dai {

/**
 * Specify properties for VideoEncoder such as profile, bitrate, ...
 */
struct VideoEncoderProperties {
    /**
     * Rate control mode specifies if constant or variable bitrate should be used (H264 / H265)
     */
    enum class RateControlMode : int { CBR, VBR };

    /**
     * Encoding profile, H264, H265 or MJPEG
     */
    enum class Profile : int { H264_BASELINE, H264_HIGH, H264_MAIN, H265_MAIN, MJPEG };
    /**
     * Specifies prefered bitrate (kb) of compressed output bitstream
     */
    std::int32_t bitrate = 8000;
    /**
     * Every x number of frames a keyframe will be inserted
     */
    std::int32_t keyframeFrequency = 30;
    /**
     * Specifies maximum bitrate (kb) of compressed output bitstream
     */
    std::int32_t maxBitrate = 8000;
    /**
     * Specifies number of B frames to be inserted
     */
    std::int32_t numBFrames = 0;
    /**
     * This options specifies how many frames are available in this nodes pool (can help if
     * receiver node is slow at consuming
     */
    std::uint32_t numFramesPool = 4;
    /**
     * Encoding profile, H264, H265 or MJPEG
     */
    Profile profile = Profile::H264_BASELINE;
    /**
     * Value between 0-100% (approximates quality)
     */
    std::int32_t quality = 80;
    /**
     * Lossless mode ([M]JPEG only)
     */
    bool lossless = false;
    /**
     * Rate control mode specifies if constant or variable bitrate should be used (H264 / H265)
     */
    RateControlMode rateCtrlMode = RateControlMode::CBR;
    /**
     * Input and compressed output frame width
     */
    std::int32_t width = 1920;
    /**
     * Input and compressed output frame height
     */
    std::int32_t height = 1080;
    /**
     * Frame rate
     */
    float frameRate = 30.0f;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VideoEncoderProperties,
                                   bitrate,
                                   keyframeFrequency,
                                   maxBitrate,
                                   numBFrames,
                                   numFramesPool,
                                   profile,
                                   quality,
                                   lossless,
                                   rateCtrlMode,
                                   width,
                                   height,
                                   frameRate);

}  // namespace dai
