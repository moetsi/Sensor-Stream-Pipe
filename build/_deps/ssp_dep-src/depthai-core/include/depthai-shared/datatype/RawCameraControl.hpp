#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "DatatypeEnum.hpp"
#include "RawBuffer.hpp"

namespace dai {

/// RawCameraControl structure
struct RawCameraControl : public RawBuffer {
    enum class Command : uint8_t {
        START_STREAM = 1,
        STOP_STREAM = 2,
        STILL_CAPTURE = 3,
        MOVE_LENS = 4, /* [1] lens position: 0-255
                        */
        AF_TRIGGER = 5,
        AE_MANUAL = 6, /* [1] exposure time [us]
                        * [2] sensitivity [iso]
                        * [3] frame duration [us]
                        */
        AE_AUTO = 7,
        AWB_MODE = 8,                  /* [1] awb_mode: AutoWhiteBalanceMode
                                        */
        SCENE_MODE = 9,                /* [1] scene_mode: SceneMode
                                        */
        ANTIBANDING_MODE = 10,         /* [1] antibanding_mode: AntiBandingMode
                                        */
        EXPOSURE_COMPENSATION = 11,    /* [1] value
                                        */
        AE_LOCK = 13,                  /* [1] ae_lock_mode: bool
                                        */
        AE_TARGET_FPS_RANGE = 14,      /* [1] min_fps
                                        * [2] max_fps
                                        */
        AWB_LOCK = 16,                 /* [1] awb_lock_mode: bool
                                        */
        CAPTURE_INTENT = 17,           /* [1] capture_intent_mode: CaptureIntent
                                        */
        CONTROL_MODE = 18,             /* [1] control_mode: ControlMode
                                        */
        FRAME_DURATION = 21,           /* [1] frame_duration
                                        */
        SENSITIVITY = 23,              /* [1] iso_val
                                        */
        EFFECT_MODE = 24,              /* [1] effect_mode: EffectMode
                                        */
        AF_MODE = 26,                  /* [1] af_mode: AutoFocusMode
                                        */
        NOISE_REDUCTION_STRENGTH = 27, /* [1] value
                                        */
        SATURATION = 28,               /* [1] value
                                        */
        BRIGHTNESS = 31,               /* [1] value
                                        */
        STREAM_FORMAT = 33,            /* [1] format
                                        */
        RESOLUTION = 34,               /* [1] width
                                        * [2] height
                                        */
        SHARPNESS = 35,                /* [1] value
                                        */
        CUSTOM_USECASE = 40,           /* [1] value
                                        */
        CUSTOM_CAPT_MODE = 41,         /* [1] value
                                        */
        CUSTOM_EXP_BRACKETS = 42,      /* [1] val1
                                        * [2] val2
                                        * [3] val3
                                        */
        CUSTOM_CAPTURE = 43,           /* [1] value
                                        */
        CONTRAST = 44,                 /* [1] value
                                        */
        AE_REGION = 45,                /* [1] x
                                        * [2] y
                                        * [3] width
                                        * [4] height
                                        * [5] priority
                                        */
        AF_REGION = 46,                /* [1] x
                                        * [2] y
                                        * [3] width
                                        * [4] height
                                        * [5] priority
                                        */
        LUMA_DENOISE = 47,             /* [1] value
                                        */
        CHROMA_DENOISE = 48,           /* [1] value
                                        */
    };

    enum class AutoFocusMode : uint8_t {
        /// Autofocus disabled. Suitable for manual focus
        OFF = 0,
        /// Runs autofocus once at startup, and at subsequent trigger commands
        AUTO,
        /// TODO
        MACRO,
        /// Runs autofocus when the scene is detected as out-of-focus
        CONTINUOUS_VIDEO,
        CONTINUOUS_PICTURE,
        EDOF,
    };

    enum class AutoWhiteBalanceMode : uint8_t {
        OFF = 0,
        AUTO,
        INCANDESCENT,
        FLUORESCENT,
        WARM_FLUORESCENT,
        DAYLIGHT,
        CLOUDY_DAYLIGHT,
        TWILIGHT,
        SHADE,
    };

    enum class SceneMode : uint8_t {
        UNSUPPORTED = 0,
        FACE_PRIORITY,
        ACTION,
        PORTRAIT,
        LANDSCAPE,
        NIGHT,
        NIGHT_PORTRAIT,
        THEATRE,
        BEACH,
        SNOW,
        SUNSET,
        STEADYPHOTO,
        FIREWORKS,
        SPORTS,
        PARTY,
        CANDLELIGHT,
        BARCODE,
    };

    enum class AntiBandingMode : uint8_t {
        OFF = 0,
        MAINS_50_HZ,
        MAINS_60_HZ,
        AUTO,
    };

    enum class CaptureIntent : uint8_t {
        CUSTOM = 0,
        PREVIEW,
        STILL_CAPTURE,
        VIDEO_RECORD,
        VIDEO_SNAPSHOT,
        ZERO_SHUTTER_LAG,
    };

    enum class ControlMode : uint8_t {
        OFF = 0,
        AUTO,
        USE_SCENE_MODE,
    };

    enum class EffectMode : uint8_t {
        OFF = 0,
        MONO,
        NEGATIVE,
        SOLARIZE,
        SEPIA,
        POSTERIZE,
        WHITEBOARD,
        BLACKBOARD,
        AQUA,
    };

    struct ManualExposureParams {
        uint32_t exposureTimeUs;
        uint32_t sensitivityIso;
        uint32_t frameDurationUs;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(ManualExposureParams, exposureTimeUs, sensitivityIso, frameDurationUs);
    };

    // AE_REGION / AF_REGION
    struct RegionParams {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        // Set to 1 for now. TODO
        uint32_t priority;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(RegionParams, x, y, width, height, priority);
    };

    uint64_t cmdMask = 0;

    AutoFocusMode autoFocusMode = AutoFocusMode::CONTINUOUS_VIDEO;

    /**
     * Lens/VCM position, range: 0..255. Used with `autoFocusMode = OFF`.
     * With current IMX378 modules:
     * - max 255: macro focus, at 8cm distance
     * - infinite focus at about 120..130 (may vary from module to module)
     * - lower values lead to out-of-focus (lens too close to the sensor array)
     */
    uint8_t lensPosition = 0;

    ManualExposureParams expManual;
    RegionParams aeRegion, afRegion;
    AutoWhiteBalanceMode awbMode;
    SceneMode sceneMode;
    AntiBandingMode antiBandingMode;
    EffectMode effectMode;
    bool aeLockMode;
    bool awbLockMode;
    int8_t expCompensation;  //  -9 ..  9
    int8_t brightness;       // -10 .. 10
    int8_t contrast;         // -10 .. 10
    int8_t saturation;       // -10 .. 10
    uint8_t sharpness;       //   0 ..  4
    uint8_t lumaDenoise;     //   0 ..  4
    uint8_t chromaDenoise;   //   0 ..  4

    void setCommand(Command cmd, bool value = true) {
        uint64_t mask = 1ull << (uint8_t)cmd;
        if(value) {
            cmdMask |= mask;
        } else {
            cmdMask &= ~mask;
        }
    }
    void clearCommand(Command cmd) {
        setCommand(cmd, false);
    }
    bool getCommand(Command cmd) {
        return !!(cmdMask & (1ull << (uint8_t)cmd));
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        nlohmann::json j = *this;
        metadata = nlohmann::json::to_msgpack(j);
        datatype = DatatypeEnum::CameraControl;
    };

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(RawCameraControl,
                                   cmdMask,
                                   autoFocusMode,
                                   lensPosition,
                                   expManual,
                                   aeRegion,
                                   afRegion,
                                   awbMode,
                                   sceneMode,
                                   antiBandingMode,
                                   aeLockMode,
                                   awbLockMode,
                                   effectMode,
                                   expCompensation,
                                   brightness,
                                   contrast,
                                   saturation,
                                   sharpness,
                                   lumaDenoise,
                                   chromaDenoise);
};

}  // namespace dai
