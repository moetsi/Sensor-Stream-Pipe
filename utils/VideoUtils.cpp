#include "VideoUtils.h"

void avframeToMatYUV(const AVFrame *frame, cv::Mat &image) {
    int width = frame->width;
    int height = frame->height;

    SwsContext *conversion;

    image = cv::Mat(height, width, CV_8UC3);
    conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24,
                                SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
}

void avframeToMatGray(const AVFrame *frame, cv::Mat &image) {
    int width = frame->width;
    int height = frame->height;

    image = cv::Mat(height, width, CV_16UC1);
    for (uint y = 0; y < frame->height; y++) {
        for (uint x = 0; x < frame->width; x++) {
            ushort lower = frame->data[0][y * frame->linesize[0] + x * 2];
            ushort upper = frame->data[0][y * frame->linesize[0] + x * 2 + 1];
            ushort value;

            if (frame->format == AV_PIX_FMT_GRAY12LE) {
                value = upper << 8 | lower;
                image.at<ushort>(y, x) = value;
            } else if (frame->format == AV_PIX_FMT_GRAY16BE) {
                value = lower << 8 | upper;
                image.at<ushort>(y, x) = value;
                //for some reason, the png decoder sums the frame values
                //zeroing the data solves the problem
                //TODO: zero matrix by mem copy
                frame->data[0][y * frame->linesize[0] + x * 2] = 0;
                frame->data[0][y * frame->linesize[0] + x * 2 + 1] = 0;
            }
        }
    }
}

void prepareDecodingStruct(FrameStruct &f, std::unordered_map<std::string, AVCodec *> &pCodecs,
                           std::unordered_map<std::string, AVCodecContext *> &pCodecContexts,
                           std::unordered_map<std::string, AVCodecParameters *> &pCodecParameters) {
    AVCodecParameters *pCodecParameter = f.codec_data.getParams();
    AVCodec *pCodec = avcodec_find_decoder(f.codec_data.getParams()->codec_id);
    AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);

    if (!pCodecContext) {
        std::cerr << "failed to allocated memory for AVCodecContext" << std::endl;
        exit(1);
    }

    if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
        std::cerr << ("failed to copy codec params to codec context") << std::endl;
        exit(1);
    }

    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
        std::cerr << ("failed to open codec through avcodec_open2") << std::endl;
        exit(1);
    }

    pCodecs[f.streamId + std::to_string(f.sensorId)] = pCodec;
    pCodecContexts[f.streamId + std::to_string(f.sensorId)] = pCodecContext;
    pCodecParameters[f.streamId + std::to_string(f.sensorId)] = pCodecParameter;
}


cv::Mat getFloat(cv::Mat &input) {
    cv::Mat output;

    input.convertTo(output, CV_32FC1);

    return output / MAX_DEPTH_VALUE_12_BITS;

}

cv::Mat getUMat(cv::Mat &input) {
    cv::Mat outputF, outputU;

    input.convertTo(outputF, CV_32FC1);

    outputF = (outputF / MAX_DEPTH_VALUE_12_BITS) * MAX_DEPTH_VALUE_8_BITS;

    outputF.convertTo(outputU, CV_8UC1);

    return outputU;

}

std::vector<ushort> unique(const cv::Mat &input, bool sort) {

    std::vector<ushort> out;
    for (int y = 0; y < input.rows; ++y) {
        const ushort *row_ptr = input.ptr<ushort>(y);
        for (int x = 0; x < input.cols; ++x) {
            ushort value = row_ptr[x];

            if (std::find(out.begin(), out.end(), value) == out.end())
                out.push_back(value);
        }
    }

    if (sort)
        std::sort(out.begin(), out.end());

    return out;
}


void prepareGrayDepthFrame(AVFrame *pFrameO, AVFrame *pFrame) {
    for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
            ushort value = 0;
            //ushort value = frame.at<ushort>(y, x);
            uint upper = value >> 8;
            uint lower = value & 0xff;
            pFrame->data[0][y * pFrame->linesize[0] + x * 2] = lower;
            pFrame->data[0][y * pFrame->linesize[0] + x * 2 + 1] = upper;
        }
    }
}


void prepareDepthFrame(AVFrame *pFrameO, AVFrame *pFrame) {
    for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
            //ushort value = frame.at<ushort>(y, x);
            uint lower = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2];
            uint upper = pFrameO->data[0][y * pFrameO->linesize[0] + x * 2 + 1];
            ushort value = upper << 8 | lower;
            pFrame->data[0][y * pFrame->linesize[0] + x] = value;
        }
    }

    for (uint y = 0; y < pFrame->height / 2; y++) {
        for (uint x = 0; x < pFrame->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
            pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
        }
    }
}



void prepareGrayDepthFrame(cv::Mat &frame, AVFrame *pFrame) {
    for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
            ushort value = frame.at<ushort>(y, x);
            uint upper = value >> 8;
            uint lower = value & 0xff;
            pFrame->data[0][y * pFrame->linesize[0] + x * 2] = lower;
            pFrame->data[0][y * pFrame->linesize[0] + x * 2 + 1] = upper;
        }
    }
}


void prepareDepthFrame(cv::Mat &frame, AVFrame *pFrame) {
    for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frame.at<float>(y, x) * MAX_DEPTH_VALUE_8_BITS;
        }
    }

    for (uint y = 0; y < pFrame->height / 2; y++) {
        for (uint x = 0; x < pFrame->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
            pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
        }
    }
}

void prepareColorFrame(cv::Mat &frame, AVFrame *pFrame) {
    for (uint y = 0; y < pFrame->height; y++) {
        for (uint x = 0; x < pFrame->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frame.at<cv::Vec3b>(y, x).val[0];
        }
    }
    for (uint y = 0; y < pFrame->height / 2; y++) {
        for (uint x = 0; x < pFrame->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = frame.at<cv::Vec3b>(2 * y, 2 * x).val[1];
            pFrame->data[2][y * pFrame->linesize[2] + x] = frame.at<cv::Vec3b>(2 * y, 2 * x).val[2];
        }
    }

}
