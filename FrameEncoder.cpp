//
// Created by amourao on 26-06-2019.
//

#include <cv.hpp>
#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string filename, std::string codec_parameters_file) : FrameReader(filename) {
    codec_parameters = YAML::LoadFile(codec_parameters_file);
    av_register_all();
    init();
    streamId = randomString(16);
    totalCurrentFrameCounter = 0;
    reset();
}

std::vector<unsigned char> FrameEncoder::currentFrameBytes() {
    return std::vector<unsigned char>(pPacket->data, pPacket->data + pPacket->size);
}

FrameEncoder::~FrameEncoder() {
    avformat_close_input(&pFormatContext);
    avformat_free_context(pFormatContext);
    av_packet_free(&pPacket);
    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecContext);
}


cv::Mat getFloat(cv::Mat &input) {
    cv::Mat output;
    double minVal, maxVal;

    minMaxIdx(input.reshape(0), &minVal, &maxVal);

    input.convertTo(output, CV_32FC1);

    return output / maxVal;

}

cv::Mat getUMat(cv::Mat &input) {
    cv::Mat outputF, outputU;
    double minVal, maxVal;

    minMaxIdx(input.reshape(0), &minVal, &maxVal);

    input.convertTo(outputF, CV_32FC1);

    outputF = (outputF / maxVal) * 256;

    outputF.convertTo(outputU, CV_8UC1);

    return outputU;

}

void FrameEncoder::nextFrame() {
    if (FrameReader::hasNextFrame())
        FrameReader::nextFrame();
    else {
        FrameReader::reset();
    }

    int i, ret, x, y;

    std::string line = frameLines[FrameReader::currentFrameId()];
    std::stringstream ss(line);

    std::string frameIdStr, framePath;

    getline(ss, frameIdStr, ';');
    getline(ss, framePath);


    cv::Mat frameOri = cv::imread(framePath, CV_LOAD_IMAGE_UNCHANGED);

    if (frameOri.channels() == 1) {
        prepareDepthFrame(getFloat(frameOri));
    } else {
        cv::Mat frameYUV;
        cv::cvtColor(frameOri, frameYUV, cv::COLOR_BGR2YUV);
        prepareColorFrame(frameYUV);
    }


    //cv::imshow("i1", getFloat(frameOri));
    //cv::imshow("i1a", getUMat(frameOri));
    //cv::imshow("i2", frameBGR);
    //cv::imshow("i3", frameYUV);


    //std::cout << frameOri.type() << " " << frameBGR.type() << " " << frameYUV.type() << std::endl;
    //std::cout << frameOri.size[0] << " " << frameBGR.size[0] << " " << frameYUV.size[0] << std::endl;
    //std::cout << frameOri.at<ushort>(213, 442) << " " << frameBGR.at<cv::Vec3b>(213, 442) << " " << frameYUV.at<cv::Vec3b>(213, 442) << std::endl;

    /* make sure the frame data is writable */
    ret = av_frame_make_writable(pFrame);
    if (ret < 0)
        exit(1);


    pFrame->pts = currentFrameId();


    /* encode the image */
    encode();
    totalCurrentFrameCounter += 1;

}

void FrameEncoder::goToFrame(unsigned int frameId) {
}

void FrameEncoder::encode() {
    int ret;


    //TODO: add an explanation regarding the do-while cycle and "error" codes
    do {
        ret = avcodec_send_frame(pCodecContext, pFrame);
        ret = avcodec_receive_packet(pCodecContext, pPacket);
    } while (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);

    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        return;
    else if (ret < 0) {
        fprintf(stderr, "Error during encoding\n");
        exit(1);
    }


}

void FrameEncoder::init() {
    int ret;
    uint8_t endcode[] = {0, 0, 1, 0xb7};

    std::cout << codec_parameters << std::endl;

    pCodec = avcodec_find_encoder_by_name(codec_parameters["codec_name"].as<std::string>().c_str());
    pCodecContext = avcodec_alloc_context3(pCodec);
    pPacket = av_packet_alloc();
    pCodecParameters = avcodec_parameters_alloc();

    //TODO: get parameters from frames and command line options
    /* put sample parameters */
    pCodecContext->bit_rate = codec_parameters["bitrate"].as<int>();

    /* get res from file */
    std::string line = frameLines[FrameReader::currentFrameId()];
    std::stringstream ss(line);

    std::string frameIdStr, framePath;

    getline(ss, frameIdStr, ';');
    getline(ss, framePath);
    cv::Mat frameOri = cv::imread(framePath, CV_LOAD_IMAGE_UNCHANGED);

    pCodecContext->width = frameOri.cols;
    pCodecContext->height = frameOri.rows;
    /* frames per second */
    pCodecContext->time_base = (AVRational) {1, (int) getFps()};
    pCodecContext->framerate = (AVRational) {(int) getFps(), 1};

    pCodecContext->gop_size = codec_parameters["gop_size"].as<int>(); // 10
    pCodecContext->max_b_frames = codec_parameters["max_b_frames"].as<int>(); // 1
    pCodecContext->pix_fmt = av_get_pix_fmt(codec_parameters["pix_fmt"].as<std::string>().c_str()); // yuv420, gray16le


    pCodecParameters->codec_type = AVMEDIA_TYPE_VIDEO;

    //TODO: check what happens for different codecs
    //TODO: check what happens for depth frames
    pCodecParameters->codec_id = pCodec->id;
    pCodecParameters->codec_tag = pCodecContext->codec_tag;
    //pCodecParameters->format
    pCodecParameters->bit_rate = pCodecContext->bit_rate;
    pCodecParameters->bits_per_coded_sample = pCodecContext->bits_per_coded_sample;
    pCodecParameters->bits_per_raw_sample = pCodecContext->bits_per_raw_sample;
    pCodecParameters->profile = pCodecContext->level;
    pCodecParameters->width = pCodecContext->width;
    pCodecParameters->height = pCodecContext->height;

    pCodecParameters->color_space = pCodecContext->colorspace;
    pCodecParameters->sample_rate = pCodecContext->sample_rate;

    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */

    if (pCodec->id == AV_CODEC_ID_H264)
        av_opt_set(pCodecContext->priv_data, "preset", "slow", 0);


    ret = avcodec_open2(pCodecContext, pCodec, NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not open codec: %s\n", av_err2str(ret));
        exit(1);
    }

    pFrame = av_frame_alloc();
    if (!pFrame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }

    pFrame->format = pCodecContext->pix_fmt;
    pFrame->width = pCodecContext->width;
    pFrame->height = pCodecContext->height;

    ret = av_frame_get_buffer(pFrame, 30);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate the video frame data\n");
        exit(1);
    }
}

CodecParamsStruct FrameEncoder::getCodecParamsStruct() {
    void *sEPointer = pCodecParameters->extradata;
    size_t sESize = pCodecParameters->extradata_size;
    size_t sSize = sizeof(*pCodecParameters);

    std::vector<unsigned char> e(sSize);
    std::vector<unsigned char> ed(sESize);

    memcpy(&e[0], pCodecParameters, sSize);
    memcpy(&ed[0], sEPointer, sESize);

    CodecParamsStruct cParamsStruct(e, ed);

    return cParamsStruct;
}

std::string FrameEncoder::getStreamID() {
    return streamId;
}

unsigned int FrameEncoder::currentFrameId() {
    return totalCurrentFrameCounter;
}

void FrameEncoder::prepareDepthFrame(cv::Mat frame) {
    for (uint y = 0; y < pCodecContext->height; y++) {
        for (uint x = 0; x < pCodecContext->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frame.at<float>(y, x) * 255;
        }
    }

    for (uint y = 0; y < pCodecContext->height / 2; y++) {
        for (uint x = 0; x < pCodecContext->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
            pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
        }
    }
}

void FrameEncoder::prepareColorFrame(cv::Mat frame) {
    for (uint y = 0; y < pCodecContext->height; y++) {
        for (uint x = 0; x < pCodecContext->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frame.at<cv::Vec3b>(y, x).val[0];
        }
    }

    for (uint y = 0; y < pCodecContext->height / 2; y++) {
        for (uint x = 0; x < pCodecContext->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = frame.at<cv::Vec3b>(2 * y, 2 * x).val[1];
            pFrame->data[2][y * pFrame->linesize[2] + x] = frame.at<cv::Vec3b>(2 * y, 2 * x).val[2];
        }
    }

}
