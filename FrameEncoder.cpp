//
// Created by amourao on 26-06-2019.
//

#include <cv.hpp>
#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string filename, std::string codec_info) : FrameReader(filename) {
    codec_info_string = codec_info;
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

    std::string frameIdStr, colorFramePath, depthFramePath;

    getline(ss, frameIdStr, ';');
    getline(ss, colorFramePath, ';');
    getline(ss, depthFramePath);

    cv::Mat frameOri, frameOriU, frameOriF, frameBGR, frameYUV;
    frameOri = cv::imread(depthFramePath, CV_LOAD_IMAGE_UNCHANGED);
    frameOriU = getUMat(frameOri);
    frameOriF = getFloat(frameOri);

    cv::cvtColor(getFloat(frameOri), frameBGR, cv::COLOR_GRAY2BGR);
    cv::cvtColor(frameBGR, frameYUV, cv::COLOR_BGR2YUV);

    cv::imshow("i1", getFloat(frameOri));
    //cv::imshow("i1a", getUMat(frameOri));
    //cv::imshow("i2", frameBGR);
    //cv::imshow("i3", frameYUV);
    cv::waitKey(1);

    //std::cout << frameOri.type() << " " << frameBGR.type() << " " << frameYUV.type() << std::endl;
    //std::cout << frameOri.size[0] << " " << frameBGR.size[0] << " " << frameYUV.size[0] << std::endl;
    //std::cout << frameOri.at<ushort>(213, 442) << " " << frameBGR.at<cv::Vec3b>(213, 442) << " " << frameYUV.at<cv::Vec3b>(213, 442) << std::endl;

    /* make sure the frame data is writable */
    ret = av_frame_make_writable(pFrame);
    if (ret < 0)
        exit(1);

    for (y = 0; y < pCodecContext->height; y++) {
        for (x = 0; x < pCodecContext->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frameOriF.at<float>(y, x) * 255;
        }
    }

    for (y = 0; y < pCodecContext->height / 2; y++) {
        for (x = 0; x < pCodecContext->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = 128;
            pFrame->data[2][y * pFrame->linesize[2] + x] = 128;
        }
    }
    /* prepare a dummy image */
    /*
    for (y = 0; y < pCodecContext->height; y++) {
        for (x = 0; x < pCodecContext->width; x++) {
            pFrame->data[0][y * pFrame->linesize[0] + x] = frameYUV.at<cv::Vec3b>(y, x).val[0];
        }
    }

    for (y = 0; y < pCodecContext->height / 2; y++) {
        for (x = 0; x < pCodecContext->width / 2; x++) {
            pFrame->data[1][y * pFrame->linesize[1] + x] = frameYUV.at<cv::Vec3b>(2 * y, 2 * x).val[1];
            pFrame->data[2][y * pFrame->linesize[2] + x] = frameYUV.at<cv::Vec3b>(2 * y, 2 * x).val[2];
        }
    }
    */

    pFrame->pts = currentFrameId();


    /* encode the image */
    encode();
    totalCurrentFrameCounter += 1;

}

void FrameEncoder::goToFrame(unsigned int frameId) {
}

void FrameEncoder::encode() {
    int ret;

    /* send the frame to the encoder */
    if (pFrame)
        printf("Send frame %3 PRId64\n", pFrame->pts);

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

    pCodec = avcodec_find_encoder_by_name(codec_info_string.c_str());
    pCodecContext = avcodec_alloc_context3(pCodec);
    pPacket = av_packet_alloc();
    pCodecParameters = avcodec_parameters_alloc();

    //TODO: get parameters from frames and command line options
    /* put sample parameters */
    pCodecContext->bit_rate = 400000;
    /* resolution must be a multiple of two */
    pCodecContext->width = 640;
    pCodecContext->height = 480;
    /* frames per second */
    pCodecContext->time_base = (AVRational) {1, (int) getFps()};
    pCodecContext->framerate = (AVRational) {(int) getFps(), 1};

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
    pCodecContext->gop_size = 10;
    pCodecContext->max_b_frames = 1;
    pCodecContext->pix_fmt = AV_PIX_FMT_YUV420P;

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
