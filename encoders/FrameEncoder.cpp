//
// Created by amourao on 26-06-2019.
//

#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string filename, std::string codec_parameters_file) : FrameReader(filename) {
    codec_parameters = YAML::LoadFile(codec_parameters_file);
    av_register_all();
    init();
    streamId = randomString(16);
    totalCurrentFrameCounter = 0;
    currentFrameCounterReset = 0;
    bufferSize = 1;
    reset();
}

std::vector<unsigned char> FrameEncoder::currentFrameBytes() {
    return std::vector<unsigned char>(pPacket->data, pPacket->data + pPacket->size);
}

FrameEncoder::~FrameEncoder() {
    av_packet_free(&pPacket);
    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecContext);
}

void FrameEncoder::nextFrame() {
    encode();
    buffer.pop();
}

void FrameEncoder::prepareFrame() {


    int i, ret, x, y;


    std::string line = frameLines[FrameReader::currentFrameId()];
    std::stringstream ss(line);

    std::string frameIdStr, framePath;

    getline(ss, frameIdStr, ';');
    getline(ss, framePath);


    cv::Mat frameOri = cv::imread(framePath, CV_LOAD_IMAGE_UNCHANGED);

    ret = av_frame_make_writable(pFrame);
    if (ret < 0) {
        std::cerr << "Error making frames writable" << std::endl;
        exit(1);
    }
    // yuv420p 640 320 320 format 0
    // yuv422p 640 320 320 format 4

    if (pCodecContext->pix_fmt == AV_PIX_FMT_GRAY12LE) {
        cv::Mat frameOriSquached;
        //minMaxFilter<ushort>(frameOri, frameOriSquached, 0, MAX_DEPTH_VALUE);
        //frameOriSquached = frameOriSquached * 255;
        //cv::Mat frameOriSquachedF = getFloat(frameOriSquached);
        prepareGrayDepthFrame(frameOri, pFrame);
    } else if (frameOri.channels() == 1) {
        cv::Mat frameOriF = getFloat(frameOri);
        prepareDepthFrame(frameOriF, pFrame);
    } else {
        cv::Mat frameYUV;
        cv::cvtColor(frameOri, frameYUV, cv::COLOR_BGR2YUV);
        prepareColorFrame(frameYUV, pFrame);
    }

    if (FrameReader::hasNextFrame())
        FrameReader::nextFrame();
    else {
        FrameReader::reset();
    }


}

void FrameEncoder::reset() {
    std::cout << "Reset" << std::endl;
    currentFrameCounterReset = 0;
    bufferSize = 1;
    needsToBreak = false;
}

bool FrameEncoder::hasNextFrame() {
    std::cout << bufferSize << " " << buffer.size() << " " << currentFrameCounterReset << " "
              << totalCurrentFrameCounter << std::endl;
    return bufferSize > 0;
}

void FrameEncoder::encode() {

    int ret;

    int i = 0;
    // avcodec_send_frame and avcodec_receive_packet may require multiple calls,
    // (ret == AVERROR(EAGAIN)) before one is able to retrieve a packet.


    do {
        //TODO: make it work with live feed
        if (!FrameReader::hasNextFrame()) {
            needsToBreak = true;
            FrameReader::reset();
        }

        if (buffer.empty()) {
            buffer.emplace(FrameReader::currentFrame());
            buffer.emplace(FrameReader::currentFrame());
        }
        buffer.emplace(FrameReader::currentFrame());
        prepareFrame();

        if (needsToBreak) {
            bufferSize--;
        } else {
            bufferSize = buffer.size() - 2;
        }

        ret = avcodec_send_frame(pCodecContext, pFrame);
        ret = avcodec_receive_packet(pCodecContext, pPacket);
        //std::cout << pFrame->pts << " " << pFrame->pkt_dts << " " << pPacket->pts << " " << pPacket->dts << std::endl << std::endl;
        encoderCalls++;
    } while (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);

    if (ret < 0) {
        std::cerr << "Error during encoding" << std::endl;
    }


    pFrame->pts = totalCurrentFrameCounter++;
    currentFrameCounterReset++;
}

void FrameEncoder::init() {
    int ret;

    std::cout << codec_parameters << std::endl;

    pCodec = avcodec_find_encoder_by_name(codec_parameters["codec_name"].as<std::string>().c_str());
    pCodecContext = avcodec_alloc_context3(pCodec);
    pPacket = av_packet_alloc();
    pCodecParameters = avcodec_parameters_alloc();


    // http://git.videolan.org/?p=ffmpeg.git;a=blob;f=libavcodec/options_table.h;hb=HEAD
    // https://stackoverflow.com/questions/3553003/how-to-encode-h-264-with-libavcodec-x264
    // TODO: adapt to use variable bitrate and check the links above for more parameters
    // check what other parameters are required for different codecs
    pCodecContext->bit_rate = codec_parameters["bitrate"].as<int>();
    av_opt_set(pCodecContext->priv_data, "preset", "b", codec_parameters["bitrate"].as<int>());

    // get resolution from image file
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

    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    //TODO: check what other parameters are required for different codecs
    pCodecContext->gop_size = codec_parameters["gop_size"].as<int>(); // 10
    pCodecContext->max_b_frames = codec_parameters["max_b_frames"].as<int>(); // 1

    // libx264: Supported pixel formats: yuv420p yuvj420p yuv422p yuvj422p yuv444p yuvj444p nv12 nv16 nv21
    // libx265: Supported pixel formats: yuv420p yuv422p yuv444p gbrp yuv420p10le yuv422p10le yuv444p10le gbrp10le yuv420p12le yuv422p12le yuv444p12le gbrp12le gray gray10le gray12le
    pCodecContext->pix_fmt = av_get_pix_fmt(codec_parameters["pix_fmt"].as<std::string>().c_str());


    pCodecParameters->codec_type = AVMEDIA_TYPE_VIDEO;

    pCodecParameters->codec_id = pCodec->id;
    pCodecParameters->codec_tag = pCodecContext->codec_tag;
    pCodecParameters->bit_rate = pCodecContext->bit_rate;
    pCodecParameters->bits_per_coded_sample = pCodecContext->bits_per_coded_sample;
    pCodecParameters->bits_per_raw_sample = pCodecContext->bits_per_raw_sample;
    pCodecParameters->profile = pCodecContext->level;
    pCodecParameters->width = pCodecContext->width;
    pCodecParameters->height = pCodecContext->height;

    pCodecParameters->color_space = pCodecContext->colorspace;
    pCodecParameters->sample_rate = pCodecContext->sample_rate;


    if (pCodec->id == AV_CODEC_ID_H264 || pCodec->id == AV_CODEC_ID_H265)
        av_opt_set(pCodecContext->priv_data, "preset", "slow", 0);


    ret = avcodec_open2(pCodecContext, pCodec, NULL);
    if (ret < 0) {
        std::cerr << "Could not open codec: " << av_err2str(ret) << std::endl;
        exit(1);
    }

    pFrame = av_frame_alloc();
    if (!pFrame) {
        std::cerr << "Could not allocate video frame." << std::endl;
        exit(1);
    }

    pFrame->format = pCodecContext->pix_fmt;
    pFrame->width = pCodecContext->width;
    pFrame->height = pCodecContext->height;

    pFrame->pts = 0;
    pFrame->pkt_dts = 0;

    ret = av_frame_get_buffer(pFrame, 30);
    if (ret < 0) {
        std::cerr << "Could not allocate the video frame data" << std::endl;
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


std::vector<FrameStruct> FrameEncoder::currentFrame() {
    std::vector<FrameStruct> res;
    FrameStruct f = FrameReader::currentFrame().at(0);

    f.messageType = 1;
    f.codec_data = getCodecParamsStruct();
    f.frame = currentFrameBytes();
    f.frameId = currentFrameId();

    res.push_back(f);

    return res;
}

std::vector<FrameStruct> FrameEncoder::currentFrameSync() {
    if (buffer.empty())
        return std::vector<FrameStruct>();
    std::vector<FrameStruct> res = buffer.front();

    return res;
}

