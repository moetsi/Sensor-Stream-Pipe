//
// Created by amourao on 26-06-2019.
//

#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string codec_parameters_file, uint _fps) {
    codec_parameters = YAML::LoadFile(codec_parameters_file);
    fps = _fps;
    av_register_all();

    ready = false;
    totalCurrentFrameCounter = 0;
}

std::vector<unsigned char> FrameEncoder::currentFrameBytes(AVPacket &packet) {
    return std::vector<unsigned char>(packet.data, packet.data + packet.size);
}

FrameEncoder::~FrameEncoder() {
    av_packet_free(&pPacket);
    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecContext);
}

void FrameEncoder::nextPacket() {
    if (!buffer.empty())
        buffer.pop();
    if (!pBuffer.empty())
        pBuffer.pop();
}

void FrameEncoder::prepareFrame() {

    std::vector<unsigned char> frameData = buffer.front().frame;
    cv::Mat frameOri = cv::imdecode(frameData, CV_LOAD_IMAGE_UNCHANGED);

    int ret = av_frame_make_writable(pFrame);
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
}

void FrameEncoder::encodeA(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt) {
    int ret;
    /* send the frame to the encoder */
    ret = avcodec_send_frame(enc_ctx, frame);
    if (ret < 0) {
        fprintf(stderr, "Error sending a frame for encoding\n");
        exit(1);
    }
    while (ret >= 0) {
        ret = avcodec_receive_packet(enc_ctx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0) {
            fprintf(stderr, "Error during encoding\n");
            exit(1);
        }

        AVPacket newPacket(*pPacket);
        newPacket.data = reinterpret_cast<uint8_t *>(new uint64_t[
        (pPacket->size + FF_INPUT_BUFFER_PADDING_SIZE) / sizeof(uint64_t) + 1]);
        memcpy(newPacket.data, pPacket->data, pPacket->size);
        pBuffer.emplace(newPacket);
    }
}

void FrameEncoder::encode() {

    int ret;

    int i = 0;


    prepareFrame();

    pFrame->pts = totalCurrentFrameCounter++;

    encodeA(pCodecContext, pFrame, pPacket);

}


void FrameEncoder::init(FrameStruct fs) {
    int ret;

    std::cout << codec_parameters << std::endl;

    pCodec = avcodec_find_encoder_by_name(codec_parameters["codec_name"].as<std::string>().c_str());
    pCodecContext = avcodec_alloc_context3(pCodec);
    pPacket = av_packet_alloc();
    pCodecParameters = avcodec_parameters_alloc();


    // http://git.videolan.org/?p=ffmpeg.git;a=blob;f=libavcodec/options_table.h;hb=HEAD
    // https://stackoverflow.com/questions/3553003/how-to-encode-h-264-with-libavcodec-x264
    // TODO: adapt to use variable bitrate and check the links above for more parameters: bitrate versus crf
    // check what other parameters are required for different codecs
    pCodecContext->bit_rate = codec_parameters["bitrate"].as<int>();
    av_opt_set(pCodecContext->priv_data, "preset", "b", codec_parameters["bitrate"].as<int>());


    /* get resolution from image file
    std::stringstream ss;
    ss << "/home/amourao/example" << totalCurrentFrameCounter << ".jpg";
    std::ofstream hFile;
    hFile.open(ss.str(), std::ios::out | std::ios::binary);
    if (hFile.is_open())
    {
        hFile.write((char *)&fs.frame[0], static_cast<std::streamsize>(fs.frame.size()));
        hFile.close();
    }
    cv::Mat frameOri = cv::imread(ss.str(), cv::IMREAD_UNCHANGED);
     */
    cv::Mat frameOri = cv::imdecode(fs.frame, CV_LOAD_IMAGE_UNCHANGED);

    std::cout << frameOri.cols << " " << fs.frame.size() << " " << (int) fs.frame[100] << " "
              << (int) fs.frame[fs.frame.size() - 100] << std::endl;

    pCodecContext->width = frameOri.cols;
    pCodecContext->height = frameOri.rows;

    /* frames per second */
    pCodecContext->time_base = (AVRational) {1, (int) fps};
    pCodecContext->framerate = (AVRational) {(int) fps, 1};

    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    //TODO: check what other parameters are required for different codecs
    pCodecContext->gop_size = codec_parameters["gop_size"].as<int>(); // 10
    pCodecContext->max_b_frames = codec_parameters["max_b_frames"].as<int>(); // 1

    // fmpeg -h encoder=hevc
    // libx264:                 yuv420p yuvj420p yuv422p yuvj422p yuv444p yuvj444p nv12 nv16 nv21
    // libx264rgb:              bgr0 bgr24 rgb24
    // libx265:                 yuv420p yuv422p yuv444p gbrp yuv420p10le yuv422p10le yuv444p10le gbrp10le yuv420p12le yuv422p12le yuv444p12le gbrp12le gray gray10le gray12le
    // hevc_nvenc (gpu x265):   yuv420p nv12 p010le yuv444p yuv444p16le bgr0 rgb0 cuda nv12 p010le
    // nvenc_h264 (gpu x264):   yuv420p nv12 p010le yuv444p yuv444p16le bgr0 rgb0 cuda

    pCodecContext->pix_fmt = av_get_pix_fmt(codec_parameters["pix_fmt"].as<std::string>().c_str());
    //TODO: B frames and the GPU
    av_opt_set(pCodecContext->priv_data, "tune", "zerolatency", 0);
    av_opt_set(pCodecContext->priv_data, "rcParams", "zeroReorderDelay", 1);

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

    //TODO: does this parameter really matter?
    if (pCodec->id == AV_CODEC_ID_H264 || pCodec->id == AV_CODEC_ID_H265)
        av_opt_set(pCodecContext->priv_data, "preset", "ultrafast", 0);


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

unsigned int FrameEncoder::currentFrameId() {
    return totalCurrentFrameCounter;
}


FrameStruct FrameEncoder::currentFrame() {
    FrameStruct f = buffer.front();

    f.messageType = 1;
    f.codec_data = getCodecParamsStruct();
    f.frame = currentFrameBytes(pBuffer.front());
    f.frameId = totalCurrentFrameCounter;

    return f;
}

FrameStruct FrameEncoder::currentFrameOriginal() {
    if (buffer.empty())
        return FrameStruct();
    return buffer.front();
}


void FrameEncoder::addFrameStruct(FrameStruct &fs) {
    if (!ready) {
        ready = true;
        init(fs);
    }

    buffer.push(fs);
    encode();

}

uint FrameEncoder::getFps() {
    return fps;
}

bool FrameEncoder::hasNextPacket() {
    return !pBuffer.empty();
}
