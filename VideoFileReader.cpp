//
// Created by amourao on 26-06-2019.
//

#include "VideoFileReader.h"

VideoFileReader::VideoFileReader(std::string filename) {
    // bundle_fusion_apt0;0;0;30
    currentFrameCounter = 0;
    eofReached = false;

    av_register_all();
    std::cout << "initializing all the containers, codecs and protocols." << std::endl;

    // AVFormatContext holds the header information from the format (Container)
    // Allocating memory for this component
    // http://ffmpeg.org/doxygen/trunk/structAVFormatContext.html
    pFormatContext = avformat_alloc_context();
    if (!pFormatContext) {
        std::cout << "ERROR could not allocate memory for Format Context" << std::endl;
        exit(1);
    }

    std::cout << "opening the input file and loading format (container) header" << filename << std::endl;
    // Open the file and read its header. The codecs are not opened.
    // The function arguments are:
    // AVFormatContext (the component we allocated memory for),
    // url (filename),
    // AVInputFormat (if you pass NULL it'll do the auto detect)
    // and AVDictionary (which are options to the demuxer)
    // http://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#ga31d601155e9035d5b0e7efedc894ee49
    if (avformat_open_input(&pFormatContext, filename.c_str(), NULL, NULL) != 0) {
        std::cerr << "ERROR could not open the file" << std::endl;
        exit(1);
    }

    std::cout << "format %s, duration %lld us, bit_rate %lld" <<
              pFormatContext->iformat->name << " " <<
              pFormatContext->duration << " " <<
              pFormatContext->bit_rate
              << std::endl;


    std::cout << "finding stream info from format" << std::endl;
    // read Packets from the Format to get stream information
    // this function populates pFormatContext->streams
    // (of size equals to pFormatContext->nb_streams)
    // the arguments are:
    // the AVFormatContext
    // and options contains options for codec corresponding to i-th stream.
    // On return each dictionary will be filled with options that were not found.
    // https://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#gad42172e27cddafb81096939783b157bb
    if (avformat_find_stream_info(pFormatContext, NULL) < 0) {
        std::cout << "ERROR could not get the stream info" << std::endl;
        exit(-1);
    }

    // the component that knows how to enCOde and DECode the stream
    // it's the codec (audio or video)
    // http://ffmpeg.org/doxygen/trunk/structAVCodec.html

    // this component describes the properties of a codec used by the stream i
    // https://ffmpeg.org/doxygen/trunk/structAVCodecParameters.html
    pCodecParameters = NULL;

    // loop though all the streams and print its main information
    for (int i = 0; i < pFormatContext->nb_streams; i++) {
        pLocalCodecParameters = pFormatContext->streams[i]->codecpar;
        std::cout << "AVStream->time_base before open coded %d/%d" << " " << pFormatContext->streams[i]->time_base.num
                  << " " <<
                  pFormatContext->streams[i]->time_base.den << std::endl;
        std::cout << "AVStream->r_frame_rate before open coded %d/%d" << " "
                  << pFormatContext->streams[i]->r_frame_rate.num << " " <<
                  pFormatContext->streams[i]->r_frame_rate.den << std::endl;
        std::cout << "AVStream->start_time %" PRId64 << " " << pFormatContext->streams[i]->start_time << std::endl;
        std::cout << "AVStream->duration %" PRId64 << " " << pFormatContext->streams[i]->duration << std::endl;

        std::cout << "finding the proper decoder (CODEC)" << std::endl;



        // finds the registered decoder for a codec ID
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga19a0ca553277f019dd5b0fec6e1f9dca
        pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);

        if (pLocalCodec == NULL) {
            std::cout << "ERROR unsupported codec!" << std::endl;
            exit(-1);
        }

        // when the stream is a video we store its index, codec parameters and codec
        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            pCodec = pLocalCodec;
            pCodecParameters = pLocalCodecParameters;

            std::cout << "Video Codec: resolution %d x %d" << " " << pLocalCodecParameters->width << " "
                      << pLocalCodecParameters->height << std::endl;
        } else if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_AUDIO) {
            std::cout << "Audio Codec: %d channels, sample rate %d" << " " << pLocalCodecParameters->channels << " " <<
                      pLocalCodecParameters->sample_rate << std::endl;
        }

        // print its name, id and bitrate
        //std::cout << "\tCodec %s ID %d bit_rate %lld" << " " << pLocalCodec->name << " " << pLocalCodec->id << " " << pCodecParameters->bit_rate << std::endl;
    }
    // https://ffmpeg.org/doxygen/trunk/structAVCodecContext.html
    pCodecContext = avcodec_alloc_context3(pCodec);
    if (!pCodecContext) {
        std::cout << "failed to allocated memory for AVCodecContext" << std::endl;
        exit(-1);
    }

    // Fill the codec context based on the values from the supplied codec parameters
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#gac7b282f51540ca7a99416a3ba6ee0d16
    if (avcodec_parameters_to_context(pCodecContext, pCodecParameters) < 0) {
        std::cout << "failed to copy codec params to codec context" << std::endl;
        exit(-1);
    }

    // Initialize the AVCodecContext to use the given AVCodec.
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#ga11f785a188d7d9df71621001465b0f1d
    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
        std::cout << "failed to open codec through avcodec_open2" << std::endl;
        exit(-1);
    }

    pFrame = av_frame_alloc();
    if (!pFrame) {
        std::cout << "failed to allocated memory for AVFrame" << std::endl;
        exit(-1);
    }
    // https://ffmpeg.org/doxygen/trunk/structAVPacket.html
    pPacket = av_packet_alloc();
    if (!pPacket) {
        std::cout << "failed to allocated memory for AVPacket" << std::endl;
        exit(-1);
    }

    std::vector<std::string> valuesAll = split(filename, "-");
    std::vector<std::string> values = split(valuesAll[0], ".");

    if (values.size() == 3) {
        sensorId = std::stoul(values[1]);
        type = values[2];
    } else {
        sensorId = 0;
        type = values[1];
    }

    deviceId = 0;
    fps = av_q2d(pFormatContext->streams[video_stream_index]->r_frame_rate);

}

VideoFileReader::~VideoFileReader() {
    avformat_close_input(&pFormatContext);
    avformat_free_context(pFormatContext);
    av_packet_free(&pPacket);
    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecContext);
}

std::vector<unsigned char> VideoFileReader::readFile(std::string &filename) {
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> fileData(fileSize);
    file.read((char *) &fileData[0], fileSize);
    return fileData;
}

FrameStruct VideoFileReader::createFrameStruct(unsigned int frameId) {
    // 0;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg;/home/amourao/data/bundle_fusion/apt0/frame-000000.color.jpg

    std::string line = frameLines[frameId];
    std::stringstream ss(line);

    std::string frameIdStr, colorFramePath, depthFramePath;

    getline(ss, frameIdStr, ';');
    getline(ss, colorFramePath, ';');
    getline(ss, depthFramePath);

    unsigned int readFrameId = std::stoul(frameIdStr);

    if (readFrameId != currentFrameCounter)
        std::cerr << "Warning: frame ids do not match: " << readFrameId << " read vs. " << currentFrameCounter
                  << " expected." << std::endl;

    std::vector<unsigned char> colorFileData = readFile(colorFramePath);
    std::vector<unsigned char> depthFileData = readFile(depthFramePath);
    FrameStruct frame = FrameStruct();

    frame.messageType = 0;

    frame.sceneDesc = sceneDesc;
    frame.deviceId = deviceId;
    frame.sensorId = sensorId;

    frame.frameId = readFrameId;
    frame.colorFrame = colorFileData;
    frame.depthFrame = depthFileData;

    return frame;
}

std::string VideoFileReader::getStructBytes(FrameStruct frame) {
    std::ostringstream os(std::ios::binary);

    {
        cereal::BinaryOutputArchive oarchive(os);
        oarchive(frame);
    }

    return os.str();

}

unsigned int VideoFileReader::currentFrameId() {
    return currentFrameCounter;
}

std::string VideoFileReader::currentFrameBytes() {
    return getStructBytes(currentFrameInternal);
}

FrameStruct VideoFileReader::currentFrame() {
    return currentFrameInternal;
}

#undef  av_err2str
#define av_err2str(errnum) av_make_error_string((char*)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE), AV_ERROR_MAX_STRING_SIZE, errnum)


void VideoFileReader::nextFrame() {
    // https://ffmpeg.org/doxygen/trunk/structAVFrame.html

    int error = 0;
    while (error = av_read_frame(pFormatContext, pPacket) >= 0) {
        // if it's the video stream
        if (pPacket->stream_index == video_stream_index) {
            AVPacket *newPacket = av_packet_alloc();
            av_packet_from_data(newPacket, pPacket->data, pPacket->size);
            std::cout << "AVPacket->pts " << PRId64 << " " << newPacket->pts << std::endl;
            int response = decode_packet();
            break;
        }
        // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
        av_packet_unref(pPacket);
    }
    if (error == 0)
        eofReached = true;
    else if (error == 1)
        currentFrameCounter += 1;
    else
        std::cout << "Next frame error: " << error << " " << av_err2str(error) << " " << currentFrameCounter
                  << std::endl;

}


int VideoFileReader::decode_packet() {
    // Supply raw packet data as input to a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
    int response = avcodec_send_packet(pCodecContext, pPacket);

    uint8_t *buffer;
    int numBytes;

    if (response < 0) {
        char errbuf[1000];
        std::cout << "Error while sending a packet to the decoder: %s" <<
                  av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;
        return response;
    }

    while (response >= 0) {
        // Return decoded output data (into a frame) from a decoder
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
        response = avcodec_receive_frame(pCodecContext, pFrame);
        if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
            break;
        } else if (response < 0) {
            char errbuf[1000];
            std::cout << "Error while receiving a frame from the decoder: %s" << " " <<
                      av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;
            return response;
        }

        if (response >= 0) {
            std::cout <<
                      "Frame %d (type=%c, size=%d bytes) pts %d key_frame %d [DTS %d]" <<
                      pCodecContext->frame_number << " " <<
                      av_get_picture_type_char(pFrame->pict_type) << " " <<
                      pFrame->pkt_size << " " <<
                      pFrame->pts << " " <<
                      pFrame->key_frame << " " <<
                      pFrame->coded_picture_number << " "
                      << std::endl;

            // char frame_filename[1024];
            // snprintf(frame_filename, sizeof(frame_filename), "%s-%d.pgm", "frame", pCodecContext->frame_number);
            // save a grayscale frame into a .pgm file
            // save_gray_frame(pFrame->data[0], pFrame->linesize[0], pFrame->width, pFrame->height, frame_filename);

            //cv::Mat img;
            //avframeToMat(pFrame, img);
            //cv::imshow("display", img);
            //cvWaitKey(1000);

        }
    }
    return 0;
}


bool VideoFileReader::hasNextFrame() {
    return !eofReached;
}

void VideoFileReader::goToFrame(unsigned int frameId) {
    std::cerr << "Not implemented!" << std::endl;
}

void VideoFileReader::reset() {
    currentFrameCounter = 0;
    eofReached = false;
    int error = av_seek_frame(pFormatContext, video_stream_index, 0, AVSEEK_FLAG_ANY);
    std::cout << "Reset error: " << error << " " << av_err2str(error) << " " << currentFrameCounter << std::endl;
    //currentFrameInternal = createFrameStruct(currentFrameCounter);
}

unsigned int VideoFileReader::getFps() {
    return fps;
}

unsigned int VideoFileReader::getSensorId() {
    return sensorId;
}

unsigned int VideoFileReader::getDeviceId() {
    return deviceId;
}

std::string VideoFileReader::getSceneDesc() {
    return sceneDesc;
}


int encode(AVCodecContext *avctx, AVPacket *pkt, int *got_packet, AVFrame *frame) {
    int ret;

    *got_packet = 0;

    ret = avcodec_send_frame(avctx, frame);
    if (ret < 0)
        return ret;

    ret = avcodec_receive_packet(avctx, pkt);
    if (!ret)
        *got_packet = 1;
    if (ret == AVERROR(EAGAIN))
        return 0;

    return ret;
}

int decode(AVCodecContext *avctx, AVFrame *frame, int *got_frame, AVPacket *pkt) {
    int ret;

    *got_frame = 0;

    if (pkt) {
        ret = avcodec_send_packet(avctx, pkt);
        // In particular, we don't expect AVERROR(EAGAIN), because we read all
        // decoded frames with avcodec_receive_frame() until done.
        if (ret < 0)
            return ret == AVERROR_EOF ? 0 : ret;
    }

    ret = avcodec_receive_frame(avctx, frame);
    if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
        return ret;
    if (ret >= 0)
        *got_frame = 1;

    return 0;
}
