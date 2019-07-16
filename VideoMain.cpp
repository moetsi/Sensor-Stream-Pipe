#include <string>

#include <iostream>


extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}


#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "VideoFileReader.h"

// print out the steps and errors
static void logging(const char *fmt, ...);

// decode packets into frames
static int decode_packet(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame);

// save a frame into a .pgm file
static void save_gray_frame(unsigned char *buf, int wrap, int xsize, int ysize, char *filename);

int main(int argc, const char *argv[]) {
    std::string filename_color(argv[1]);
    VideoFileReader vc(filename_color);

    std::string filename_depth(argv[2]);
    VideoFileReader vd(filename_depth);

    do {
        FrameStruct f;

        vc.nextFrame();
        vd.nextFrame();

        f.colorFrame = vc.currentFrameBytes();
        f.depthFrame = vd.currentFrameBytes();

        if (!vc.hasNextFrame())
            vc.reset();
        if (!vd.hasNextFrame())
            vd.reset();
    } while (vc.hasNextFrame());

}

int main2(int argc, const char *argv[]) {

    av_register_all();
    logging("initializing all the containers, codecs and protocols.");

    // AVFormatContext holds the header information from the format (Container)
    // Allocating memory for this component
    // http://ffmpeg.org/doxygen/trunk/structAVFormatContext.html
    AVFormatContext *pFormatContext = avformat_alloc_context();

    if (!pFormatContext) {
        logging("ERROR could not allocate memory for Format Context");
        return -1;
    }

    logging("opening the input file (%s) and loading format (container) header", argv[1]);
    // Open the file and read its header. The codecs are not opened.
    // The function arguments are:
    // AVFormatContext (the component we allocated memory for),
    // url (filename),
    // AVInputFormat (if you pass NULL it'll do the auto detect)
    // and AVDictionary (which are options to the demuxer)
    // http://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#ga31d601155e9035d5b0e7efedc894ee49
    if (avformat_open_input(&pFormatContext, argv[1], NULL, NULL) != 0) {
        logging("ERROR could not open the file");
        return -1;
    }

    // now we have access to some information about our file
    // since we read its header we can say what format (container) it's
    // and some other information related to the format itself.
    logging("format %s, duration %lld us, bit_rate %lld", pFormatContext->iformat->name, pFormatContext->duration,
            pFormatContext->bit_rate);

    logging("finding stream info from format");
    // read Packets from the Format to get stream information
    // this function populates pFormatContext->streams
    // (of size equals to pFormatContext->nb_streams)
    // the arguments are:
    // the AVFormatContext
    // and options contains options for codec corresponding to i-th stream.
    // On return each dictionary will be filled with options that were not found.
    // https://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#gad42172e27cddafb81096939783b157bb
    if (avformat_find_stream_info(pFormatContext, NULL) < 0) {
        logging("ERROR could not get the stream info");
        return -1;
    }

    // the component that knows how to enCOde and DECode the stream
    // it's the codec (audio or video)
    // http://ffmpeg.org/doxygen/trunk/structAVCodec.html
    AVCodec *pCodec = NULL;
    // this component describes the properties of a codec used by the stream i
    // https://ffmpeg.org/doxygen/trunk/structAVCodecParameters.html
    AVCodecParameters *pCodecParameters = NULL;
    int video_stream_index = -1;

    // loop though all the streams and print its main information
    for (int i = 0; i < pFormatContext->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = NULL;

        pLocalCodecParameters = pFormatContext->streams[i]->codecpar;
        logging("AVStream->time_base before open coded %d/%d", pFormatContext->streams[i]->time_base.num,
                pFormatContext->streams[i]->time_base.den);
        logging("AVStream->r_frame_rate before open coded %d/%d", pFormatContext->streams[i]->r_frame_rate.num,
                pFormatContext->streams[i]->r_frame_rate.den);
        logging("AVStream->start_time %" PRId64, pFormatContext->streams[i]->start_time);
        logging("AVStream->duration %" PRId64, pFormatContext->streams[i]->duration);

        logging("finding the proper decoder (CODEC)");

        AVCodec *pLocalCodec = NULL;

        // finds the registered decoder for a codec ID
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga19a0ca553277f019dd5b0fec6e1f9dca
        pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);

        if (pLocalCodec == NULL) {
            logging("ERROR unsupported codec!");
            return -1;
        }

        // when the stream is a video we store its index, codec parameters and codec
        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            if (video_stream_index == -1) {
                video_stream_index = i;
                pCodec = pLocalCodec;
                pCodecParameters = pLocalCodecParameters;
            }

            logging("Video Codec: resolution %d x %d", pLocalCodecParameters->width, pLocalCodecParameters->height);
        } else if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_AUDIO) {
            logging("Audio Codec: %d channels, sample rate %d", pLocalCodecParameters->channels,
                    pLocalCodecParameters->sample_rate);
        }

        // print its name, id and bitrate
        logging("\tCodec %s ID %d bit_rate %lld", pLocalCodec->name, pLocalCodec->id, pCodecParameters->bit_rate);
    }

    // https://ffmpeg.org/doxygen/trunk/structAVCodecContext.html
    AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);
    if (!pCodecContext) {
        logging("failed to allocated memory for AVCodecContext");
        return -1;
    }

    // Fill the codec context based on the values from the supplied codec parameters
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#gac7b282f51540ca7a99416a3ba6ee0d16
    if (avcodec_parameters_to_context(pCodecContext, pCodecParameters) < 0) {
        logging("failed to copy codec params to codec context");
        return -1;
    }

    // Initialize the AVCodecContext to use the given AVCodec.
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#ga11f785a188d7d9df71621001465b0f1d
    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
        logging("failed to open codec through avcodec_open2");
        return -1;
    }

    // https://ffmpeg.org/doxygen/trunk/structAVFrame.html
    AVFrame *pFrame = av_frame_alloc();
    if (!pFrame) {
        logging("failed to allocated memory for AVFrame");
        return -1;
    }
    // https://ffmpeg.org/doxygen/trunk/structAVPacket.html
    AVPacket *pPacket = av_packet_alloc();
    if (!pPacket) {
        logging("failed to allocated memory for AVPacket");
        return -1;
    }

    int response = 0;
    int how_many_packets_to_process = 10;

    // fill the Packet with data from the Stream
    // https://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#ga4fdb3084415a82e3810de6ee60e46a61
    int error = 0;
    while (error = avcodec_receive_packet(pCodecContext, pPacket) >= 0) {
        // if it's the video stream
        if (pPacket->stream_index == video_stream_index) {
            AVPacket *newPacket = av_packet_alloc();
            av_packet_from_data(newPacket, pPacket->data, pPacket->size);
            logging("AVPacket->pts %" PRId64, newPacket->pts);
            response = decode_packet(newPacket, pCodecContext, pFrame);
            if (response < 0)
                break;
            // stop it, otherwise we'll be saving hundreds of frames
            if (--how_many_packets_to_process <= 0) break;
        }
        // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
        av_packet_unref(pPacket);
    }
    char errbuf[1000];
    logging("Error: %s", av_make_error_string(errbuf, (size_t) 1000, error));

    logging("releasing all the resources");

    avformat_close_input(&pFormatContext);
    avformat_free_context(pFormatContext);
    av_packet_free(&pPacket);
    av_frame_free(&pFrame);
    avcodec_free_context(&pCodecContext);
    return 0;
}

static void logging(const char *fmt, ...) {
    va_list args;
    fprintf(stderr, "LOG: ");
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n");
}

static int decode_packet2(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame) {
    // Supply raw packet data as input to a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
    int response = avcodec_send_packet(pCodecContext, pPacket);

    if (response < 0) {
        char errbuf[1000];
        logging("Error while sending a packet to the decoder: %s",
                av_make_error_string(errbuf, (size_t) 1000, response));
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
            logging("Error while receiving a frame from the decoder: %s",
                    av_make_error_string(errbuf, (size_t) 1000, response));
            return response;
        }

        if (response >= 0) {
            logging(
                    "Frame %d (type=%c, size=%d bytes) pts %d key_frame %d [DTS %d]",
                    pCodecContext->frame_number,
                    av_get_picture_type_char(pFrame->pict_type),
                    pFrame->pkt_size,
                    pFrame->pts,
                    pFrame->key_frame,
                    pFrame->coded_picture_number
            );

            char frame_filename[1024];
            snprintf(frame_filename, sizeof(frame_filename), "%s-%d.pgm", "frame", pCodecContext->frame_number);
            // save a grayscale frame into a .pgm file
            save_gray_frame(pFrame->data[0], pFrame->linesize[0], pFrame->width, pFrame->height, frame_filename);


        }
    }
    return 0;
}

static void avframeToMat(const AVFrame *frame, cv::Mat &image) {
    int width = frame->width;
    int height = frame->height;

    std::cout << av_frame_get_color_range(frame) << " " << image.step1() << " " << image.step1(0) << std::endl;

    SwsContext *conversion;

    // Allocate the opencv mat and store its stride in a 1-element array
    image = cv::Mat(height, width, CV_8UC3);
    conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24,
                                    SWS_FAST_BILINEAR, NULL, NULL, NULL);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
}


static int decode_packet(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame) {
    // Supply raw packet data as input to a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
    int response = avcodec_send_packet(pCodecContext, pPacket);

    uint8_t *buffer;
    int numBytes;

    if (response < 0) {
        char errbuf[1000];
        logging("Error while sending a packet to the decoder: %s",
                av_make_error_string(errbuf, (size_t) 1000, response));
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
            logging("Error while receiving a frame from the decoder: %s",
                    av_make_error_string(errbuf, (size_t) 1000, response));
            return response;
        }

        if (response >= 0) {
            logging(
                    "Frame %d (type=%c, size=%d bytes) pts %d key_frame %d [DTS %d]",
                    pCodecContext->frame_number,
                    av_get_picture_type_char(pFrame->pict_type),
                    pFrame->pkt_size,
                    pFrame->pts,
                    pFrame->key_frame,
                    pFrame->coded_picture_number
            );

            // char frame_filename[1024];
            // snprintf(frame_filename, sizeof(frame_filename), "%s-%d.pgm", "frame", pCodecContext->frame_number);
            // save a grayscale frame into a .pgm file
            // save_gray_frame(pFrame->data[0], pFrame->linesize[0], pFrame->width, pFrame->height, frame_filename);

            cv::Mat img;
            avframeToMat(pFrame, img);
            cv::imshow("display", img);
            cvWaitKey(1000);

        }
    }
    return 0;
}


static void save_gray_frame(unsigned char *buf, int wrap, int xsize, int ysize, char *filename) {
    FILE *f;
    int i;
    f = fopen(filename, "w");
    // writing the minimal required header for a pgm file format
    // portable graymap format -> https://en.wikipedia.org/wiki/Netpbm_format#PGM_example
    fprintf(f, "P5\n%d %d\n%d\n", xsize, ysize, 255);

    // writing line by line
    for (i = 0; i < ysize; i++)
        fwrite(buf + i * wrap, 1, xsize, f);
    fclose(f);
}