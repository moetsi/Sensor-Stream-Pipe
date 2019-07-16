//
// Created by amourao on 26-06-2019.
//

#include <iostream>
#include <chrono>
#include <thread>

#include <zmq.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}


#include "FrameStruct.hpp"
#include "FrameReader.h"
#include "Utils.h"

#define BUFFER_SIZE 1610610


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


static int decode_packet(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame, cv::Mat &img) {
    // Supply raw packet data as input to a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
    int response = avcodec_send_packet(pCodecContext, pPacket);

    uint8_t *buffer;
    int numBytes;

    if (response < 0) {
        char errbuf[1000];
        std::cerr << "Error while sending a packet to the decoder: %s" <<
                  av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;
        return response;
    }
    char errbuf[1000];
    std::cerr << "Error while sending a packet to the decoder: %s" <<
              av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;
    while (response >= 0) {
        // Return decoded output data (into a frame) from a decoder
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
        response = avcodec_receive_frame(pCodecContext, pFrame);
        if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
            break;
        } else if (response < 0) {
            char errbuf[1000];
            std::cout << "Error while receiving a frame from the decoder: %s" <<
                      av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;
            return response;
        }
        char errbuf[1000];
        std::cout << "Error while receiving a frame from the decoder: %s" <<
                  av_make_error_string(errbuf, (size_t) 1000, response) << std::endl;

        if (response >= 0) {
            std::cout <<
                      "Frame %d (type=%c, size=%d bytes) pts %d key_frame %d [DTS %d]" << " " <<
                      pCodecContext->frame_number << " " <<
                      av_get_picture_type_char(pFrame->pict_type) << " " <<
                      pFrame->pkt_size << " " <<
                      pFrame->pts << " " <<
                      pFrame->key_frame << " " <<
                      pFrame->coded_picture_number << " " << std::endl;

            // char frame_filename[1024];
            // snprintf(frame_filename, sizeof(frame_filename), "%s-%d.pgm", "frame", pCodecContext->frame_number);
            // save a grayscale frame into a .pgm file
            // save_gray_frame(pFrame->data[0], pFrame->linesize[0], pFrame->width, pFrame->height, frame_filename);

            avframeToMat(pFrame, img);

        }
    }
    return 0;
}



int main(int argc, char *argv[]) {
    try {
        if (argc != 2) {
            std::cerr << "Usage: client <port>" << std::endl;
            return 1;
        }
        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PULL);
        socket.bind("tcp://*:" + std::string(argv[1]));

        //socket.setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);

        uint64_t last_time = current_time_ms();
        uint64_t start_time = last_time;
        uint64_t rec_frames = 0;
        double rec_mbytes = 0;

        AVCodec *pColorCodec;
        AVCodec *pDepthCodec;

        AVCodecContext *pColorCodecContext;
        AVCodecContext *pDepthCodecContext;

        AVCodecParameters *pColorCodecParameters;
        AVCodecParameters *pDepthCodecParameters;;
        av_register_all();

        for (;;) {


            zmq::message_t request;

            socket.recv(&request);

            if (rec_frames == 0) {
                last_time = current_time_ms();
                start_time = last_time;
            }

            rec_frames += 1;
            uint64_t diff_time = current_time_ms() - last_time;
            double diff_start_time = (current_time_ms() - start_time) / (double) rec_frames;
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                avg_fps = 1000 / diff_start_time;

            //std::cout << "Message received, took " << diff_time << " ms; size " << paclet_len << "; avg " << avg_fps << " fps" << std::endl;

            last_time = current_time_ms();

            //std::string result = request.str();
            //TODO: check if it is also necessary to copy from zeromq buffer
            std::string result = std::string(static_cast<char *>(request.data()), request.size());

            VideoFrameStruct f = VideoFrameStruct::parseFrameStruct(result);
            rec_mbytes += request.size() / 1000;

            if (rec_frames == 1) {
                pColorCodecParameters = f.codec_data[0].getParams();
                pDepthCodecParameters = f.codec_data[1].getParams();

                std::cout << pColorCodecParameters->codec_id << std::endl;
                std::cout << pDepthCodecParameters->codec_id << std::endl;

                std::cout << pColorCodecParameters->codec_tag << std::endl;
                std::cout << pDepthCodecParameters->codec_tag << std::endl;


                pColorCodec = avcodec_find_decoder(pColorCodecParameters->codec_id);
                pDepthCodec = avcodec_find_decoder(pDepthCodecParameters->codec_id);

                // https://ffmpeg.org/doxygen/trunk/structAVCodecContext.html
                pColorCodecContext = avcodec_alloc_context3(pColorCodec);
                pDepthCodecContext = avcodec_alloc_context3(pDepthCodec);
                if (!pColorCodecContext || !pDepthCodecContext) {
                    std::cout << ("failed to allocated memory for AVCodecContext") << std::endl;
                    return -1;
                }

                // Fill the codec context based on the values from the supplied codec parameters
                // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#gac7b282f51540ca7a99416a3ba6ee0d16
                if (avcodec_parameters_to_context(pColorCodecContext, pColorCodecParameters) < 0 ||
                    avcodec_parameters_to_context(pDepthCodecContext, pDepthCodecParameters) < 0
                        ) {
                    std::cout << ("failed to copy codec params to codec context") << std::endl;
                    return -1;
                }

                // Initialize the AVCodecContext to use the given AVCodec.
                // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#ga11f785a188d7d9df71621001465b0f1d
                if (avcodec_open2(pColorCodecContext, pColorCodec, NULL) < 0 ||
                    avcodec_open2(pDepthCodecContext, pDepthCodec, NULL) < 0

                        ) {
                    std::cout << ("failed to open codec through avcodec_open2") << std::endl;
                    return -1;
                }

            }

            AVFrame *pFrame = av_frame_alloc();
            if (!pFrame) {
                std::cout << ("failed to allocated memory for AVFrame") << std::endl;
                return -1;
            }
            // https://ffmpeg.org/doxygen/trunk/structAVPacket.html
            AVPacket *pPacket = av_packet_alloc();
            if (!pPacket) {
                std::cout << ("failed to allocated memory for AVPacket") << std::endl;
                return -1;
            }

            av_packet_from_data(pPacket, &f.frames[0][0], f.frames[0].size());
            std::cout << pPacket << std::endl;

            int response = 0;
            int how_many_packets_to_process = 10;

            // fill the Packet with data from the Stream
            // https://ffmpeg.org/doxygen/trunk/group__lavf__decoding.html#ga4fdb3084415a82e3810de6ee60e46a61
            int error = 0;

            cv::Mat img;

            response = decode_packet(pPacket, pColorCodecContext, pFrame, img);

            cv::namedWindow("Display Window");
            cv::imshow("Display Window", img);
            cv::waitKey(1000);
            if (response < 0)
                break;
            // stop it, otherwise we'll be saving hundreds of frames
            // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
            av_packet_unref(pPacket);

            char errbuf[1000];
            std::cout << "Error: " << error << " " << av_make_error_string(errbuf, (size_t) 1000, error) << std::endl;

            //cv::Mat color = f.getColorFrame();
            //cv::Mat depth = f.getDepthFrame();
            //cv::namedWindow("Display Window");
            //cv::imshow("Display Window", color);
            //cv::waitKey(1);

            std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " received, took " << diff_time
                      << " ms; size " << request.size()
                      << "; avg " << avg_fps << " fps; " << 8 * (rec_mbytes / (current_time_ms() - start_time))
                      << " Mbps" << std::endl;

        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    //avformat_close_input(&pFormatContext);
    //avformat_free_context(pFormatContext);
    //av_packet_free(&pPacket);
    //av_frame_free(&pFrame);
    //avcodec_free_context(&pCodecContext);

    return 0;
}


