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

    //TODO: this only works for 3 channel 8 bit color frames, make it work also for 1 channel 16 bit depth
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
    return 0;
}



int main(int argc, char *argv[]) {

    srand(time(NULL));

    try {
        if (argc != 2) {
            std::cerr << "Usage: client <port>" << std::endl;
            return 1;
        }
        zmq::context_t context(1);
        zmq::socket_t socket(context, ZMQ_PULL);
        socket.bind("tcp://*:" + std::string(argv[1]));

        uint64_t last_time = currentTimeMs();
        uint64_t start_time = last_time;
        uint64_t rec_frames = 0;
        double rec_mbytes = 0;

        std::unordered_map<std::string, std::vector<AVCodec *>> pCodecs;
        std::unordered_map<std::string, std::vector<AVCodecContext *>> pCodecContexts;
        std::unordered_map<std::string, std::vector<AVCodecParameters *>> pCodecParameters;

        av_register_all();

        for (;;) {


            zmq::message_t request;

            socket.recv(&request);

            if (rec_frames == 0) {
                last_time = currentTimeMs();
                start_time = last_time;
            }

            rec_frames += 1;
            uint64_t diff_time = currentTimeMs() - last_time;
            double diff_start_time = (currentTimeMs() - start_time) / (double) rec_frames;
            int64_t avg_fps;
            if (diff_start_time == 0)
                avg_fps = -1;
            else
                avg_fps = 1000 / diff_start_time;

            //std::cout << "Message received, took " << diff_time << " ms; size " << paclet_len << "; avg " << avg_fps << " fps" << std::endl;

            last_time = currentTimeMs();

            //std::string result = request.str();
            //TODO: check if it is also necessary to copy from zeromq buffer
            std::string result = std::string(static_cast<char *>(request.data()), request.size());

            VideoFrameStruct f = VideoFrameStruct::parseFrameStruct(result);
            rec_mbytes += request.size() / 1000;

            if (pCodecs.find(f.streamId) == pCodecs.end()) {
                pCodecs[f.streamId] = std::vector<AVCodec *>();
                pCodecContexts[f.streamId] = std::vector<AVCodecContext *>();
                pCodecParameters[f.streamId] = std::vector<AVCodecParameters *>();

                for (uint i = 0; i < f.codec_data.size(); i++) {
                    AVCodecParameters *pCodecParameter = f.codec_data[i].getParams();
                    AVCodec *pCodec = avcodec_find_decoder(pCodecParameter->codec_id);
                    AVCodecContext *pCodecContext = avcodec_alloc_context3(pCodec);

                    if (!pCodecContext) {
                        std::cout << "failed to allocated memory for AVCodecContext" << std::endl;
                    }

                    if (avcodec_parameters_to_context(pCodecContext, pCodecParameter) < 0) {
                        std::cout << ("failed to copy codec params to codec context") << std::endl;
                    }

                    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0) {
                        std::cout << ("failed to open codec through avcodec_open2") << std::endl;
                    }

                    pCodecParameters[f.streamId].push_back(pCodecParameter);
                    pCodecs[f.streamId].push_back(pCodec);
                    pCodecContexts[f.streamId].push_back(pCodecContext);

                }
            }

            for (uint i = 0; i < f.frames.size(); i++) {

                if (f.frameId == 1) { // reset the codec context pm video reset
                    avcodec_flush_buffers(pCodecContexts[f.streamId][i]);
                }

                AVCodecContext *pCodecContext = pCodecContexts[f.streamId][i];

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

                av_packet_from_data(pPacket, &f.frames[i][0], f.frames[i].size());

                int response = 0;

                int error = 0;

                cv::Mat img;

                response = avcodec_send_packet(pCodecContext, pPacket);
                if (response >= 0) {
                    // Return decoded output data (into a frame) from a decoder
                    response = avcodec_receive_frame(pCodecContext, pFrame);
                    if (response >= 0) {
                        response = decode_packet(pPacket, pCodecContext, pFrame, img);

                        cv::namedWindow(f.streamId + std::to_string(i));
                        cv::imshow(f.streamId + std::to_string(i), img);
                        cv::waitKey(1);
                        //av_packet_unref(pPacket);
                    }

                }



                //cv::Mat color = f.getColorFrame();
                //cv::Mat depth = f.getDepthFrame();
                //cv::namedWindow("Display Window");
                //cv::imshow("Display Window", color);
                //cv::waitKey(1);


            }
            std::cout << f.deviceId << ";" << f.sensorId << ";" << f.frameId << " received, took " << diff_time
                      << " ms; size " << request.size()
                      << "; avg " << avg_fps << " fps; " << 8 * (rec_mbytes / (currentTimeMs() - start_time))
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


