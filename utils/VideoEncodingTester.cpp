//
// Created by amourao on 07/08/19.
//

#include <ctime>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>
#include <unistd.h>


extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}


#include "../structs/FrameStruct.hpp"
#include "../encoders/FrameEncoder.h"
#include "../utils/Utils.h"


int main(int argc, char *argv[]) {
    srand(time(NULL) * getpid());
    //srand(getpid());

    try {
        if (argc < 3) {
            std::cerr << "Usage: video_encoder_test <codec parameters> <frame_file>"
                      << std::endl;
            return 1;
        }
        std::string codec_parameters_file = std::string(argv[1]);
        std::string frame_file = std::string(argv[2]);

        FrameEncoder fc(frame_file, codec_parameters_file);

        while (fc.hasNextFrame()) {
            FrameStruct f = fc.currentFrame();

            cv::Mat frameOri = cv::imdecode(f.frame, CV_LOAD_IMAGE_UNCHANGED);


            double localMin, localMax;
            cv::minMaxLoc(frameOri, &localMin, &localMax);

            fc.nextFrame();
        }
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}