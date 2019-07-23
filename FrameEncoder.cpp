//
// Created by amourao on 26-06-2019.
//

#include "FrameEncoder.h"

FrameEncoder::FrameEncoder(std::string filename, std::string frame_filename) : FrameReader(filename) {
    reset();
}

FrameStruct FrameEncoder::currentFrame() {
    return FrameStruct();
}


void FrameEncoder::nextFrame() {

}

void FrameEncoder::goToFrame(unsigned int frameId) {
}

void FrameEncoder::reset() {
}
