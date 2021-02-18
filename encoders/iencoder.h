//
// Created by amourao on 12-09-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"

class IEncoder {

public:
  virtual ~IEncoder() {}

  //Provided a pointer to a FrameStruct, it will pass reference to pointer
  //This prevents copying of data
  //This triggers the encoding
  virtual void AddFrameStruct(std::shared_ptr<FrameStruct> &frame_struct) = 0;

  //Packet means "encoded frame"
  //NextPacket() prepares the encoder for the next FrameStruct
  virtual void NextPacket() = 0;

  //Check to see if the next encoded frame is ready (sometimes it requires 2 FrameStructs) to
  //generate the initial packet
  virtual bool HasNextPacket() = 0;

  //Method to access the encoded data
  virtual std::shared_ptr<FrameStruct> CurrentFrameEncoded() = 0;

  //Method to access the FrameStruct that generated the CurrentFrameEncoded()
  virtual std::shared_ptr<FrameStruct> CurrentFrameOriginal() = 0;

  //Goes into CodecParamsStruct of the FrameStruct and returns the data
  virtual std::shared_ptr<CodecParamsStruct> GetCodecParamsStruct() = 0;

  //Get FPS returns the speed of frame generation of the frame source
  virtual unsigned int GetFps() = 0;
};