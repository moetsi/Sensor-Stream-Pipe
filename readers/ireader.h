//
// Created by amourao on 14-08-2019.
//

#pragma once

#include "../structs/frame_struct.hpp"

//This is a frame iterator
class IReader {

public:
  virtual ~IReader() {}

  //Returns a vector of pointers to a FrameStruct
  //NextFrame() "loads" the data, and GetCurrentFrame() "accesses" the data
  virtual std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame() = 0;

  //This method returns a vector of ints that defines what frame types are supported
  //in the instantiated IReader (so <1> means only depth, <0,2> is color and ir)
  virtual std::vector<unsigned int> GetType() = 0;

  //This is used for iteration to check if the frame source is able to provide another frame
  //Server uses this to check "while(HasNextFrame), then NextFrame()"
  virtual bool HasNextFrame() = 0;
  //Reader will fill all the frame necessary in this method and make it available for "GetCurrentFrame()"
  //NextFrame() "loads" the data, GetCurrentFrame() "accesses" the data
  virtual void NextFrame() = 0;

  //Cannot Reset() for Kinect (or any "live" stream)
  //Used for image reader or video reader
  virtual void Reset() = 0;

  //Cannot GoToFrame() for Kinect (or any "live" stream)
  //Used for image reader or video reader
  virtual void GoToFrame(unsigned int frame_id) = 0;

  virtual unsigned int GetCurrentFrameId() = 0;

  //Will return target FPS of the reader
  //This will be used by the server to "pace" how quickly to generate the vector of frame structs
  virtual unsigned int GetFps() = 0;
};
