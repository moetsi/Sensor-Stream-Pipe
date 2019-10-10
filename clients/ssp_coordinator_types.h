//
// Created by amourao on 08/10/19.
//

#pragma once

#include <string>
enum ExchangeDataType {
  EXCHANGE_DATA_TYPE_VECTOR_FRAME_STRUCT = 0,
  EXCHANGE_DATA_TYPE_VECTOR_CV_MAT,
  EXCHANGE_DATA_TYPE_VECTOR_POINTCLOUD,
  EXCHANGE_DATA_TYPE_K4A_CAPTURE,
  EXCHANGE_DATA_TYPE_VECTOR_SKELETON
};

enum FrameSourceType {
  FRAME_SOURCE_ANY = 0,
  FRAME_SOURCE_KINECT_DK,
  FRAME_SOURCE_VIDEO,
  FRAME_SOURCE_FRAMES,
  FRAME_SOURCE_CAMERA
};

enum ExecStatus {
  EXEC_STATUS_UNINIT = 0,
  EXEC_STATUS_STOPPED,
  EXEC_STATUS_WAITING,
  EXEC_STATUS_RUNNING
};

struct ProcessorInstance {
  std::string id;
  FrameSourceType source_type;
  ExchangeDataType data_type;
  std::string metadata;
  std::string host;
  std::string port;
  ExecStatus status = EXEC_STATUS_UNINIT;
};

struct FrameServerInstance {
  std::string id;
  FrameSourceType source_type;
  std::string metadata;
  std::string host;
  std::string port;
  ExecStatus status = EXEC_STATUS_UNINIT;
};

struct FrameServerProcessorConnection {
  std::string id;
  ProcessorInstance processor;
  FrameServerInstance frameserver;
  ExecStatus status = EXEC_STATUS_UNINIT;
};

struct BrokerInstance {
  std::string id;
  std::string host;
  int port;
  ExecStatus status = EXEC_STATUS_UNINIT;
};