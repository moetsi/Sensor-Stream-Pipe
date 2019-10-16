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
  ExecStatus status = EXEC_STATUS_UNINIT;
};

struct FrameServerInstance {
  std::string id;
  FrameSourceType source_type;
  std::string metadata;
  std::string host;
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
  ExecStatus status = EXEC_STATUS_UNINIT;
};

enum MsgType {
  SSP_MESSAGE_CONNECT = 0,
  SSP_MESSAGE_START,
  SSP_MESSAGE_STOP,
  SSP_MESSAGE_EXIT,
  SSP_MESSAGE_REG_FS,
  SSP_MESSAGE_REG_P,
  SSP_MESSAGE_REG_CON,
  SSP_MESSAGE_QUE_FS,
  SSP_MESSAGE_QUE_P,
  SSP_MESSAGE_QUE_CON,
  SSP_MESSAGE_CON_FS,
  SSP_MESSAGE_CON_P,
  SSP_MESSAGE_DATA,
  SSP_MESSAGE_OK,
  SSP_MESSAGE_ERROR
};