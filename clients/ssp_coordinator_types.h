//
// Created by amourao on 08/10/19.
//

#pragma once

#include <string>
#include <zmq.hpp>
enum ExchangeDataType {
  SSP_EXCHANGE_DATA_TYPE_VECTOR_FRAME_STRUCT = 0,
  SSP_EXCHANGE_DATA_TYPE_VECTOR_CV_MAT,
  SSP_EXCHANGE_DATA_TYPE_VECTOR_POINTCLOUD,
  SSP_EXCHANGE_DATA_TYPE_K4A_CAPTURE,
  SSP_EXCHANGE_DATA_TYPE_VECTOR_SKELETON,
  SSP_EXCHANGE_DATA_TYPE_CUSTOM
};

enum ConnectionType {
  SSP_CONNECTION_TYPE_BROKER = 0,
  SSP_CONNECTION_TYPE_FRAMESOURCE,
  SSP_CONNECTION_TYPE_PROCESSOR,
  SSP_CONNECTION_TYPE_CONNECTION,
};

enum ExecStatus {
  SSP_EXEC_STATUS_UNINIT = 0,
  SSP_EXEC_STATUS_STOPPED,
  SSP_EXEC_STATUS_WAITING,
  SSP_EXEC_STATUS_RUNNING
};

struct ProcessorInstance {
  std::string id;
  std::string zmq_id;
  ExchangeDataType source_type;
  ExchangeDataType data_type;
  std::string metadata;
  std::string host;
  ExecStatus status = SSP_EXEC_STATUS_UNINIT;
};

struct FrameServerInstance {
  std::string id;
  std::string zmq_id;
  ExchangeDataType source_type;
  std::string metadata;
  std::string host;
  ExecStatus status = SSP_EXEC_STATUS_UNINIT;
};

struct BrokerInstance {
  std::string id;
  std::string zmq_id;
  std::string host;
  std::string host_out;
  ExecStatus status = SSP_EXEC_STATUS_UNINIT;
};

struct FrameServerProcessorConnection {
  std::string id;
  ProcessorInstance processor;
  FrameServerInstance frameserver;
  ExecStatus status = SSP_EXEC_STATUS_UNINIT;
};

enum MsgType {
  SSP_MESSAGE_CONNECT = 0,
  SSP_MESSAGE_DISCONNECT,
  SSP_MESSAGE_START,
  SSP_MESSAGE_STOP,
  SSP_MESSAGE_EXIT,
  SSP_MESSAGE_REG_CON,
  SSP_MESSAGE_QUE,
  SSP_MESSAGE_DATA,
  SSP_MESSAGE_OK,
  SSP_MESSAGE_DUMMY,
  SSP_MESSAGE_ERROR
};
