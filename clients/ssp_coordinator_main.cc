//
// Created by amourao on 17-10-2019.
//

#include "../utils/logger.h"
#include "../utils/utils.h"
#include "ssp_coordinator.h"
#include <iostream>
#include <zmq.hpp>

zmq::message_t BuildOKMessage() {
  std::string ok_msg = std::string(1, char(SSP_MESSAGE_OK));
  return zmq::message_t(ok_msg.c_str(), ok_msg.size());
}

zmq::message_t BuildOKMessage(const std::string output) {
  std::string ok_msg = std::string(1, char(SSP_MESSAGE_OK)) + " " + output;
  return zmq::message_t(ok_msg.c_str(), ok_msg.size());
}

zmq::message_t BuildErrorMessage(const int error_int, const std::string error) {
  std::string error_msg = std::string(1, char(SSP_MESSAGE_ERROR)) + " " +
                          std::to_string(error_int) + " " + error;
  return zmq::message_t(error_msg.c_str(), error_msg.size());
}

zmq::message_t BuildMessage(const std::string zmq_id) {
  return zmq::message_t(zmq_id.c_str(), zmq_id.size());
}

int main() {

  spdlog::set_level(spdlog::level::debug);

  std::string coor_host = "127.0.0.1";
  int coor_port = 9999;

  std::string error_msg;
  int error;

  int SIZE = 256 * 256;
  zmq::message_t in_request(SIZE);
  int i = 0;

  zmq::context_t context(1);
  zmq::socket_t coor_socket(context, ZMQ_ROUTER);
  coor_socket.bind("tcp://" + coor_host + ":" + std::to_string(coor_port));

  SSPCoordinator ssp_coordinator;

  // error = ssp_coordinator.RegisterBroker("127.0.0.1:10000", "dummy", "",
  // error_msg);

  bool leave = false;

  while (!leave) {
    zmq::message_t id_request(SIZE);
    zmq::message_t emp_request(SIZE);
    spdlog::info("Waiting for request");
    coor_socket.recv(&id_request);
    coor_socket.recv(&emp_request);
    coor_socket.recv(&in_request);

    std::string id_msg((char *)id_request.data(), id_request.size());
    std::string msg_rsp((char *)in_request.data(), in_request.size());
    char msg_type = msg_rsp.substr(0, 1).c_str()[0];

    switch (msg_type) {
    case SSP_MESSAGE_CONNECT: {
      char conn_type = msg_rsp.substr(1, 1).c_str()[0];
      switch (conn_type) {
      case SSP_CONNECTION_TYPE_BROKER: {
        spdlog::info("SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_BROKER request");
        std::string identity = in_request.gets("Identity");
        std::string host = in_request.gets("Peer-Address");

        std::string id = msg_rsp.substr(3, msg_rsp.size() - 3);
        std::string zmq_id = id_msg;

        int port_in;
        int port_out;
        error = ssp_coordinator.RegisterBroker(host, id, zmq_id, port_in,
                                               port_out, error_msg);
        zmq::message_t answer;
        if (error != 0) {
          spdlog::error(
              "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_BROKER error " +
              std::to_string(error) + "\": " + error_msg + "\"");
          answer = BuildErrorMessage(error, error_msg);
        } else {
          spdlog::info("SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_BROKER ok");
          answer = BuildOKMessage(std::to_string(port_in) + " " +
                                  std::to_string(port_out));
        }
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      case SSP_CONNECTION_TYPE_FRAMESOURCE: {
        spdlog::info(
            "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_FRAMESOURCE request");
        std::string data = msg_rsp.substr(2, msg_rsp.size() - 2);
        spdlog::info(data);
        std::string delimiter = " ";
        std::vector<std::string> sdata = SplitString(data, delimiter);
        std::string host = sdata.at(0);
        std::string id = sdata.at(1);
        std::string fst_string = sdata.at(2);
        std::string metadata = sdata.at(3);

        std::string zmq_id = id_msg;

        ExchangeDataType fst =
            static_cast<ExchangeDataType>(fst_string.c_str()[0]);

        error = ssp_coordinator.RegisterFrameSource(host, id, zmq_id, fst,
                                                    metadata, error_msg);
        zmq::message_t answer;
        if (error != 0) {
          spdlog::error(
              "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_FRAMESOURCE error " +
              std::to_string(error) + "\": " + error_msg + "\"");
          answer = BuildErrorMessage(error, error_msg);
        } else {
          spdlog::info(
              "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_FRAMESOURCE ok");
          BrokerInstance b;
          ssp_coordinator.GetBroker(b, error_msg);
          answer = BuildOKMessage(b.host);
        }
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      case SSP_CONNECTION_TYPE_PROCESSOR: {
        spdlog::info(
            "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_PROCESSOR request");
        std::string data = msg_rsp.substr(2, msg_rsp.size() - 2);
        std::string delimitor = " ";
        std::vector<std::string> sdata = SplitString(data, delimitor);
        std::string host = sdata.at(0);
        std::string id = sdata.at(1);
        std::string fst_string = sdata.at(2);
        std::string edt_string = sdata.at(3);
        std::string metadata = sdata.at(4);

        std::string zmq_id = id_msg;

        ExchangeDataType fst =
            static_cast<ExchangeDataType>(fst_string.c_str()[0]);

        ExchangeDataType edt =
            static_cast<ExchangeDataType>(edt_string.c_str()[0]);

        error = ssp_coordinator.RegisterProcessor(host, id, zmq_id, fst, edt,
                                                  metadata, error_msg);
        zmq::message_t answer;
        if (error != 0) {
          spdlog::error(
              "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_PROCESSOR error " +
              std::to_string(error) + "\": " + error_msg + "\"");
          answer = BuildErrorMessage(error, error_msg);
        } else {
          spdlog::info("SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_PROCESSOR ok");
          BrokerInstance b;
          ssp_coordinator.GetBroker(b, error_msg);
          answer = BuildOKMessage(b.host_out);
        }
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      default: {
        spdlog::error("SSP_MESSAGE_CONNECT UNKNOWN SSP_CONNECTION_TYPE");
        error = -1;
        error_msg = "SSP_MESSAGE_CONNECT UNKNOWN SSP_CONNECTION_TYPE";
        zmq::message_t answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      }
      break;
    }
    case SSP_MESSAGE_QUE: {
      char conn_type = msg_rsp.substr(1, 1).c_str()[0];
      switch (conn_type) {
      case SSP_CONNECTION_TYPE_BROKER: {
        spdlog::info("SSP_MESSAGE_QUE SSP_CONNECTION_TYPE_BROKER request");
        std::vector<std::pair<std::string, std::string>> results;
        error = ssp_coordinator.GetBrokers(results, error_msg);
        std::string answer_text = "";
        for (auto res : results) {
          answer_text += res.first + "," + res.second + ";";
        }
        zmq::message_t answer = BuildOKMessage(answer_text);

        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      case SSP_CONNECTION_TYPE_FRAMESOURCE: {
        spdlog::info("SSP_MESSAGE_QUE SSP_CONNECTION_TYPE_FRAMESOURCE request");
        std::vector<std::pair<std::string, ExchangeDataType>> results;
        error = ssp_coordinator.GetFrameSources(results, error_msg);
        std::string answer_text = "";
        for (auto res : results) {
          answer_text +=
              res.first + "," + std::string(1, (char)res.second) + ";";
        }
        zmq::message_t answer = BuildOKMessage(answer_text);

        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      case SSP_CONNECTION_TYPE_PROCESSOR: {
        spdlog::info("SSP_MESSAGE_QUE SSP_CONNECTION_TYPE_PROCESSOR request");
        std::vector<std::pair<std::string,
                              std::pair<ExchangeDataType, ExchangeDataType>>>
            results;
        error = ssp_coordinator.GetProcessors(results, error_msg);
        std::string answer_text = "";
        for (auto res : results) {
          answer_text += res.first + "," +
                         std::string(1, (char)res.second.first) + "," +
                         std::string(1, (char)res.second.second) + ";";
        }
        zmq::message_t answer = BuildOKMessage(answer_text);

        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      case SSP_CONNECTION_TYPE_CONNECTION: {
        spdlog::info("SSP_MESSAGE_QUE SSP_CONNECTION_TYPE_CONNECTION request");
        std::vector<std::pair<std::string,
                              std::pair<ExchangeDataType, ExchangeDataType>>>
            results;
        error = ssp_coordinator.GetConnections(results, error_msg);
        std::string answer_text = "";
        for (auto res : results) {
          answer_text += res.first + "," +
                         std::string(1, (char)res.second.first) + "," +
                         std::string(1, (char)res.second.second) + ";";
        }
        zmq::message_t answer = BuildOKMessage(answer_text);

        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      default: {
        spdlog::error("SSP_MESSAGE_CONNECT UNKNOWN SSP_CONNECTION_TYPE");
        error = -1;
        error_msg = "SSP_MESSAGE_CONNECT UNKNOWN SSP_CONNECTION_TYPE";
        zmq::message_t answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }
      }
      break;
    }
    case SSP_MESSAGE_REG_CON: {
      // TODO: what should happen if mulitple frame sources connect to the same
      // processor
      std::string data = msg_rsp.substr(1, msg_rsp.size() - 1);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string id_fs = sdata.at(0);
      std::string id_proc = sdata.at(1);

      FrameServerProcessorConnection connection;
      connection.status = SSP_EXEC_STATUS_RUNNING;

      error = ssp_coordinator.Connect(id_fs, id_proc, connection, error_msg);

      zmq::message_t answer;
      if (error != 0) {
        spdlog::error(
            "SSP_MESSAGE_CONNECT SSP_CONNECTION_TYPE_PROCESSOR error " +
            std::to_string(error) + "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
      } else {
        ProcessorInstance pi;
        FrameServerInstance fsi;
        error = ssp_coordinator.GetProcessorInfo(id_proc, pi, error_msg);
        error = ssp_coordinator.GetFrameServerInfo(id_fs, fsi, error_msg);

        std::string id_proc_request = pi.zmq_id;
        zmq::message_t id_proc_msg = BuildMessage(id_proc_request);
        coor_socket.send(id_proc_msg, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);

        std::string connect_msg =
            std::string(1, char(SSP_MESSAGE_CONNECT)) +
            std::string(1, char(SSP_CONNECTION_TYPE_FRAMESOURCE)) + " " +
            fsi.host + " " + fsi.id;

        zmq::message_t request = BuildMessage(connect_msg);
        coor_socket.send(request);

        spdlog::info(
            "Send SSP_MESSAGE_CONNECT to SSP_CONNECTION_TYPE_PROCESSOR ok");
        answer =
            BuildOKMessage(connection.processor.host + " " + connection.id);
      }
      coor_socket.send(id_request, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(answer);
      break;
    }
    case SSP_MESSAGE_START: {
      std::string data = msg_rsp.substr(1, msg_rsp.size() - 1);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string id_con = sdata.at(0);

      FrameServerProcessorConnection connection;
      error = ssp_coordinator.GetConnectionInfo(id_con, connection, error_msg);

      zmq::message_t answer;
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_START error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      error = ssp_coordinator.Start(id_con, error_msg);
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_START error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      ProcessorInstance pi = connection.processor;
      FrameServerInstance fsi = connection.frameserver;

      std::string id_fsi_request = fsi.zmq_id;
      zmq::message_t id_fsi_msg = BuildMessage(id_fsi_request);
      coor_socket.send(id_fsi_msg, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);

      std::string connect_msg = std::string(1, char(SSP_MESSAGE_START));

      zmq::message_t request = BuildMessage(connect_msg);
      coor_socket.send(request);

      spdlog::info("SSP_MESSAGE_START " + id_con + " ok");
      answer = BuildOKMessage();

      coor_socket.send(id_request, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(answer);
      break;
    }
    case SSP_MESSAGE_STOP: {
      std::string data = msg_rsp.substr(1, msg_rsp.size() - 1);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string id_con = sdata.at(0);

      FrameServerProcessorConnection connection;
      error = ssp_coordinator.GetConnectionInfo(id_con, connection, error_msg);

      zmq::message_t answer;
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_STOP error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      error = ssp_coordinator.Stop(id_con, error_msg);
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_STOP error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      ProcessorInstance pi = connection.processor;
      FrameServerInstance fsi = connection.frameserver;

      std::string id_fsi_request = fsi.zmq_id;
      std::string id_pi_request = pi.zmq_id;
      zmq::message_t id_fsi_msg = BuildMessage(id_fsi_request);
      zmq::message_t id_pi_msg = BuildMessage(id_pi_request);

      std::string connect_msg = std::string(1, char(SSP_MESSAGE_STOP));
      zmq::message_t request = BuildMessage(connect_msg);

      // TODO: what should happen to the processor on stop?
      coor_socket.send(id_fsi_msg, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(request);

      spdlog::info("SSP_MESSAGE_STOP " + id_con + " ok");
      answer = BuildOKMessage();

      coor_socket.send(id_request, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(answer);
      break;
    }
    case SSP_MESSAGE_DISCONNECT: {
      std::string data = msg_rsp.substr(1, msg_rsp.size() - 1);
      std::string delimitor = " ";
      std::vector<std::string> sdata = SplitString(data, delimitor);
      std::string id_con = sdata.at(0);

      FrameServerProcessorConnection connection;
      error = ssp_coordinator.GetConnectionInfo(id_con, connection, error_msg);

      zmq::message_t answer;
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_DISCONNECT error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      ProcessorInstance pi = connection.processor;
      FrameServerInstance fsi = connection.frameserver;

      std::string id_fsi_request = fsi.zmq_id;
      std::string id_pi_request = pi.zmq_id;
      zmq::message_t request;

      // TODO: decide if the frame source should stop if the processor
      // disconnects

      /*
      error = ssp_coordinator.Stop(id_con, error_msg);
      if (error != 0) {
        spdlog::info("SSP_MESSAGE_DISCONNECT info " + std::to_string(error) +
            "\": " + error_msg + "\"");
        break;
      }

      zmq::message_t id_fsi_msg = BuildMessage(id_fsi_request);


      std::string stop_msg = std::string(1, char(SSP_MESSAGE_STOP));
      zmq::message_t request = BuildMessage(stop_msg);

      coor_socket.send(id_fsi_msg, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(request);
       */

      error = ssp_coordinator.Disconnect(id_con, error_msg);
      if (error != 0) {
        spdlog::error("SSP_MESSAGE_DISCONNECT error " + std::to_string(error) +
                      "\": " + error_msg + "\"");
        answer = BuildErrorMessage(error, error_msg);
        coor_socket.send(id_request, ZMQ_SNDMORE);
        coor_socket.send(emp_request, ZMQ_SNDMORE);
        coor_socket.send(answer);
        break;
      }

      std::string disc_msg = std::string(1, char(SSP_MESSAGE_DISCONNECT));

      request = BuildMessage(disc_msg);

      zmq::message_t id_pi_msg = BuildMessage(id_pi_request);

      coor_socket.send(id_pi_msg, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(request);

      spdlog::info("SSP_MESSAGE_STOP " + id_con + " ok");
      answer = BuildOKMessage();

      coor_socket.send(id_request, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(answer);
      break;
    }
    case SSP_MESSAGE_DUMMY: {
      break;
    }
    default: {
      spdlog::error("UNKNOWN REQUEST");
      error = -1;
      error_msg = "UNKNOWN REQUEST";
      zmq::message_t answer = BuildErrorMessage(error, error_msg);
      coor_socket.send(id_request, ZMQ_SNDMORE);
      coor_socket.send(emp_request, ZMQ_SNDMORE);
      coor_socket.send(answer);
      break;
    }
    }
  }

  return 0;
}