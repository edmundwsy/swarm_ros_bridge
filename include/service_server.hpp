/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 *
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 *
 * @version 1.0
 * @date 2023-01-01
 *
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 *
 */

#ifndef __SERVICE_SERVER__
#define __SERVICE_SERVER__
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/service_callback_helper.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <thread>
#include <zmqpp/zmqpp.hpp>

// TODO: how to define ROS service in such header file?

/**
 * @class SwarmServiceServer
 * @brief Listen for ZeroMQ requests and forward them to ROS service, convert the response to ZeroMQ
 * reply
 */
class SwarmServiceServer {
 public:
  SwarmServiceServer(ros::NodeHandle  &nh,
                     const std::string service_name,
                     const std::string zmq_conn_str)
      : nh_(nh), name_(service_name), socket_(context_, zmqpp::socket_type::reply) {
    socket_.bind(zmq_conn_str);
  }

  ~SwarmServiceServer() { socket_.close(); }

  template <typename ServiceType>
  void spin() {
    while (ros::ok()) {
      // Receive a ZeroMQ request
      zmqpp::message zmq_req;
      socket_.receive(zmq_req);

      // Create a ServiceEvent to hold the deserialized request
      ServiceType srv;

      // Deserialize the ZeroMQ message into the request part of the ServiceEvent
      std::string                 req_str = zmq_req.get(0);
      ros::serialization::IStream stream((uint8_t *)req_str.data(), req_str.size());
      ros::serialization::deserialize(stream, srv.request);

      // Send the deserialized ROS request to the ROS server
      ros::ServiceClient client = nh_.serviceClient<ServiceType>(name_);
      if (!client.call(srv)) {
        ROS_ERROR("[SwarmBridge] Service call failed");
      }

      // Serialize the response part of the ServiceEvent back into a ZeroMQ message
      uint32_t             serial_size = ros::serialization::serializationLength(srv.response);
      std::vector<uint8_t> buffer(serial_size);
      ros::serialization::OStream ostream(&buffer[0], serial_size);
      ros::serialization::serialize(ostream, srv.response);

      // Send the response
      zmqpp::message zmq_res;
      zmq_res.add_raw(buffer.data(), buffer.size());
      socket_.send(zmq_res);
    }
  }

 private:
  ros::NodeHandle nh_;
  std::string     name_;
  zmqpp::context  context_;
  zmqpp::socket   socket_;
};

#endif  // __SERVICE_SERVER__
