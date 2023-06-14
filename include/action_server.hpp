
#ifndef __ACTION_SERVER__
#define __ACTION_SERVER__
#include <actionlib/server/simple_action_server.h>
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

template <typename ActionType>
class ZeroMQActionServer {
 public:
  ZeroMQActionServer(ros::NodeHandle nh, std::string name, std::string zmq_conn_str)
      : nh_(nh)
      , as_(nh_, name, boost::bind(&ZeroMQActionServer::executeCb, this, _1), false)
      , goal_socket_(context_, zmqpp::socket_type::rep)
      , result_socket_(context_, zmqpp::socket_type::pub)
      , feedback_socket_(context_, zmqpp::socket_type::pub) {
    as_.start();
    goal_socket_.bind(zmq_conn_str);  // TODO: replace with actual addresses
    result_socket_.bind(zmq_conn_str);
    feedback_socket_.bind(zmq_conn_str);
  }

  void executeCb(const typename ActionType::_action_goal_type::ConstPtr& goal) {
    // TODO: goal execution logic
    //       send feedback with feedback_socket_
    //       when done, send result with result_socket_
  }

  void spin() {
    while (ros::ok()) {
      zmqpp::message zmq_msg;
      if (goal_socket_.receive(zmq_msg)) {
        // TODO: deserialize the ZeroMQ message into a ROS goal message
        //       and publish it on the action server

        std::string                            goal_str = zmq_msg.get(0);
        ros::serialization::IStream            stream((uint8_t*)goal_str.data(), goal_str.size());
        typename ActionType::_action_goal_type goal;
        ros::serialization::deserialize(stream, goal);
      }
    }
  }

 private:
  ros::NodeHandle                           nh_;
  actionlib::SimpleActionServer<ActionType> as_;
  zmqpp::context                            context_;
  zmqpp::socket                             goal_socket_;
  zmqpp::socket                             result_socket_;
  zmqpp::socket                             feedback_socket_;
};

#endif  // __ACTION_SERVER__
