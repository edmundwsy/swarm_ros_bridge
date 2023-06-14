/**
 * @file server_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 *
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 *
 * Core Idea: It would create the receving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 *
 * @version 1.0
 * @date 2023-01-01
 *
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 *
 */
#include "ros_srv_clt.hpp"
#include "service_client.hpp"

// IP
XmlRpc::XmlRpcValue                ip_xml;
std::map<std::string, std::string> ip_map;  // map host name and IP

XmlRpc::XmlRpcValue service_info_xml;

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_service_bridge");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;

  std::string ns = ros::this_node::getNamespace();  // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  //************************ Parse configuration file **************************
  // get hostnames and IPs
  if (nh.getParam("IP", ip_xml) == false) {
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return 1;
  }

  std::cout << "-------------IP------------" << std::endl;
  for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter) {
    std::string host_name = iter->first;
    std::string host_ip   = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end()) {  // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    ip_map[host_name] = host_ip;
  }

  std::cout << "-------------service------------" << std::endl;
  if (nh.getParam("services", service_info_xml)) {
    ROS_ASSERT(service_info_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
  } else {
    ROS_ERROR("[bridge node] No service_name found in the configuration!");
    return 1;
  }

  ROS_ASSERT(service_info_xml[0].getType() == XmlRpc::XmlRpcValue::TypeStruct);
  XmlRpc::XmlRpcValue service_info = service_info_xml[0];
  std::string         service_name = service_info["srv_name"];
  std::string         service_type = service_info["srv_type"];
  std::string         server_IP    = ip_map[service_info["server_IP"]];
  int                 server_port  = service_info["port"];

  std::cout << "service_name: " << service_name << std::endl;
  std::cout << "service_type: " << service_type << std::endl;
  std::cout << "server_IP: " << server_IP << std::endl;
  std::cout << "server_port: " << server_port << std::endl;

  const std::string url = "tcp://" + server_IP + ":" + std::to_string(server_port);

  // auto zmq_client = SwarmServiceClient<SRV_CLASS>(nh, service_name, url);
  auto client_ptr = std::make_shared<SwarmServiceClient<SRV_CLASS>>(nh, service_name, url);

  // auto      client = std::make_shared<SwarmServiceClient>(url);
  // SRV_CLASS req;
  // std::cout << "request: " << req.request << std::endl;
  // if (client->call(req)) {
  //   ROS_INFO("Service call success!");
  //   std::cout << "response: " << req.response << std::endl;
  // } else {
  //   ROS_ERROR("Service call failed!");
  // }

  return 0;
}
