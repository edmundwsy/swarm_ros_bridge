#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/service_callback_helper.h>
#include <std_msgs/String.h>
#include <string>
#include <zmqpp/zmqpp.hpp>

/**
 * @class SwarmServiceClient
 * @brief Listen for ZeroMQ responses and forward them to ROS client, convert the ZeroMQ reply to
 * ROS client
 *
 */
template <typename ServiceType>
class SwarmServiceClient {
 public:
  SwarmServiceClient(ros::NodeHandle& nh, std::string service_name, std::string zmq_conn_str)
      : nh_(nh), socket_(context_, zmqpp::socket_type::request) {
    std::cout << "Connecting to " << zmq_conn_str << std::endl;
    srv_ = nh_.advertiseService(service_name, &SwarmServiceClient::serviceCallback, this);
    socket_.connect(zmq_conn_str);
  }

  ~SwarmServiceClient() {
    socket_.close();
    srv_.shutdown();
  }

  template <typename ServiceReq, typename ServiceRes>
  bool serviceCallback(ServiceReq& req, ServiceRes& res) {
    // Serialize the request into a ZeroMQ message
    uint32_t                    serial_size = ros::serialization::serializationLength(srv.request);
    std::vector<uint8_t>        buffer(serial_size);
    ros::serialization::OStream ostream(&buffer[0], serial_size);
    ros::serialization::serialize(ostream, req);

    // Send the request to remote
    zmqpp::message zmq_req;
    zmq_req.add_raw(buffer.data(), buffer.size());
    socket_.send(zmq_req);

    // Receive the response from remote
    zmqpp::message zmq_res;
    socket_.receive(zmq_res);

    // Deserialize the ZeroMQ message into the response
    std::string                 res_str = zmq_res.get(0);
    ros::serialization::IStream stream((uint8_t*)res_str.data(), res_str.size());
    ros::serialization::deserialize(stream, res);

    // Return true if the call was successful
    return true;
  }

 private:
  ros::NodeHandle    nh_;
  ros::ServiceServer srv_;
  zmqpp::context     context_;
  zmqpp::socket      socket_;
};
