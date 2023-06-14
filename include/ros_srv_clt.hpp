/**
 * @file ros_srv_clt.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file for different ROS message type.
 *
 * Core Idea: modify the macros about MSG_TYPEx and MSG_CLASSx,
 *  it will generate template functions for different ros message types.
 *  Remember to add the dependent package in find_package() of ../CMakeLists.txt
 *
 * Note: the sub_cb() and deserialize_pub() are only declared here,
 *  you should define them in you .cpp file according to your need.
 *
 * @version 1.0
 * @date 2023-01-01
 *
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 *
 */
#ifndef __ROS_SRV_CLT__
#define __ROS_SRV_CLT__
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#define SRV_TYPE  "std_srvs/Trigger"
#define SRV_CLASS std_srvs::Trigger

#endif  // __ROS_SRV_CLT__
