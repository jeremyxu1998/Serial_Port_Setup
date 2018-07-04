/**
Software License Agreement (proprietary)

\file      serial_logger_node.h
\authors   Jeremy Xu <jxu@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#ifndef SERIAL_LOGGER_SERIAL_LOGGER_NODE_H
#define SERIAL_LOGGER_SERIAL_LOGGER_NODE_H

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <termios.h>    // Unix API for terminal I/O
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions

class SerialLoggerNode
{
public:
  SerialLoggerNode();
  ~SerialLoggerNode();
  void Run();

private:
  ros::NodeHandle n_;
  ros::Publisher serial_pub_;
  std_msgs::String msg_;
  int USB_port;
  struct termios tty;
};
#endif  // SERIAL_LOGGER_SERIAL_LOGGER_NODE_H
