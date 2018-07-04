/**
Software License Agreement (proprietary)

\file      main.cpp
\authors   Jeremy Xu <jxu@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include <ros/ros.h>
#include "serial_logger/serial_logger_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logger");
  SerialLoggerNode node;

  node.Run();

  return 0;
}
