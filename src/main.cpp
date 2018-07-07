#include <ros/ros.h>
#include "serial_logger/serial_logger_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logger");
  SerialLoggerNode node;

  node.Run();

  return 0;
}
