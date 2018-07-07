#ifndef SERIAL_LOGGER_SERIAL_LOGGER_NODE_H
#define SERIAL_LOGGER_SERIAL_LOGGER_NODE_H

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// Open Group Libraries
// Unix API for terminal I/O, see: http://pubs.opengroup.org/onlinepubs/7908799/xsh/termios.h.html
#include <termios.h>
// UNIX standard function definitions, see: http://pubs.opengroup.org/onlinepubs/7908799/xsh/unistd.h.html
#include <unistd.h>
// File control definitions, see: http://pubs.opengroup.org/onlinepubs/000095399/basedefs/fcntl.h.html
#include <fcntl.h>
// Error number definitions, see: http://pubs.opengroup.org/onlinepubs/000095399/basedefs/errno.h.html
#include <errno.h>

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
