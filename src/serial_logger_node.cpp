/**
Software License Agreement (proprietary)

\file      serial_logger_node.cpp
\authors   Jeremy Xu <jxu@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include "serial_logger/serial_logger_node.h"
#include <string>

SerialLoggerNode::SerialLoggerNode()
{
  // The publisher
  serial_pub_ = n_.advertise<std_msgs::String>("serial", 100);

  // Serial port initialization
  std::string port;
  int baudrate, baudrate_in, charsize;
  bool stopbits, parity_enable, xonxoff, rtscts;
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  ros::param::param<int>("~baudrate", baudrate_in, 115200);
  ros::param::param<int>("~charsize", charsize, 8);
  ros::param::param<bool>("~stopbits", stopbits, false);
  ros::param::param<bool>("~parityenable", parity_enable, false);
  ros::param::param<bool>("~xonxoff", xonxoff, false);
  ros::param::param<bool>("~rtscts", rtscts, false);

  // O_RDWR - Read/Write access to serial port, O_NOCTTY - No terminal will control the process
  USB_port = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (USB_port < 0) ROS_ERROR_STREAM("Error opening " << port << ": " << errno);
  memset (&tty, 0, sizeof tty);

  // The Bxxx baudrate is replaced by internal code using #define
  // Therefore the simpliest way to transfer is using a switch statement
  switch (baudrate_in)
  {
    case 9600:
      baudrate = B9600;  break;
    case 19200:
      baudrate = B19200;  break;
    case 57600:
      baudrate = B57600;  break;
    case 115200:
      baudrate = B115200;  break;
    default:
      ROS_ERROR_STREAM("Unknown baud rate, probably need to extend case statement");
  }
  cfsetospeed(&tty, (speed_t)baudrate);
  cfsetispeed(&tty, (speed_t)baudrate);

  tty.c_cflag  &=  ~CSIZE;  // CSIZE is a mask for the number of bits per character
  switch (charsize)
  {
    case 8:
      tty.c_cflag  |=  CS8;  break;  // Set to 8 bits per character
    case 7:
      tty.c_cflag  |=  CS7;  break;
    case 6:
      tty.c_cflag  |=  CS6;  break;
    case 5:
      tty.c_cflag  |=  CS5;  break;
    default:
      ROS_ERROR_STREAM("Setting character size failed");
  }

  if (parity_enable) tty.c_cflag  |=  PARENB;
  else tty.c_cflag  &=  ~PARENB;  // No parity bit is added to the output characters

  if (stopbits) tty.c_cflag  |=  CSTOPB;  // Use two stop-bits
  else tty.c_cflag  &=  ~CSTOPB;  // Only one stop-bit is used

  if (xonxoff) tty.c_iflag  |=  (IXON | IXOFF);
  else tty.c_iflag  &=  ~(IXON | IXOFF);  // Disable software flow control

  if (rtscts) tty.c_cflag  |=  CRTSCTS;
  else tty.c_cflag  &=  ~CRTSCTS;  // Disable RTS/CTS (hardware) flow control

  tty.c_cflag  |=  CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag  &=  ~(ICANON | ECHO | ECHOE | ISIG);  // Disabling canonical mode and make raw
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout
  cfmakeraw(&tty);  // Raw mode: sets the terminal to something like the "raw" mode of the old Version 7 terminal driver

  // Flush port, then apply attributes
  tcflush(USB_port, TCIFLUSH);
  if (tcsetattr (USB_port, TCSANOW, &tty) != 0) ROS_ERROR_STREAM("Error " << errno << " from tcsetattr");
}

SerialLoggerNode::~SerialLoggerNode()
{
  close(USB_port);
}

void SerialLoggerNode::Run()
{
  while (ros::ok())
  {
    ssize_t num_bytes_read = 0;
    int response_index = 0;
    char buf = '\0';  // Temporary storage for the current character read
    // Whole response
    char response[1024];
    memset(response, '\0', sizeof response);

    // Because we don't know length of each line of message, we read in char by char until we meet carriage return
    do
    {
      num_bytes_read = read(USB_port, &buf, 1);
      snprintf(&response[response_index], 100, "%c", buf);
      response_index += num_bytes_read;
    }
    while (buf != '\n' && num_bytes_read > 0);

    msg_.data = (std::string(response)).substr(0, response_index-2);  // Convert chararray into string and discard \r\n
    serial_pub_.publish(msg_);
    ros::spinOnce();
  }
}
