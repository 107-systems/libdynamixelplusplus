/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cstdlib>

#include <sstream>
#include <iostream>

#include <dynamixel++/dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace dynamixelplusplus;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  size_t      const DEFAULT_BAUD_RATE   = 115200;
  std::string const DEFAULT_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";

  size_t baud_rate{DEFAULT_BAUD_RATE};
  std::string device_name{DEFAULT_DEVICE_NAME};

  if (argc > 1) {
    device_name = std::string(argv[1]);
  }
  if (argc > 2) {
    std::stringstream baud_rate_ss;
    baud_rate_ss << argv[2];
    baud_rate_ss >> baud_rate;
  }

  std::cout << "configured for \"" << device_name << "\" with a baud rate of \"" << baud_rate << "\"." << std::endl;

  Dynamixel dynamixel_ctrl(device_name,
                           Dynamixel::Protocol::V2_0,
                           baud_rate);

  auto const id_vect = dynamixel_ctrl.broadcastPing();

  std::cout << "detected Dynamixel servos: ";
  for (auto id : id_vect)
    std::cout << static_cast<int>(id) << " ";
  std::cout << std::endl;

  return EXIT_SUCCESS;
}
catch (dynamixelplusplus::CommunicationError const & e)
{
  std::cerr << "CommunicationError caught: " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (dynamixelplusplus::StatusError const & e)
{
  std::cerr << "StatusError caught: " << e.what() << std::endl;
  return EXIT_FAILURE;
}
