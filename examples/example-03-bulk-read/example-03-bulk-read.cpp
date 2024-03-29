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
 * CONST
 **************************************************************************************/

static uint16_t const MX28_ControlTable_HardwareErrorStatus = 70;
static uint16_t const MX28_ControlTable_PresentPosition     = 132;

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

  /* Send a broadcast ping to determine which
   * servos are available (more convenient for
   * the user compared to manually editing this
   * file).
   */

  auto const id_vect = dynamixel_ctrl.broadcastPing();
  if (id_vect.empty()) {
    std::cerr << "No dynamixel servos detected." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "detected Dynamixel servos: ";
  for (auto id : id_vect)
    std::cout << static_cast<int>(id) << " ";
  std::cout << std::endl;

  /* Read the current angle from all those servos
   * and print it to std::cout.
   */
  Dynamixel::BulkReadDataMap position_map;
  try
  {
    Dynamixel::BulkReadRequestVect bulk_read_req;
    for (auto id : id_vect)
      bulk_read_req.push_back(std::make_tuple(id, MX28_ControlTable_PresentPosition, sizeof(uint32_t)));

    position_map = dynamixel_ctrl.bulkRead(bulk_read_req);
  }
  catch (dynamixelplusplus::HardwareAlert const & e)
  {
    uint8_t const hw_err_code = dynamixel_ctrl.read<uint8_t>(MX28_ControlTable_HardwareErrorStatus, e.id());
    if (hw_err_code) {
      std::cerr << "HardwareAlert for servo #" << static_cast<int>(e.id()) << "caught: " << static_cast<int>(hw_err_code) << std::endl;
      dynamixel_ctrl.reboot(e.id());
    }
  }

  for (auto [id, position_raw] : position_map)
  {
    float const position_deg = static_cast<float>(std::get<uint32_t>(position_raw)) * 360.0f / 4096;
    std::cout << "Dynamixel MX28 servo #"
              << static_cast<int>(id)
              << ": "
              << position_deg
              << " DEG"
              << std::endl;
  }

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
