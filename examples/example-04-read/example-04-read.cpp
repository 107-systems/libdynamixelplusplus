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

#include <iostream>

#include <dynamixel++/Dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace dynamixelplusplus;

/**************************************************************************************
 * CONST
 **************************************************************************************/

static uint16_t const MX28_ControlTable_Firmware_Version    =  6;
static uint16_t const MX28_ControlTable_HardwareErrorStatus = 70;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  Dynamixel dynamixel_ctrl("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0",
                           Dynamixel::Protocol::V2_0,
                           115200);

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

  /* Read the firmware version from every single
   * servo and display it via stdout.
   */

  for (auto id: id_vect)
  {
    try
    {
      uint8_t const firmware_version = dynamixel_ctrl.read<uint8_t>(MX28_ControlTable_Firmware_Version, id);

      std::cout << "Servo #" << static_cast<int>(id)
                << " firmware version rev. " << static_cast<int>(firmware_version) << std::endl;
    }
    catch (dynamixelplusplus::HardwareAlert const & e)
    {
      uint8_t const hw_err_code = dynamixel_ctrl.read<uint8_t>(MX28_ControlTable_HardwareErrorStatus, e.id());
      if (hw_err_code) {
        std::cerr << "HardwareAlert for servo #" << static_cast<int>(e.id()) << "caught: " << static_cast<int>(hw_err_code) << std::endl;
        dynamixel_ctrl.reboot(e.id());
      }

    }
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
