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

static uint16_t const MX28_ControlTable_Firmware_Version = 6;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  Dynamixel dynamixel_ctrl("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0",
                           Dynamixel::Protocol::V2_0,
                           115200);

  /* Send a broadcast ping to determine which
   * servos are available (more convenient for
   * the user compared to manually editing this
   * file).
   */

  auto [err_id, id_vect] = dynamixel_ctrl.broadcastPing();
  if (err_id != Dynamixel::Error::None) {
    std::cerr << "'broadcastPing' failed with error code " << static_cast<int>(err_id) << std::endl;
    return EXIT_FAILURE;
  }

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
    uint8_t firmware_version = 0;
    auto const err_read = dynamixel_ctrl.read(MX28_ControlTable_Firmware_Version, id, firmware_version);

    if (err_read != Dynamixel::Error::None) {
      std::cerr << "'read' failed with error code " << static_cast<int>(err_read) << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Servo #" << static_cast<int>(id)
              << " firmware version rev. " << static_cast<int>(firmware_version) << std::endl;
  }

  return EXIT_SUCCESS;
}
