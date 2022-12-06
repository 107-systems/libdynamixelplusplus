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

static uint16_t const MX28_ControlTable_PresentPosition = 132;

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

  /* Read the current angle from all those servos
   * and print it to std::cout.
   */
  std::map<Dynamixel::Id, uint32_t> position_map;
  dynamixel_ctrl.syncRead(MX28_ControlTable_PresentPosition, id_vect, position_map);

  for (auto [id, position_raw] : position_map)
  {
    float const position_deg = static_cast<float>(position_raw) * 360.0f / 4096;
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
