/**
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

  std::stringstream id_list;
  for (auto id : id_vect)
    id_list << static_cast<int>(id) << " ";

  std::cout << "detected Dynamixel servos: { %s}" << id_list.str() << std::endl;

  /* Read the current angle from all those servos
   * and print it to std::cout.
   */
  std::map<Dynamixel::Id, uint32_t> position_map;
  auto err_read = dynamixel_ctrl.syncRead(MX28_ControlTable_PresentPosition, id_vect, position_map);
  if (err_read != Dynamixel::Error::None)
  {
    std::cerr << "'syncRead' failed with error code " << static_cast<int>(err_read) << std::endl;
    return EXIT_FAILURE;
  }
  
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
