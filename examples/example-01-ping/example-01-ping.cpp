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
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  Dynamixel dynamixel_ctrl("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0",
                           Dynamixel::Protocol::V2_0,
                           115200);

  auto [err, id_vect] = dynamixel_ctrl.broadcastPing();
  if (err != Dynamixel::Error::None) {
    std::cerr << "'broadcastPing' failed with error code " << static_cast<int>(err) << std::endl;
    return EXIT_FAILURE;
  }

  std::stringstream id_list;
  for (auto id : id_vect)
    id_list << static_cast<int>(id) << " ";

  std::cout << "detected Dynamixel servos: { %s}" << id_list.str() << std::endl;

  return EXIT_SUCCESS;
}