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

#include <thread>
#include <iostream>

#include <dynamixel++/Dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace dynamixelplusplus;

/**************************************************************************************
 * CONST
 **************************************************************************************/

static uint16_t const MX28_ControlTable_LED = 65;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

Dynamixel::Error turnLedOn (Dynamixel & dynamixel_ctrl, Dynamixel::Id const id);
Dynamixel::Error turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id);

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

  /* The following code turns the LEDs of all servos on,
   * keeps them turned on for a certain duration of time,
   * then turns them off again. This is repeated until the
   * process is killed with Ctrl + C.
   */
  for (;;)
  {
    /* Turn all LEDs on. */
    for (auto id : id_vect)
    {
      if (auto err = turnLedOn(dynamixel_ctrl, id); err != Dynamixel::Error::None)
      {
        std::cerr << "'turnLedOn' failed with error code " << static_cast<int>(err) << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    /* Turn all LEDs off. */
    for (auto id : id_vect)
    {
      if (auto err = turnLedOff(dynamixel_ctrl, id); err != Dynamixel::Error::None)
      {
        std::cerr << "'turnLedOff' failed with error code " << static_cast<int>(err) << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

Dynamixel::Error turnLedOn(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id)
{
  uint8_t const LED_ON = 1;
  return dynamixel_ctrl.write(MX28_ControlTable_LED, id, LED_ON);
}

Dynamixel::Error turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id)
{
  uint8_t const LED_OFF = 0;
  return dynamixel_ctrl.write(MX28_ControlTable_LED, id, LED_OFF);
}
