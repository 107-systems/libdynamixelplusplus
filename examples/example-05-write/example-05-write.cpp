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

void turnLedOn (Dynamixel & dynamixel_ctrl, Dynamixel::Id const id);
void turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id);

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

  /* The following code turns the LEDs of all servos on,
   * keeps them turned on for a certain duration of time,
   * then turns them off again. This is repeated until the
   * process is killed with Ctrl + C.
   */
  for (;;)
  {
    for (auto id : id_vect)
      turnLedOn(dynamixel_ctrl, id);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for (auto id : id_vect)
      turnLedOff(dynamixel_ctrl, id);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return EXIT_SUCCESS;
}
catch (dynamixelplusplus::CommunicationError const & e)
{
  std::cerr << "CommunicationError caught: " << e.what() << std::endl;
  return EXIT_FAILURE;
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void turnLedOn(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id)
{
  uint8_t const LED_ON = 1;
  return dynamixel_ctrl.write(MX28_ControlTable_LED, id, LED_ON);
}

void turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::Id const id)
{
  uint8_t const LED_OFF = 0;
  return dynamixel_ctrl.write(MX28_ControlTable_LED, id, LED_OFF);
}
