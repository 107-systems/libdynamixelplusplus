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

#include <dynamixel++/dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace dynamixelplusplus;

/**************************************************************************************
 * CONST
 **************************************************************************************/

static uint16_t const MX28_ControlTable_LED                 = 65;
static uint16_t const MX28_ControlTable_HardwareErrorStatus = 70;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void turnLedOn (Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect);
void turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect);

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
    try
    {
      turnLedOn(dynamixel_ctrl, id_vect);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      turnLedOff(dynamixel_ctrl, id_vect);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void turnLedOn(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect)
{
  std::map<Dynamixel::Id, uint8_t> led_on_data_map;

  for (auto id : id_vect)
    led_on_data_map[id] = 1;

  dynamixel_ctrl.syncWrite(MX28_ControlTable_LED, led_on_data_map);
}

void turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect)
{
  std::map<Dynamixel::Id, uint8_t> led_off_data_map;

  for (auto id : id_vect)
    led_off_data_map[id] = 0;

  dynamixel_ctrl.syncWrite(MX28_ControlTable_LED, led_off_data_map);
}
