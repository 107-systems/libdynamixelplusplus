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

Dynamixel::Error turnLedOn (Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect);
Dynamixel::Error turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

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
    if (auto err = turnLedOn(dynamixel_ctrl, id_vect); err != Dynamixel::Error::None) {
      std::cerr << "'turnLedOn' failed with error code " << static_cast<int>(err) << std::endl;
      return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    /* Turn all LEDs off. */
    if (auto err = turnLedOff(dynamixel_ctrl, id_vect); err != Dynamixel::Error::None) {
      std::cerr << "'turnLedOff' failed with error code " << static_cast<int>(err) << std::endl;
      return EXIT_FAILURE;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

Dynamixel::Error turnLedOn(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect)
{
  std::map<Dynamixel::Id, uint8_t> led_on_data_map;

  for (auto id : id_vect)
    led_on_data_map[id] = 1;

  return dynamixel_ctrl.syncWrite(MX28_ControlTable_LED, led_on_data_map);
}

Dynamixel::Error turnLedOff(Dynamixel & dynamixel_ctrl, Dynamixel::IdVect const & id_vect)
{
  std::map<Dynamixel::Id, uint8_t> led_off_data_map;

  for (auto id : id_vect)
    led_off_data_map[id] = 0;

  return dynamixel_ctrl.syncWrite(MX28_ControlTable_LED, led_off_data_map);
}
