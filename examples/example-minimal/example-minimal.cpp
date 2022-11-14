#include <cstdlib>
#include <iostream>
#include <dynamixel++/Dynamixel++.h>

using namespace dynamixelplusplus;

static uint16_t const MX28_ControlTable_Torque_Enable = 64;
static uint16_t const MX28_ControlTable_GoalPosition  = 116;

int main(int argc, char **argv)
{
  Dynamixel dynamixel_ctrl("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0",
                           Dynamixel::Protocol::V2_0,
                           115200);

  Dynamixel::IdVect const id_vect{1,2,3,4};

  /* Enable torque. */
  std::map<Dynamixel::Id, uint8_t> torque_on_data_map;
  for (auto id : id_vect) torque_on_data_map[id] = 1;
  dynamixel_ctrl.syncWrite(MX28_ControlTable_Torque_Enable, torque_on_data_map);

  /* Set goal position. */
  std::map<Dynamixel::Id, uint32_t> goal_position_data_map;
  for (auto id : id_vect) goal_position_data_map[id] = (id - 1) * 1024;
  dynamixel_ctrl.syncWrite(MX28_ControlTable_Torque_Enable, goal_position_data_map);

  return EXIT_SUCCESS;
}
