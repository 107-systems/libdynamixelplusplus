#include <cstdlib>
#include <iostream>
#include <dynamixel++/Dynamixel++.h>

using namespace dynamixelplusplus;

static uint16_t const MX28_ControlTable_Torque_Enable   =  64;
static uint16_t const MX28_ControlTable_GoalPosition    = 116;
static uint16_t const MX28_ControlTable_PresentPosition = 132;

int main(int argc, char **argv) try
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

  /* Read current position. */
  std::map<Dynamixel::Id, uint32_t> const position_map = dynamixel_ctrl.syncRead<uint32_t>(MX28_ControlTable_PresentPosition, id_vect);

  for (auto [id, position_raw] : position_map)
    std::cout << "Dynamixel MX28 servo #" << static_cast<int>(id) << ": " << position_raw << std::endl;

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
