#include <ctime>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <dynamixel++/dynamixel++.h>

using namespace dynamixelplusplus;

static uint16_t const MX28_ControlTable_Torque_Enable   =  64;
static uint16_t const MX28_ControlTable_GoalPosition    = 116;
static uint16_t const MX28_ControlTable_PresentPosition = 132;

int main(int argc, char **argv) try
{
  size_t      const DEFAULT_BAUD_RATE   = 115200;
  std::string const DEFAULT_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";

  size_t baud_rate{DEFAULT_BAUD_RATE};
  std::string device_name{DEFAULT_DEVICE_NAME};

  if (argc > 1) {
    device_name = std::string(argv[1]);
  }
  if (argc > 2) {
    std::stringstream baud_rate_ss;
    baud_rate_ss << argv[2];
    baud_rate_ss >> baud_rate;
  }

  std::cout << "configured for \"" << device_name << "\" with a baud rate of \"" << baud_rate << "\"." << std::endl;

  Dynamixel dynamixel_ctrl(device_name,
                           Dynamixel::Protocol::V2_0,
                           baud_rate);

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

  /* Enable torque. */
  std::map<Dynamixel::Id, uint8_t> torque_on_data_map;
  for (auto id : id_vect) torque_on_data_map[id] = 1;
  dynamixel_ctrl.syncWrite(MX28_ControlTable_Torque_Enable, torque_on_data_map);

  /* Set goal position. */
  std::map<Dynamixel::Id, uint32_t> goal_position_data_map;
  srand((unsigned) time(NULL));
  for (auto id : id_vect) goal_position_data_map[id] = (rand() % 4096);
  dynamixel_ctrl.syncWrite(MX28_ControlTable_GoalPosition, goal_position_data_map);

  /* Read current position. */
  std::map<Dynamixel::Id, uint32_t> const position_map = dynamixel_ctrl.syncRead<uint32_t>(MX28_ControlTable_PresentPosition, id_vect);

  for (auto [id, position_raw] : position_map)
    std::cout << "Dynamixel MX28 servo #" << static_cast<int>(id) << ": " << position_raw << std::endl;

  return EXIT_SUCCESS;
}
catch (CommunicationError const & e)
{
  std::cerr << "CommunicationError caught: " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (StatusError const & e)
{
  std::cerr << "StatusError caught: " << e.what() << std::endl;
  return EXIT_FAILURE;
}
