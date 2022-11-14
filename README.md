<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `libdynamixelplusplus`
====================================
[![Spell Check status](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/libdynamixelplusplus/actions/workflows/spell-check.yml)

A comfortable modern C++17 wrapper for the Robotis [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) [v3.7.60](https://github.com/ROBOTIS-GIT/DynamixelSDK/releases/tag/3.7.60).

### How-to-build
```bash
git clone https://github.com/107-systems/libdynamixelplusplus && cd libdynamixelplusplus
mkdir build && cd build
cmake .. && make
```
or
```bash
cmake -DBUILD_EXAMPLES=ON .. && make
```

### How-to-use
```C++
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
```
