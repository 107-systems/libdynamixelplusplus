/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

#ifndef DYNAMIXEL_SYNCGROUP_H
#define DYNAMIXEL_SYNCGROUP_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "Dynamixel.hpp"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixelplusplus
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SyncGroup
{
public:
  SyncGroup(SharedDynamixel dyn_ctrl, Dynamixel::IdVect const & id_vect)
  : _dyn_ctrl{dyn_ctrl}
  , _id_vect{id_vect}
  { }


  template<typename T> std::vector<T> read (uint16_t const start_address);
  template<typename T> void           write(uint16_t const start_address, T const val);
  template<typename T> void           write(uint16_t const start_address, std::vector<T> const & val_vect);


private:
  SharedDynamixel _dyn_ctrl;
  Dynamixel::IdVect const _id_vect;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

/**************************************************************************************
 * TEMPLATE IMPLEMENTATION
 **************************************************************************************/

#include "SyncGroup.ipp"

#endif /* DYNAMIXEL_SYNCGROUP_H */
