/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixelplusplus
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

template<typename T> std::vector<T> SyncGroup::read(uint16_t const start_address)
{
  std::vector<T> data_vect;
  std::map<Dynamixel::Id, T> const data_map = _dyn_ctrl->syncRead<T>(start_address, _id_vect);

  for (auto [id, val] : data_map)
    data_vect.push_back(val);

  return data_vect;
}

template<typename T> void SyncGroup::write(uint16_t const start_address, T const val)
{
  std::map<Dynamixel::Id, T> data_map;

  for (auto id : _id_vect)
    data_map[id] = val;

  _dyn_ctrl->syncWrite(start_address, data_map);
}

template<typename T> void SyncGroup::write(uint16_t const start_address, std::vector<T> const & val_vect)
{
  assert(val_vect.size() == _id_vect.size());

  std::map<Dynamixel::Id, T> data_map;

  auto id_citer = _id_vect.cbegin();
  auto val_citer = val_vect.cbegin();

  for (; id_citer != _id_vect.cend(); id_citer++, val_citer++)
    data_map[*id_citer] = *val_citer;

  _dyn_ctrl->syncWrite(start_address, data_map);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */
