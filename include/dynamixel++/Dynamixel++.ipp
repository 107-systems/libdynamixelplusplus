/**
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
 * CLASS DECLARATION
 **************************************************************************************/

template<typename T> Dynamixel::Error Dynamixel::syncWrite(uint16_t const start_address, Id const id, T const val)
{
  std::tuple<Id, T> const data = std::make_tuple(id, val);
  std::vector<std::tuple<Id, T>> const data_vect{data};
  return syncWrite(start_address, data_vect);
}

template<typename T> Dynamixel::Error Dynamixel::syncWrite(uint16_t const start_address, std::vector<std::tuple<Id, T>> const & data)
{
  /* Convert the functions input data into the required
   * format to feed to the Dynamixel SDK.
   */
  SyncWriteDataVect data_vect;
  for (auto [id, val] : data) {
    SyncWriteData const d = std::make_tuple(id, &val);
    data_vect.push_back(d);
  }

  /* Call the actual sync write API invoking the underlying
   * DynamixelSDK sync write API.
   */
  return syncWrite(start_address, sizeof(T), data_vect);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */
