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
  std::vector<Id> id_vect;
  std::vector<T> value_vect;
  for (auto [id, val] : data)
  {
    id_vect.push_back(id);
    value_vect.push_back(val);
  }

  /* This 2-step dance is necessary because we need to pass a pointer
   * to a local variable which still needs to exist in scope until
   * syncWrite has been fully executed.
   */
  SyncWriteDataVect data_vect;
  for (size_t i = 0; i < id_vect.size(); i++)
  {
    SyncWriteData const d = std::make_tuple(id_vect[i], &value_vect[i]);
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
