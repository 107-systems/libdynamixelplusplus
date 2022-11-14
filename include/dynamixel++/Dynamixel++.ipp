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

template<typename T> Dynamixel::Error Dynamixel::syncRead(uint16_t const start_address, Id const id, T & val)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  std::map<Id, T> val_map;
  auto err = syncRead(start_address, IdVect{id}, val_map);

  if (val_map.count(id) > 0)
    val = val_map.at(id);

  return err;
}

template<typename T> Dynamixel::Error Dynamixel::syncRead(uint16_t const start_address, IdVect const & id_vect, std::map<Id, T> & val_map)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  auto [err, sync_read_data_vect] = syncRead(start_address, sizeof(T), id_vect);

  for (auto [id, opt_data] : sync_read_data_vect)
  {
    if (opt_data.has_value())
      val_map[id] = static_cast<T>(opt_data.value());
  }

  return err;
}

template<typename T> Dynamixel::Error Dynamixel::syncWrite(uint16_t const start_address, Id const id, T const val)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  std::map<Id, T> val_map;
  val_map[id] = val;
  return syncWrite(start_address, val_map);
}

template<typename T> Dynamixel::Error Dynamixel::syncWrite(uint16_t const start_address, std::map<Id, T> const & val_map)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  /* Convert the functions input data into the required
   * format to feed to the Dynamixel SDK.
   */
  std::vector<Id> id_vect;
  std::vector<T> value_vect;
  for (auto [id, val] : val_map)
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
    SyncWriteData const d = std::make_tuple(id_vect[i], reinterpret_cast<uint8_t *>(&value_vect[i]));
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
