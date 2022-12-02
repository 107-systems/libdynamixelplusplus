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
 * FUNCTION DEFINITION
 **************************************************************************************/

template<typename T> int read_n_ByteTxRx(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler *port, uint8_t id, uint16_t address, T * data, uint8_t * error);

template<> inline int read_n_ByteTxRx<uint8_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler *port, uint8_t id, uint16_t address, uint8_t * data, uint8_t * error)
{
  return packet_handler->read1ByteTxRx(port, id, address, data, error);
}
template<> inline int read_n_ByteTxRx<uint16_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler *port, uint8_t id, uint16_t address, uint16_t * data, uint8_t * error)
{
  return packet_handler->read2ByteTxRx(port, id, address, data, error);
}
template<> inline int read_n_ByteTxRx<uint32_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler *port, uint8_t id, uint16_t address, uint32_t * data, uint8_t * error)
{
  return packet_handler->read4ByteTxRx(port, id, address, data, error);
}

template<typename T> int write_n_ByteTxRx(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port, uint8_t id, uint16_t address, T data, uint8_t * error);

template<> inline int write_n_ByteTxRx<uint8_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port, uint8_t id, uint16_t address, uint8_t data, uint8_t * error)
{
  return packet_handler->write1ByteTxRx(port, id, address, data, error);
}
template<> inline int write_n_ByteTxRx<uint16_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port, uint8_t id, uint16_t address, uint16_t data, uint8_t * error)
{
  return packet_handler->write2ByteTxRx(port, id, address, data, error);
}
template<> inline int write_n_ByteTxRx<uint32_t>(dynamixel::PacketHandler * packet_handler, dynamixel::PortHandler * port, uint8_t id, uint16_t address, uint32_t data, uint8_t * error)
{
  return packet_handler->write4ByteTxRx(port, id, address, data, error);
}

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

template<typename T> Dynamixel::Error Dynamixel::read(uint16_t const start_address, Id const id, T & val)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  uint8_t error = 0;
  if (auto const rc = read_n_ByteTxRx(_packet_handler, _port_handler, id, start_address, &val, error);
      rc == COMM_SUCCESS)
    return Error::None;

  return Error::Read_n_ByteTxRx;
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

template<typename T> Dynamixel::Error Dynamixel::write(uint16_t const start_address, Id const id, T const val)
{
  static_assert(std::is_same<T, uint8_t>::value  ||
                std::is_same<T, uint16_t>::value ||
                std::is_same<T, uint32_t>::value, "Only uint8_t, uint16_t and uint32_t are allowed parameters.");

  uint8_t error = 0;
  if (auto const rc = write_n_ByteTxRx(_packet_handler, _port_handler, id, start_address, &val, error);
    rc == COMM_SUCCESS)
    return Error::None;

  return Error::Write_n_ByteTxRx;
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
