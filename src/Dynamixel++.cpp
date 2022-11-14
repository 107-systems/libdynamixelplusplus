/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <dynamixel++/Dynamixel++.h>

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixelplusplus
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Dynamixel::Dynamixel(std::string const & device_name,
                     Protocol const protocol_version,
                     int const baudrate)
: _port_handler{dynamixel::PortHandler::getPortHandler(device_name.c_str())}
, _packet_handler{dynamixel::PacketHandler::getPacketHandler(static_cast<float>(protocol_version))}
{
  if (!_port_handler->openPort())
    throw std::runtime_error("'PortHandler::openPort()' failed.");

  if (!_port_handler->setBaudRate(baudrate))
    throw std::runtime_error("'PortHandler::setBaudRate()' failed.");
}

Dynamixel::~Dynamixel()
{
  _port_handler->closePort();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTION
 **************************************************************************************/

std::tuple<Dynamixel::Error, Dynamixel::IdVect> Dynamixel::broadcastPing()
{
  IdVect servo_id_vect;

  if (int const res = _packet_handler->broadcastPing(_port_handler.get(), servo_id_vect);
      res != COMM_SUCCESS)
    return std::make_tuple(Error::BroadcastPing, servo_id_vect);

  return std::make_tuple(Error::None, servo_id_vect);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

Dynamixel::Error Dynamixel::syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data)
{
  dynamixel::GroupSyncWrite group_sync_write(_port_handler.get(), _packet_handler.get(), start_address, data_length);

  for(auto [id, data_ptr] : data)
  {
    if (!group_sync_write.addParam(id, data_ptr))
      return Error::AddParam;
  }

  if (int res = group_sync_write.txPacket();
      res != COMM_SUCCESS)
    return Error::TxPacket;

  group_sync_write.clearParam();

  return Error::None;
}

std::tuple<Dynamixel::Error, Dynamixel::SyncReadDataVect> Dynamixel::syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect)
{
  SyncReadDataVect data_vect;

  dynamixel::GroupSyncRead group_sync_read(_port_handler.get(), _packet_handler.get(), start_address, data_length);

  for(auto id : id_vect)
  {
    if (!group_sync_read.addParam(id))
      return std::make_tuple(Error::AddParam, data_vect);
  }

  if (int res = group_sync_read.txRxPacket();
      res != COMM_SUCCESS)
    return std::make_tuple(Error::TxRxPacket, data_vect);

  for(auto id : id_vect)
  {
    uint8_t dxl_error = 0;
    if (group_sync_read.getError(id, &dxl_error))
      throw std::runtime_error("'GroupSyncRead::getError(%d)' returns " + std::string(_packet_handler->getRxPacketError(dxl_error)));
  }

  for(auto id : id_vect)
  {
    if (group_sync_read.isAvailable(id, start_address, data_length))
      data_vect.push_back(std::make_tuple(id, group_sync_read.getData(id, start_address, data_length)));
    else
      data_vect.push_back(std::make_tuple(id, std::nullopt));
  }

  group_sync_read.clearParam();

  return std::make_tuple(Error::None, data_vect);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */
