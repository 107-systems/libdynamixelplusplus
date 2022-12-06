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

Dynamixel::IdVect Dynamixel::broadcastPing()
{
  IdVect servo_id_vect;

  if (int const res = _packet_handler->broadcastPing(_port_handler.get(), servo_id_vect);
      res != COMM_SUCCESS) {
    throw CommunicationError(_packet_handler.get(), res);
  }

  return servo_id_vect;
}

void Dynamixel::reboot(Id const id)
{
  uint8_t error = 0;
  if (int const res = _packet_handler->reboot(_port_handler.get(), id, &error);
      res != COMM_SUCCESS) {
    throw CommunicationError(_packet_handler.get(), res);
  }

  if (error)
    throw StatusError(_packet_handler.get(), error);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Dynamixel::syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data)
{
  dynamixel::GroupSyncWrite group_sync_write(_port_handler.get(), _packet_handler.get(), start_address, data_length);

  for(auto [id, data_ptr] : data)
    group_sync_write.addParam(id, data_ptr);

  if (int const res = group_sync_write.txPacket();
      res != COMM_SUCCESS) {
    throw CommunicationError(_packet_handler.get(), res);
  }
  group_sync_write.clearParam();
}

Dynamixel::SyncReadDataVect Dynamixel::syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect)
{
  SyncReadDataVect data_vect;

  dynamixel::GroupSyncRead group_sync_read(_port_handler.get(), _packet_handler.get(), start_address, data_length);

  for(auto id : id_vect)
    group_sync_read.addParam(id);

  if (int const res = group_sync_read.txRxPacket();
      res != COMM_SUCCESS) {
    throw CommunicationError(_packet_handler.get(), res);
  }

  for(auto id : id_vect)
  {
    uint8_t dxl_error = 0;
    if (group_sync_read.getError(id, &dxl_error))
      if (dxl_error)
        throw StatusError(_packet_handler.get(), dxl_error);
  }

  for(auto id : id_vect)
  {
    if (group_sync_read.isAvailable(id, start_address, data_length))
      data_vect.push_back(std::make_tuple(id, group_sync_read.getData(id, start_address, data_length)));
    else
      data_vect.push_back(std::make_tuple(id, std::nullopt));
  }

  group_sync_read.clearParam();

  return data_vect;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */
