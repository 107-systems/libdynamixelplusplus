/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

#include <tuple>
#include <vector>
#include <memory>
#include <optional>

#include <dynamixel_sdk.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixelplusplus
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Dynamixel
{
public:

  enum class ProtocolVersion : int
  {
    V1_0 = 1,
    V2_0 = 2,
  };

   Dynamixel(std::string const & device_name,
             ProtocolVersion const protocol_version,
             int const baudrate);
  ~Dynamixel();


  enum class Error : int
  {
    None          =  0,
    AddParam      = -1,
    TxPacket      = -2,
    TxRxPacket    = -3,
    BroadcastPing = -4,
  };

  typedef uint8_t                                 Id;
  typedef std::vector<Id>                         IdVect;
  typedef std::tuple<Id, uint8_t *>               SyncWriteData;
  typedef std::vector<SyncWriteData>              SyncWriteDataVect;
  typedef std::tuple<Id, std::optional<uint32_t>> SyncReadData;
  typedef std::vector<SyncReadData>               SyncReadDataVect;


  std::tuple<Error, IdVect> broadcastPing();

  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteData const & data);
  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data);

  std::tuple<Error, SyncReadData>     syncRead(uint16_t const start_address, uint16_t const data_length, uint8_t const id);
  std::tuple<Error, SyncReadDataVect> syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect);


private:

  std::unique_ptr<dynamixel::PortHandler> _port_handler;
  std::unique_ptr<dynamixel::PacketHandler> _packet_handler;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<Dynamixel> SharedDynamixel;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

#endif /* DYNAMIXEL_H_ */
