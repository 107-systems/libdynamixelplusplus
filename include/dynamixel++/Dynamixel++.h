/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
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

  enum class Protocol : int
  {
    V1_0 = 1,
    V2_0 = 2,
  };

   Dynamixel(std::string const & device_name,
             Protocol const protocol_version,
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

  typedef uint8_t         Id;
  typedef std::vector<Id> IdVect;


  std::tuple<Error, IdVect> broadcastPing();

  template<typename T> Error syncRead(uint16_t const start_address, Id const id, T & val);
  template<typename T> Error syncRead(uint16_t const start_address, IdVect const & id_vect, std::map<Id, T> & val_map);

  template<typename T> Error syncWrite(uint16_t const start_address, Id const id, T const val);
  template<typename T> Error syncWrite(uint16_t const start_address, std::vector<std::tuple<Id, T>> const & val_id_vect);


private:
  std::unique_ptr<dynamixel::PortHandler> _port_handler;
  std::unique_ptr<dynamixel::PacketHandler> _packet_handler;

  typedef std::tuple<Id, uint8_t *>  SyncWriteData;
  typedef std::vector<SyncWriteData> SyncWriteDataVect;
  Error syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data);

  typedef std::vector<std::tuple<
                                 Id,
                                 std::optional<uint32_t>
                                >
                     > SyncReadDataVect;
  std::tuple<Error, SyncReadDataVect> syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect);
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<Dynamixel> SharedDynamixel;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

/**************************************************************************************
 * TEMPLATE IMPLEMENTATION
 **************************************************************************************/

#include "Dynamixel++.ipp"

#endif /* DYNAMIXEL_H_ */
