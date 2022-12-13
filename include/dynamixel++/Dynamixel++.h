/**
 * This software is distributed under the terms of the MIT License.
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

#include "StatusError.h"
#include "HardwareAlert.h"
#include "CommunicationError.h"

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define _107_LIBDYNAMIXELPLUSPLUS_BASE_MAJOR 0001
#define _107_LIBDYNAMIXELPLUSPLUS_BASE_MINOR 0000
#define _107_LIBDYNAMIXELPLUSPLUS_BASE_PATCH 0000

#define _107_LIBDYNAMIXELPLUSPLUS_BASE_CONCAT_VERSION_(a,b,c) a ## b ## c
#define _107_LIBDYNAMIXELPLUSPLUS_BASE_CONCAT_VERSION(a,b,c) _107_LIBDYNAMIXELPLUSPLUS_BASE_CONCAT_VERSION_(a,b,c)

#define _107_LIBDYNAMIXELPLUSPLUS_BASE_VERSION \
        _107_LIBDYNAMIXELPLUSPLUS_BASE_CONCAT_VERSION(_107_LIBDYNAMIXELPLUSPLUS_BASE_MAJOR, \
                                                      _107_LIBDYNAMIXELPLUSPLUS_BASE_MINOR, \
                                                      _107_LIBDYNAMIXELPLUSPLUS_BASE_PATCH)

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

  static size_t constexpr MAJOR = _107_LIBDYNAMIXELPLUSPLUS_BASE_MAJOR;
  static size_t constexpr MINOR = _107_LIBDYNAMIXELPLUSPLUS_BASE_MINOR;
  static size_t constexpr PATCH = _107_LIBDYNAMIXELPLUSPLUS_BASE_PATCH;

  enum class Protocol : int
  {
    V1_0 = 1,
    V2_0 = 2,
  };

   Dynamixel(std::string const & device_name,
             Protocol const protocol_version,
             int const baudrate);
  ~Dynamixel();


  typedef uint8_t         Id;
  typedef std::vector<Id> IdVect;


  IdVect broadcastPing();

  void reboot(Id const id);

  template<typename T> T               read    (uint16_t const start_address, Id const id);
  template<typename T> std::map<Id, T> bulkRead(uint16_t const start_address, IdVect const & id_vect);
  template<typename T> std::map<Id, T> syncRead(uint16_t const start_address, IdVect const & id_vect);

  template<typename T> void write    (uint16_t const start_address, Id const id, T const val);
  template<typename T> void syncWrite(uint16_t const start_address, std::map<Id, T> const & val_map);


private:
  std::unique_ptr<dynamixel::PortHandler> _port_handler;
  std::unique_ptr<dynamixel::PacketHandler> _packet_handler;

  typedef std::tuple<Id, uint8_t *>  SyncWriteData;
  typedef std::vector<SyncWriteData> SyncWriteDataVect;
  void syncWrite(uint16_t const start_address, uint16_t const data_length, SyncWriteDataVect const & data);

  typedef std::vector<std::tuple<
                                 Id,
                                 std::optional<uint32_t>
                                >
                     > SyncReadDataVect;
  typedef SyncReadDataVect BulkReadDataVect;
  SyncReadDataVect syncRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect);
  BulkReadDataVect bulkRead(uint16_t const start_address, uint16_t const data_length, IdVect const & id_vect);
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
