/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

#ifndef DYNAMIXEL_STATUSERROR_H
#define DYNAMIXEL_STATUSERROR_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdexcept>

#include <dynamixel_sdk.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixelplusplus
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StatusError : public std::runtime_error
{
public:
  StatusError(dynamixel::PacketHandler * packet_handler, int const dxl_err_code)
    : std::runtime_error{packet_handler->getRxPacketError(dxl_err_code)}
  { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

#endif /* DYNAMIXEL_STATUSERROR_H */
