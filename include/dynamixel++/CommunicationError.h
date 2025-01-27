/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

#ifndef DYNAMIXEL_COMMUNICATIONERROR_H
#define DYNAMIXEL_COMMUNICATIONERROR_H

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

class CommunicationError : public std::runtime_error
{
public:
  CommunicationError(
    dynamixel::PacketHandler * packet_handler,
    int const dxl_err_code)
  : std::runtime_error{packet_handler->getTxRxResult(dxl_err_code)}
  , error_code(dxl_err_code)
  { }

  int const error_code;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

#endif /* DYNAMIXEL_COMMUNICATIONERROR_H */
