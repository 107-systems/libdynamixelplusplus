/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libdynamixelplusplus/graphs/contributors.
 */

#ifndef DYNAMIXEL_HARDWARE_ALERT_H
#define DYNAMIXEL_HARDWARE_ALERT_H

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

class HardwareAlert : public std::runtime_error
{
public:
  HardwareAlert()
    : std::runtime_error{"HardwareAlert"}
  { }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixelplusplus */

#endif /* DYNAMIXEL_HARDWARE_ALERT_H */
