/**
 * @file SparkFun_Qwiic_OTOS_Arduino_Library.h
 * @brief Arduino wrapper for the SparkFun Qwiic OTOS sensor driver
 * @details This file provides an Arduino-compatible interface to the SparkFun Qwiic
 *          Optical Tracking Odometry Sensor (OTOS). It wraps the platform-agnostic
 *          C++ driver to provide familiar Arduino conventions and simplified usage.
 *
 * Features:
 * - Simple Arduino-style begin() function
 * - Default Wire interface support
 * - Compatible with all Arduino platforms
 * - Inherits all functionality from base C++ driver
 * - Automatic I2C bus configuration
 *
 * @author SparkFun Electronics
 * @date February 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library
 */

// ...existing code...

/*******************************************************************************
    SparkFun_Qwiic_OTOS_Arduino_Library.h - Arduino wrapper of the C++ driver
    for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    This is a simple wrapper around the C++ driver to make it easier to use with
    Arduino.
*******************************************************************************/

#pragma once
// clang-format off
#include "Arduino.h"
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevOTOS.h"
#include <Wire.h>
// clang-format on

/// @brief Arduino class for the SparkFun Qwiic Optical Tracking Odometry Sensor
/// (OTOS)
class QwiicOTOS : public sfDevOTOS
{
  public:
    /// @brief Begins the Qwiic OTOS and verifies it is connected
    /// @param wirePort Wire port to use for I2C communication
    /// @return True if successful, false otherwise
    bool begin(TwoWire &wirePort = Wire)
    {
        // Setup Arduino I2C bus
        _theI2CBus.init(wirePort, kDefaultAddress);

        // Begin the sensor
        return sfDevOTOS::begin(&_theI2CBus) == ksfTkErrOk;
    }

  protected:
    void delayMs(uint32_t ms)
    {
        delay(ms);
    }

  private:
    sfTkArdI2C _theI2CBus;
};
