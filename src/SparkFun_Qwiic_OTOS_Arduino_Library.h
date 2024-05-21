/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    SparkFun_Qwiic_OTOS_Arduino_Library.h - Arduino wrapper of the C++ driver
    for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    This is a simple wrapper around the C++ driver to make it easier to use with
    Arduino.
*******************************************************************************/

#pragma once

#include "Arduino.h"
#include "sfeQwiicOtos.h"
#include <Wire.h>

class QwiicOTOS : public sfeQwiicOtos
{
  public:
    /// @brief Begins the Qwiic Ultrasonic sensor
    /// @param address I2C device address to use for the sensor
    /// @param wirePort Wire port to use for I2C communication
    /// @return True if successful, false otherwise
    bool begin(TwoWire &wirePort = Wire)
    {
        // Setup Arduino I2C bus
        _theI2CBus.init(wirePort, kDefaultAddress);

        // Begin the sensor
        return sfeQwiicOtos::begin(&_theI2CBus) == kSTkErrOk;
    }

  protected:
    void delayMs(uint32_t ms)
    {
        delay(ms);
    }

  private:
    sfeTkArdI2C _theI2CBus;
};
