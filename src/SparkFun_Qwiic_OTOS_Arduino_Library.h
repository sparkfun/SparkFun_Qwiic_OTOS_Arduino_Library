/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

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
    bool begin(uint8_t address = kOtosDefaultAddress, TwoWire &wirePort = Wire)
    {
        // Setup Arduino I2C bus
        _theI2CBus.init(wirePort, address);

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
