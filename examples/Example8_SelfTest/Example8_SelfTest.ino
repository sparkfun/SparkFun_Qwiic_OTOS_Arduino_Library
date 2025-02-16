/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 8 - Self Test

    This example demonstrates how to perform a self test of the SparkFun Qwiic
    Optical Tracking Odometry Sensor (OTOS).

    The self test triggers the OTOS to perform a series of tests to ensure the
    sensor is functioning correctly. This is performed during QC testing, but
    you can also perform this test yourself.
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an OTOS object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 8 - Self Test");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

    Serial.println("The OTOS must be stationary during the self test!");

    // Perform the self test
    sfTkError_t result = myOtos.selfTest();

    // Check if the self test passed
    if (result == ksfTkErrOk)
    {
        Serial.println("Self test passed!");
    }
    else
    {
        Serial.println("Self test failed!");
    }
}

void loop()
{
    // Nothing to do in this example
}
