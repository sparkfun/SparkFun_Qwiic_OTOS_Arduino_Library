/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 7 - Get Version

    This example demonstrates how to get the hardware and firmware version
    numbers from the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    There may be future hardware and/or firmware changes of the OTOS, so you can
    use this example to check which version you have. See the product page or
    hardware repository to see what the latest version is:
    https://www.sparkfun.com/products/24904
    https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an OTOS object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 7 - Get Version");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

    // Get the hardware and firmware version
    sfe_otos_version_t hwVersion;
    sfe_otos_version_t fwVersion;
    myOtos.getVersionInfo(hwVersion, fwVersion);

    // Print the hardware and firmware version
    Serial.print("OTOS Hardware Version: v");
    Serial.print(hwVersion.major);
    Serial.print(".");
    Serial.println(hwVersion.minor);
    Serial.print("OTOS Firmware Version: v");
    Serial.print(fwVersion.major);
    Serial.print(".");
    Serial.println(fwVersion.minor);
}

void loop()
{
    // Nothing to do in this example
}
