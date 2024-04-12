/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 2 - Set Units");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");
    
    // Set the desired units for linear and angular measurements. Can be either
    // meters or inches for linear, and radians or degrees for angular. If not
    // set, the default is inches and degrees. Note that this setting is not
    // stored in the sensor, it's part of the library, so you need to set at the
    // start of all your programs.
    myOtos.setLinearUnit(kOtosLinearUnitMeters);
    // myOtos.setLinearUnit(kOtosLinearUnitInches);
    myOtos.setAngularUnit(kOtosAngularUnitRadians);
    // myOtos.setAngularUnit(kOtosAngularUnitDegrees);

    // Reset the tracking algorithm, making the sensor report it's at the origin
    myOtos.resetTracking();
}

void loop()
{
    // Get the latest sensor pose, which includes the x and y coordinates, plus
    // the heading angle
    otos_pose2d_t otosPose;
    myOtos.getPosition(otosPose);

    // Print measurement
    Serial.println();
    Serial.println("Sensor pose:");
    Serial.print("X (Meters): ");
    Serial.println(otosPose.x, 4);
    Serial.print("Y (Meters): ");
    Serial.println(otosPose.y, 4);
    Serial.print("Heading (Radians): ");
    Serial.println(otosPose.h, 4);

    // Wait a bit so we don't spam the serial port
    delay(500);
}
