/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 2 - Set Units

    This example demonstrates how to change the units of the SparkFun Qwiic
    Optical Tracking Odometry Sensor (OTOS).

    The OTOS library defaults to inches and degrees, but you can change the
    units to suit the needs of your project.
*******************************************************************************/

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

    Serial.println("Ensure the OTOS is flat and stationary, then enter any key to calibrate the IMU");

    // Clear the serial buffer
    while (Serial.available())
        Serial.read();
    // Wait for user input
    while (!Serial.available())
        ;

    Serial.println("Calibrating IMU...");

    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu();

    // Set the desired units for linear and angular measurements. Can be either
    // meters or inches for linear, and radians or degrees for angular. If not
    // set, the default is inches and degrees. Note that this setting is not
    // stored in the sensor, it's part of the library, so you need to set at the
    // start of all your programs.
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    // myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    // myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);

    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();
}

void loop()
{
    // Get the latest position, which includes the x and y coordinates, plus the
    // heading angle
    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    // Print measurement
    Serial.println();
    Serial.println("Position:");
    Serial.print("X (Meters): ");
    Serial.println(myPosition.x, 4);
    Serial.print("Y (Meters): ");
    Serial.println(myPosition.y, 4);
    Serial.print("Heading (Radians): ");
    Serial.println(myPosition.h, 4);

    // Wait a bit so we don't spam the serial port
    delay(500);
}
