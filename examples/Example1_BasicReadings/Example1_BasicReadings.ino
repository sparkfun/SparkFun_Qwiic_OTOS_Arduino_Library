/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 1 - Basic Readings

    This example demonstrates how to read the position and heading from the
    SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    This example should be used to verify that the OTOS is connected and
    functioning correctly. It will just print the position and heading tracked
    by the OTOS to the serial monitor. It is recommended that you check out the
    other examples before using the OTOS in your own project.
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 1 - Basic Readings");

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
    Serial.print("X (Inches): ");
    Serial.println(myPosition.x);
    Serial.print("Y (Inches): ");
    Serial.println(myPosition.y);
    Serial.print("Heading (Degrees): ");
    Serial.println(myPosition.h);

    // Wait a bit so we don't spam the serial port
    delay(500);

    // Alternatively, you can comment out the print and delay code above, and
    // instead use the following code to rapidly refresh the data
    // Serial.print(myPosition.x);
    // Serial.print("\t");
    // Serial.print(myPosition.y);
    // Serial.print("\t");
    // Serial.println(myPosition.h);
    // delay(10);
}
