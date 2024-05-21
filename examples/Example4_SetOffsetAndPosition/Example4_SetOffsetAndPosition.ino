/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 4 - Set Offset and Position

    This example demonstrates how to set the offset and position of the SparkFun
    Qwiic Optical Tracking Odometry Sensor (OTOS).

    If your OTOS is mounted to a robot and is not centered, you can specify the
    offset for the sensor relative to the center of the robot; rather than
    returning the position of the sensor, the OTOS will calculate and return the
    position of the robot's center. If you know where your robot is located,
    such as the starting location or from another sensor, you can send that
    position to the OTOS and it will continue to track from there.
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 4 - Set Offset and Position");

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

    // Assuming you've mounted your sensor to a robot and it's not centered,
    // you can specify the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! Note that as of
    // firmware version 1.0, these values will be lost after a power cycle, so
    // you will need to set them each time you power up the sensor. For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
    sfe_otos_pose2d_t offset = {-5, 10, -90};
    myOtos.setOffset(offset);

    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();

    // After resetting the tracking, the OTOS will report that the robot is at
    // the origin. If your robot does not start at the origin, or you have
    // another source of location information (eg. vision odometry), you can set
    // the OTOS location to match and it will continue to track from there.
    sfe_otos_pose2d_t currentPosition = {0, 0, 0};
    myOtos.setPosition(currentPosition);
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

}
