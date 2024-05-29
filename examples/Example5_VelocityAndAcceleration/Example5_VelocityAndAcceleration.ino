/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 5 - Velocity and Acceleration

    This example demonstrates how to read the velocity and acceleration from the
    SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    The primary purpose of the OTOS is to track position, but it also provides
    velocity and acceleration measurements for more advanced applications. Note
    that these measurements can be noisy and inaccurate, especially if the
    sensor is not flat to the ground.
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 5 - Velocity and Acceleration");

    Wire.begin();

    Wire.setClock(100000);

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
    // Create structs for position, velocity, and acceleration
    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
    sfe_otos_pose2d_t acc;
    
    // These values can be read individually like so:
    myOtos.getPosition(pos);
    myOtos.getVelocity(vel);
    myOtos.getAcceleration(acc);

    // Or burst read them all at once with the following:
    // myOtos.getPosVelAcc(pos, vel, acc);

    // Print position
    Serial.println();
    Serial.println("Position:");
    Serial.print("X (Inches): ");
    Serial.println(pos.x);
    Serial.print("Y (Inches): ");
    Serial.println(pos.y);
    Serial.print("Heading (Degrees): ");
    Serial.println(pos.h);
    
    // Print velocity
    Serial.println();
    Serial.println("Velocity:");
    Serial.print("X (Inches/sec): ");
    Serial.println(vel.x);
    Serial.print("Y (Inches/sec): ");
    Serial.println(vel.y);
    Serial.print("Heading (Degrees/sec): ");
    Serial.println(vel.h);
    
    // Print acceleration
    Serial.println();
    Serial.println("Acceleration:");
    Serial.print("X (Inches/sec^2): ");
    Serial.println(acc.x);
    Serial.print("Y (Inches/sec^2): ");
    Serial.println(acc.y);
    Serial.print("Heading (Degrees/sec^2): ");
    Serial.println(acc.h);

    // Wait a bit so we don't spam the serial port
    delay(500);
}
