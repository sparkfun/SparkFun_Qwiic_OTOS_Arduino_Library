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

    // Or all at once with the following:
    // myOtos.getPosVelAcc(pos, vel, acc);

    // Print measurements
    Serial.printf("Pos: X: %.3f\tY: %.3f\tH: %.3f\n", pos.x, pos.y, pos.h);
    Serial.printf("Vel: X: %.3f\tY: %.3f\tH: %.3f\n", vel.x, vel.y, vel.h);
    Serial.printf("Acc: X: %.3f\tY: %.3f\tH: %.3f\n", acc.x, acc.y, acc.h);
    Serial.println();

    // Wait a bit so we don't spam the serial port
    delay(500);
}
