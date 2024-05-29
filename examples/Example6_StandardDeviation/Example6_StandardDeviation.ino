/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

/*******************************************************************************
    Example 6 - Standard Deviation

    This example demonstrates how to read the standard deviation of the
    measurements of the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).

    The OTOS uses Kalman filters to estimate the position, velocity, and
    acceleration of the x, y, and heading. The square root of the diagonal
    elements of the covariance matrices are provided for the standard deviation
    of each measurement. THEY DO NOT REPRESENT THE ACTUAL TRACKING ERROR! These
    are statistical quantities that assume a correct model of the system, but
    there could be unmodelled error sources that cause the physical error to
    become larger than these statistical error (eg. improper calibration, or
    tilting the OTOS to not be flat against the tracking surface). These are
    provided primarily for anyone wanting to perform sensor fusion with
    additional sensors.
*******************************************************************************/

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 6 - Standard Deviation");

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
    // Create structs for position and standard deviations. This example can be
    // extended to include velocity and acceleration, which have been omitted
    // for simplicity, but can be added by uncommenting the code below
    sfe_otos_pose2d_t pos;
    // sfe_otos_pose2d_t vel;
    // sfe_otos_pose2d_t acc;
    sfe_otos_pose2d_t posStdDev;
    // sfe_otos_pose2d_t velStdDev;
    // sfe_otos_pose2d_t accStdDev;
    
    // Read the position like normal (and velocity and acceleration if desired)
    myOtos.getPosition(pos);
    // myOtos.getVelocity(vel);
    // myOtos.getAcceleration(acc);

    // Read the standard deviation of the tracking. Note that these values are
    // just the square root of the diagonal elements of the covariance matrices
    // of the Kalman filters used in the firmware of the OTOS, and THEY DO NOT
    // REPRESENT THE ACTUAL TRACKING ERROR! These are statistical quantities
    // that assume a correct model of the system, but there could be unmodelled
    // error sources that cause the physical error to become larger than these
    // statistical error (eg. improper calibration, or tilting the OTOS to not
    // be flat against the tracking surface). These are provided primarily for
    // anyone wanting to perform sensor fusion with additional sensors.
    myOtos.getPositionStdDev(posStdDev);
    // myOtos.getVelocityStdDev(velStdDev);
    // myOtos.getAccelerationStdDev(accStdDev);

    // These values can instead be burst read out in chunks:
    // myOtos.getPosVelAcc(pos, vel, acc);
    // myOtos.getPosVelAccStdDev(posStdDev, velStdDev, accStdDev);
    
    // Or burst read them all at once:
    // myOtos.getPosVelAccAndStdDev(pos, vel, acc, posStdDev, velStdDev, accStdDev);

    // Print position and standard deviation
    Serial.println();
    Serial.println("Position:");
    Serial.print("X (Inches): ");
    Serial.print(pos.x);
    Serial.print(" +/- ");
    Serial.println(posStdDev.x);
    Serial.print("Y (Inches): ");
    Serial.print(pos.y);
    Serial.print(" +/- ");
    Serial.println(posStdDev.y);
    Serial.print("Heading (Degrees): ");
    Serial.print(pos.h);
    Serial.print(" +/- ");
    Serial.println(posStdDev.h);

    // Print velocity and standard deviation
    // Serial.println();
    // Serial.println("Velocity:");
    // Serial.print("X (Inches/sec): ");
    // Serial.print(vel.x);
    // Serial.print(" +/- ");
    // Serial.println(velStdDev.x);
    // Serial.print("Y (Inches/sec): ");
    // Serial.print(vel.y);
    // Serial.print(" +/- ");
    // Serial.println(velStdDev.y);
    // Serial.print("Heading (Degrees/sec): ");
    // Serial.print(vel.h);
    // Serial.print(" +/- ");
    // Serial.println(velStdDev.h);

    // Print acceleration and standard deviation
    // Serial.println();
    // Serial.println("Acceleration:");
    // Serial.print("X (Inches/sec^2): ");
    // Serial.print(acc.x);
    // Serial.print(" +/- ");
    // Serial.println(accStdDev.x);
    // Serial.print("Y (Inches/sec^2): ");
    // Serial.print(acc.y);
    // Serial.print(" +/- ");
    // Serial.println(accStdDev.y);
    // Serial.print("Heading (Degrees/sec^2): ");
    // Serial.print(acc.h);
    // Serial.print(" +/- ");
    // Serial.println(accStdDev.h);

    // Wait a bit so we don't spam the serial port
    delay(500);
}
