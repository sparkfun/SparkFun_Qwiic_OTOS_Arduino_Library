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
    Serial.println("Qwiic OTOS Example 1 - Basic Readings");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

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
    Serial.print("X (Inches): ");
    Serial.println(otosPose.x);
    Serial.print("Y (Inches): ");
    Serial.println(otosPose.y);
    Serial.print("Heading (Degrees): ");
    Serial.println(otosPose.h);

    // Wait a bit so we don't spam the serial port
    delay(500);

    // Alternatively, you can comment out the print and delay code above, and
    // instead use the following code to rapidly refresh the data
    // Serial.print(otosPose.x);
    // Serial.print("\t");
    // Serial.print(otosPose.y);
    // Serial.print("\t");
    // Serial.println(otosPose.h);
    // delay(10);
}
