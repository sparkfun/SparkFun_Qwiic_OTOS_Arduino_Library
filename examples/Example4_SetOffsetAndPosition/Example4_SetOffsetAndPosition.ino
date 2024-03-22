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
    
    // Here you can set the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
    otos_pose2d_t offset = {-5, 10, -90};
    myOtos.setOffset(offset);

    // Reset the tracking algorithm, making the sensor report it's at the origin
    myOtos.resetTracking();

    // The OTOS sensor itself does not know about the offset defined above, it
    // only tracks its own location; the offset is applied by the library when
    // getPosition() is called. After resetting the tracking, the sensor will
    // report that it's at the origin, but getPosition() will return the inverse
    // of the offset pose. This can be resolved by calling setPosition(), which
    // take into account the offset and set the sensor's location accordingly.
    // Here we set the location to the origin, but any location is valid within
    // the sensor's tracking range.
    otos_pose2d_t newPose = {0, 0, 0};
    myOtos.setPosition(newPose);
}

void loop()
{
    // Get the latest robot pose, which includes the x and y coordinates, plus
    // the heading angle
    otos_pose2d_t robotPose;
    myOtos.getPosition(robotPose);

    // Print measurement
    Serial.println();
    Serial.println("Robot pose:");
    Serial.print("X (Inches): ");
    Serial.println(robotPose.x);
    Serial.print("Y (Inches): ");
    Serial.println(robotPose.y);
    Serial.print("Heading (Degrees): ");
    Serial.println(robotPose.h);

    // Wait a bit so we don't spam the serial port
    delay(500);
}
