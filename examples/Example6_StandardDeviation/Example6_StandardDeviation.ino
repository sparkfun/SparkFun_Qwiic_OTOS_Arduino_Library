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
    // Create structs for position and standard deviations. This example can be
    // extended to include velocity and acceleration, which have been omitted
    // for simplicity, but can be added by uncommenting the code below
    otos_pose2d_t pos;
    // otos_pose2d_t vel;
    // otos_pose2d_t acc;
    otos_pose2d_t posStdDev;
    // otos_pose2d_t velStdDev;
    // otos_pose2d_t accStdDev;
    
    // Read the position like normal (and velocity and acceleration if desired)
    myOtos.getPosition(pos);
    // myOtos.getVelocity(vel);
    // myOtos.getAccerlation(acc);

    // Read the standard deviation of the tracking. Note that these values are
    // just the square root of the diagonal elements of the covariance matrices
    // of the Kalman filters used in the firmware of the OTOS, and THEY DO NOT
    // REPRESENT THE ACTUAL TRACKING ERROR! These values are provided primarily
    // for anyone wanting to do sensor fusion with additional sensors, but they
    // can be used to at least get an idea of the quality of the tracking.
    myOtos.getPositionStdDev(posStdDev);
    // myOtos.getVelocityStdDev(velStdDev);
    // myOtos.getAccerlationStdDev(accStdDev);

    // These values can instead be read out in chunks:
    // myOtos.getPosVelAcc(pos, vel, acc);
    // myOtos.getPosVelAccStdDev(posStdDev, velStdDev, accStdDev);
    
    // Or all at once:
    // myOtos.getPosVelAccAndStdDev(pos, vel, acc, posStdDev, velStdDev, accStdDev);

    // Print measurements
    Serial.println();
    Serial.println("Sensor pose:");
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

    // Serial.println();
    // Serial.println("Sensor velocity:");
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

    // Serial.println();
    // Serial.println("Sensor acceleration:");
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
