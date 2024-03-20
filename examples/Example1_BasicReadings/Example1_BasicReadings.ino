#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an OTOS object
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

    // The IMU on the OTOS includes a gyroscope and accelerometer, which could
    // have an offset. This needs to be calibrated out in order to get accurate
    // tracking, which must be done while the OTOS is stationary and flat!
    Serial.println("Calibrating IMU...");
    myOtos.calibrateImu();

    // We'll also reset the tracking to start at the origin
    Serial.println("Resetting pose...");
    otos_pose2d_t currentPose = {0, 0, 0};
    myOtos.setPosition(currentPose);

    myOtos.setLinearUnit(kOtosLinearUnitInches);
    myOtos.setAngularUnit(kOtosAngularUnitDegrees);
}

void loop()
{
    // Get latest pose from sensor, which includes the x and y coordinates, plus
    // the heading angle
    otos_pose2d_t otosPose;
    myOtos.getPosition(otosPose);

    // Print measurement
    Serial.println();
    Serial.println("Current pose:");
    Serial.print("X (Meters): ");
    Serial.println(otosPose.x, 3);
    Serial.print("Y (Meters): ");
    Serial.println(otosPose.y, 3);
    Serial.print("Heading (Radians): ");
    Serial.println(otosPose.h, 3);

    // Wait a bit so we don't spam the serial port
    delay(500);

    // Alternatively, you can comment out the print and delay code above, and
    // instead use the following code to refresh the data more quickly
    // Serial.print(otosPose.x, 3);
    // Serial.print(", ");
    // Serial.print(otosPose.y, 3);
    // Serial.print(", ");
    // Serial.println(otosPose.h, 3);
    // delay(10);
}
