#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 3 - Calibration");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");
    
    // The IMU on the OTOS includes a gyroscope and accelerometer, which could
    // have an offset. Note that as of firmware version 1.0, the calibration
    // will be lost after a power cycle; the OTOS performs a quick calibration
    // when it powers up, but it is recommended to perform a more thorough
    // calibration at the start of all your programs. Note that the sensor must
    // be completely stationary and flat during calibration! When calling
    // calibrateImu(), you can specify the number of samples to take and whether
    // to wait until the calibration is complete. If no parameters are provided,
    // it will take 255 samples and wait until done; each sample takes about
    // 2.4ms, so about 612ms total
    myOtos.calibrateImu();
    // myOtos.calibrateImu(255, true); // Or specify samples and whether to wait

    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    myOtos.setLinearScalar(1.0);
    myOtos.setAngularScalar(1.0);

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
}
