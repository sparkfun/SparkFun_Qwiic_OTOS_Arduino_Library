#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an OTOS object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 7 - Get Version");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

    // Get the hardware and firmware version
    otos_version_t hwVersion;
    otos_version_t fwVersion;
    myOtos.getVersionInfo(hwVersion, fwVersion);

    // Print the hardware and firmware version
    Serial.print("OTOS Hardware Version: v");
    Serial.print(hwVersion.major);
    Serial.print(".");
    Serial.println(hwVersion.minor);
    Serial.print("OTOS Firmware Version: v");
    Serial.print(fwVersion.major);
    Serial.print(".");
    Serial.println(fwVersion.minor);
}

void loop()
{
    // Nothing to do in this example
}
