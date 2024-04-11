#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// Create an OTOS object
QwiicOTOS myOtos;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Qwiic OTOS Example 8 - Self Test");

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

    // Perform the self test
    sfeTkError_t result = myOtos.selfTest();
    
    // Check if the self test passed
    if(result == kSTkErrOk)
    {
        Serial.println("Self test passed!");
    }
    else
    {
        Serial.println("Self test failed!");
    }
}

void loop()
{
    // Nothing to do in this example
}
