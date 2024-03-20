#pragma once

#include "SparkFun_Toolkit.h"
#include <math.h>
#include <stdint.h>

// Default I2C addresses of the Qwiic OTOS
const uint8_t kOtosDefaultAddress = 0x17;

// OTOS register map
const uint8_t kOtosRegProductId = 0x00;
const uint8_t kOtosRegGyroCalib = 0x0D;
const uint8_t kOtosRegSelfTest = 0x0E;
const uint8_t kOtosRegStatus = 0x0F;
const uint8_t kOtosRegPosXL = 0x10;
const uint8_t kOtosRegPosXH = 0x11;
const uint8_t kOtosRegPosYL = 0x12;
const uint8_t kOtosRegPosYH = 0x13;
const uint8_t kOtosRegPosHL = 0x14;
const uint8_t kOtosRegPosHH = 0x15;
const uint8_t kOtosRegVelXL = 0x16;
const uint8_t kOtosRegVelXH = 0x17;
const uint8_t kOtosRegVelYL = 0x18;
const uint8_t kOtosRegVelYH = 0x19;
const uint8_t kOtosRegVelHL = 0x1A;
const uint8_t kOtosRegVelHH = 0x1B;
const uint8_t kOtosRegAccXL = 0x1C;
const uint8_t kOtosRegAccXH = 0x1D;
const uint8_t kOtosRegAccYL = 0x1E;
const uint8_t kOtosRegAccYH = 0x1F;
const uint8_t kOtosRegAccHL = 0x20;
const uint8_t kOtosRegAccHH = 0x21;

// Product ID register value
const uint8_t kOtosProductId = 0x5F;

// Conversion factors
const float kMeterToInch = 39.37f;
const float kInchToMeter = 1.0f / kMeterToInch;
const float kRadianToDegree = 180.0f / M_PI;
const float kDegreeToRadian = M_PI / 180.0f;

// Conversion factor for X and Y position/velocity/acceleration registers.
// 16-bit signed registers with a max value of 16 gives a resolution of about
// 0.00049 meters (0.019 inches)
const float kMeterToInt16 = 32768.0f / 16.0f;
const float kInt16ToMeter = 1.0f / kMeterToInt16;

// Converstion factor for the heading register. 16-bit signed register with a
// max value of pi gives a resolution of about 0.00096 radians (0.0055 degrees)
const float kRadToInt16 = 32768.0f / M_PI;
const float kInt16ToRad = 1.0f / kRadToInt16;

// Converstion factor for the heading velocity register. 16-bit signed register
// with a max value of 34.9 rps (2000 dps) gives a resolution of about 0.0011
// rps (0.061 degrees per second)
const float kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
const float kInt16ToRps = 1.0f / kRpsToInt16;

// Converstion factor for the heading acceleration register. 16-bit signed
// register with a max value of 3141 rps^2 (180000 dps^2) gives a resolution of
// about 0.096 rps^2 (5.5 dps^2)
const float kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
const float kInt16ToRpss = 1.0f / kRpssToInt16;

// Struct to define a 2D pose, including x and y coordinates and an angle h
struct otos_pose2d_t
{
    float x;
    float y;
    float h;
};

enum otos_linear_unit_t
{
    kOtosLinearUnitMeters = 0,
    kOtosLinearUnitInches = 1
};

enum otos_angular_unit_t
{
    kOtosAngularUnitRadians = 0,
    kOtosAngularUnitDegrees = 1
};

class sfeQwiicOtos
{
  public:
    /// @brief Default constructor
    sfeQwiicOtos();

    /// @brief Begins the Qwiic OTOS
    /// @param commBus I2C bus to use for communication
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfeTkError_t begin(sfeTkII2C *commBus = nullptr);

    /// @brief Checks if the device is connected
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfeTkError_t isConnected();

    sfeTkError_t getHardwareVersion(uint8_t &hwVersion);

    sfeTkError_t getFirmwareVersion(uint8_t &fwVersion);

    sfeTkError_t calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);

    otos_linear_unit_t getLinearUnit();

    void setLinearUnit(otos_linear_unit_t unit);

    otos_angular_unit_t getAngularUnit();

    void setAngularUnit(otos_angular_unit_t unit);

    sfeTkError_t getOffset(otos_pose2d_t &pose);

    sfeTkError_t setOffset(otos_pose2d_t &pose);

    sfeTkError_t getPosition(otos_pose2d_t &pose);

    sfeTkError_t setPosition(otos_pose2d_t &pose);

    sfeTkError_t getVelocity(otos_pose2d_t &pose);

    sfeTkError_t setVelocity(otos_pose2d_t &pose);

    sfeTkError_t getAccerlation(otos_pose2d_t &pose);

    sfeTkError_t setAccerlation(otos_pose2d_t &pose);

    otos_pose2d_t transformPose(otos_pose2d_t &pose12, otos_pose2d_t &pose23);

    otos_pose2d_t invertPose(otos_pose2d_t &pose);

    float wrapAngle(float angle, otos_angular_unit_t unit = kOtosAngularUnitRadians);

  protected:
    // Virtual function that must be implemented by the derived class to delay
    // for a given number of milliseconds
    virtual void delayMs(uint32_t ms) = 0;

    // Function to read raw pose registers and convert to specified units
    sfeTkError_t readPoseRegs(uint8_t reg, otos_pose2d_t &pose, float rawToXY, float rawToH);

    // Function to write raw pose registers and convert from specified units
    sfeTkError_t writePoseRegs(uint8_t reg, otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    // I2C bus to use for communication
    sfeTkII2C *_commBus;

    // Units to be used by the public pose functions. Everything uses meters and
    // radians internally, so this just determines what conversion factor is
    // applied to the public functions
    otos_linear_unit_t _linearUnit;
    otos_angular_unit_t _angularUnit;

    // Offset pose of the OTOS from the host, and its inverse. Stored internally
    // in meters and radians
    otos_pose2d_t _offsetPose;
    otos_pose2d_t _offsetPoseInv;
};
