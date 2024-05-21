/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

#pragma once

#include "SparkFun_Toolkit.h"
#include <math.h>
#include <stdint.h>

// Default I2C addresses of the Qwiic OTOS
const uint8_t kOtosDefaultAddress = 0x17;

// OTOS register map
const uint8_t kOtosRegProductId = 0x00;
const uint8_t kOtosRegHwVersion = 0x01;
const uint8_t kOtosRegFwVersion = 0x02;
const uint8_t kOtosRegScalarLinear = 0x04;
const uint8_t kOtosRegScalarAngular = 0x05;
const uint8_t kOtosRegImuCalib = 0x06;
const uint8_t kOtosRegReset = 0x07;

const uint8_t kOtosRegSignalProcess = 0x0E;
const uint8_t kOtosRegSelfTest = 0x0F;

const uint8_t kOtosRegOffXL = 0x10;
const uint8_t kOtosRegOffXH = 0x11;
const uint8_t kOtosRegOffYL = 0x12;
const uint8_t kOtosRegOffYH = 0x13;
const uint8_t kOtosRegOffHL = 0x14;
const uint8_t kOtosRegOffHH = 0x15;

const uint8_t kOtosRegStatus = 0x1F;

const uint8_t kOtosRegPosXL = 0x20;
const uint8_t kOtosRegPosXH = 0x21;
const uint8_t kOtosRegPosYL = 0x22;
const uint8_t kOtosRegPosYH = 0x23;
const uint8_t kOtosRegPosHL = 0x24;
const uint8_t kOtosRegPosHH = 0x25;
const uint8_t kOtosRegVelXL = 0x26;
const uint8_t kOtosRegVelXH = 0x27;
const uint8_t kOtosRegVelYL = 0x28;
const uint8_t kOtosRegVelYH = 0x29;
const uint8_t kOtosRegVelHL = 0x2A;
const uint8_t kOtosRegVelHH = 0x2B;
const uint8_t kOtosRegAccXL = 0x2C;
const uint8_t kOtosRegAccXH = 0x2D;
const uint8_t kOtosRegAccYL = 0x2E;
const uint8_t kOtosRegAccYH = 0x2F;
const uint8_t kOtosRegAccHL = 0x30;
const uint8_t kOtosRegAccHH = 0x31;

const uint8_t kOtosRegPosStdXL = 0x32;
const uint8_t kOtosRegPosStdXH = 0x33;
const uint8_t kOtosRegPosStdYL = 0x34;
const uint8_t kOtosRegPosStdYH = 0x35;
const uint8_t kOtosRegPosStdHL = 0x36;
const uint8_t kOtosRegPosStdHH = 0x37;
const uint8_t kOtosRegVelStdXL = 0x38;
const uint8_t kOtosRegVelStdXH = 0x39;
const uint8_t kOtosRegVelStdYL = 0x3A;
const uint8_t kOtosRegVelStdYH = 0x3B;
const uint8_t kOtosRegVelStdHL = 0x3C;
const uint8_t kOtosRegVelStdHH = 0x3D;
const uint8_t kOtosRegAccStdXL = 0x3E;
const uint8_t kOtosRegAccStdXH = 0x3F;
const uint8_t kOtosRegAccStdYL = 0x40;
const uint8_t kOtosRegAccStdYH = 0x41;
const uint8_t kOtosRegAccStdHL = 0x42;
const uint8_t kOtosRegAccStdHH = 0x43;

// Product ID register value
const uint8_t kOtosProductId = 0x5F;

// Conversion factors
const float kMeterToInch = 39.37f;
const float kInchToMeter = 1.0f / kMeterToInch;
const float kRadianToDegree = 180.0f / M_PI;
const float kDegreeToRadian = M_PI / 180.0f;

// Conversion factor for the linear position registers. 16-bit signed registers
// with a max value of 10 meters (394 inches) gives a resolution of about 0.0003
// mps (0.012 ips)
const float kMeterToInt16 = 32768.0f / 10.0f;
const float kInt16ToMeter = 1.0f / kMeterToInt16;

// Conversion factor for the linear velocity registers. 16-bit signed registers
// with a max value of 5 mps (197 ips) gives a resolution of about 0.00015 mps
// (0.006 ips)
const float kMpsToInt16 = 32768.0f / 5.0f;
const float kInt16ToMps = 1.0f / kMpsToInt16;

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
const float kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
const float kInt16ToMpss = 1.0f / kMpssToInt16;

// Conversion factor for the angular position registers. 16-bit signed registers
// with a max value of pi radians (180 degrees) gives a resolution of about
// 0.00096 radians (0.0055 degrees)
const float kRadToInt16 = 32768.0f / M_PI;
const float kInt16ToRad = 1.0f / kRadToInt16;

// Conversion factor for the angular velocity registers. 16-bit signed registers
// with a max value of 34.9 rps (2000 dps) gives a resolution of about 0.0011
// rps (0.061 degrees per second)
const float kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
const float kInt16ToRps = 1.0f / kRpsToInt16;

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a resolution of
// about 0.096 rps^2 (5.5 dps^2)
const float kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
const float kInt16ToRpss = 1.0f / kRpssToInt16;

// Min and max scalar values for the linear and angular scalars (8-bit signed
// integer representing 0.1% increments)
const float kSfeOtosMinScalar = 0.872f;
const float kSfeOtosMaxScalar = 1.127f;

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

// Version register bit fields
typedef union {
    struct
    {
        uint8_t minor : 4;
        uint8_t major : 4;
    };
    uint8_t value;
} otos_version_t;

// Signal process register bit fields
typedef union {
    struct
    {
        uint8_t enLut : 1;
        uint8_t enAcc : 1;
        uint8_t enRot : 1;
        uint8_t enVar : 1;
        uint8_t reserved : 4;
    };
    uint8_t value;
} sfe_otos_config_signal_process_t;

// Self test register bit fields
typedef union {
    struct
    {
        uint8_t start : 1;
        uint8_t inProgress : 1;
        uint8_t pass : 1;
        uint8_t fail : 1;
        uint8_t reserved : 4;
    };
    uint8_t value;
} sfe_otos_config_self_test_t;

// Status register bit fields
typedef union {
    struct
    {
        uint8_t warnTiltAngle : 1;
        uint8_t warnOpticalTracking : 1;
        uint8_t reserved : 4;
        uint8_t errorPaa : 1;
        uint8_t errorLsm : 1;
    };
    uint8_t value;
} sfe_otos_status_t;

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

    sfeTkError_t getVersionInfo(otos_version_t &hwVersion, otos_version_t &fwVersion);

    sfeTkError_t selfTest();

    sfeTkError_t calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);

    otos_linear_unit_t getLinearUnit();

    void setLinearUnit(otos_linear_unit_t unit);

    otos_angular_unit_t getAngularUnit();

    void setAngularUnit(otos_angular_unit_t unit);

    sfeTkError_t getLinearScalar(float &scalar);

    sfeTkError_t setLinearScalar(float scalar);

    sfeTkError_t getAngularScalar(float &scalar);

    sfeTkError_t setAngularScalar(float scalar);

    sfeTkError_t resetTracking();

    sfeTkError_t getSignalProcess(sfe_otos_config_signal_process_t &config);

    sfeTkError_t setSignalProcess(sfe_otos_config_signal_process_t &config);

    sfeTkError_t getStatus(sfe_otos_status_t &status);

    sfeTkError_t getOffset(otos_pose2d_t &pose);

    sfeTkError_t setOffset(otos_pose2d_t &pose);

    sfeTkError_t getPosition(otos_pose2d_t &pose);

    sfeTkError_t setPosition(otos_pose2d_t &pose);

    sfeTkError_t getVelocity(otos_pose2d_t &pose);

    sfeTkError_t getAcceleration(otos_pose2d_t &pose);

    sfeTkError_t getPositionStdDev(otos_pose2d_t &pose);

    sfeTkError_t getVelocityStdDev(otos_pose2d_t &pose);

    sfeTkError_t getAccelerationStdDev(otos_pose2d_t &pose);

    sfeTkError_t getPosVelAcc(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc);

    sfeTkError_t getPosVelAccStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc);

    sfeTkError_t getPosVelAccAndStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc, otos_pose2d_t &posStdDev, otos_pose2d_t &velStdDev, otos_pose2d_t &accStdDev);

  protected:
    // Virtual function that must be implemented by the derived class to delay
    // for a given number of milliseconds
    virtual void delayMs(uint32_t ms) = 0;

    // Function to read raw pose registers and convert to specified units
    sfeTkError_t readPoseRegs(uint8_t reg, otos_pose2d_t &pose, float rawToXY, float rawToH);

    // Function to write raw pose registers and convert from specified units
    sfeTkError_t writePoseRegs(uint8_t reg, otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    void regsToPose(uint8_t *rawData, otos_pose2d_t &pose, float rawToXY, float rawToH);

    void poseToRegs(uint8_t *rawData, otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    // I2C bus to use for communication
    sfeTkII2C *_commBus;

    // Units to be used by the public pose functions. Everything uses meters and
    // radians internally, so this just determines what conversion factor is
    // applied to the public functions
    otos_linear_unit_t _linearUnit;
    otos_angular_unit_t _angularUnit;

    // Conversion factors from meters and radians to the current linear and
    // angular units
    float _meterToUnit;
    float _radToUnit;
};
