/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

#pragma once

#include "SparkFun_Toolkit.h"
#include <math.h>
#include <stdint.h>

// Struct to define a 2D pose, including x and y coordinates and heading angle
typedef struct
{
    float x;
    float y;
    float h;
} sfe_otos_pose2d_t;

// Enumerations for the linear units
typedef enum
{
    kSfeOtosLinearUnitMeters = 0,
    kSfeOtosLinearUnitInches = 1
} sfe_otos_linear_unit_t;

// Enumerations for the angular units
typedef enum
{
    kSfeOtosAngularUnitRadians = 0,
    kSfeOtosAngularUnitDegrees = 1
} sfe_otos_angular_unit_t;

// Version register bit fields
typedef union {
    struct
    {
        uint8_t minor : 4;
        uint8_t major : 4;
    };
    uint8_t value;
} sfe_otos_version_t;

// Signal process config register bit fields
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
} sfe_otos_signal_process_config_t;

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
} sfe_otos_self_test_config_t;

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

    sfeTkError_t getVersionInfo(sfe_otos_version_t &hwVersion, sfe_otos_version_t &fwVersion);

    sfeTkError_t selfTest();

    sfeTkError_t calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);

    sfeTkError_t getImuCalibrationProgress(uint8_t &numSamples);

    sfe_otos_linear_unit_t getLinearUnit();

    void setLinearUnit(sfe_otos_linear_unit_t unit);

    sfe_otos_angular_unit_t getAngularUnit();

    void setAngularUnit(sfe_otos_angular_unit_t unit);

    sfeTkError_t getLinearScalar(float &scalar);

    sfeTkError_t setLinearScalar(float scalar);

    sfeTkError_t getAngularScalar(float &scalar);

    sfeTkError_t setAngularScalar(float scalar);

    sfeTkError_t resetTracking();

    sfeTkError_t getSignalProcessConfig(sfe_otos_signal_process_config_t &config);

    sfeTkError_t setSignalProcessConfig(sfe_otos_signal_process_config_t &config);

    sfeTkError_t getStatus(sfe_otos_status_t &status);

    sfeTkError_t getOffset(sfe_otos_pose2d_t &pose);

    sfeTkError_t setOffset(sfe_otos_pose2d_t &pose);

    sfeTkError_t getPosition(sfe_otos_pose2d_t &pose);

    sfeTkError_t setPosition(sfe_otos_pose2d_t &pose);

    sfeTkError_t getVelocity(sfe_otos_pose2d_t &pose);

    sfeTkError_t getAcceleration(sfe_otos_pose2d_t &pose);

    sfeTkError_t getPositionStdDev(sfe_otos_pose2d_t &pose);

    sfeTkError_t getVelocityStdDev(sfe_otos_pose2d_t &pose);

    sfeTkError_t getAccelerationStdDev(sfe_otos_pose2d_t &pose);

    sfeTkError_t getPosVelAcc(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc);

    sfeTkError_t getPosVelAccStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc);

    sfeTkError_t getPosVelAccAndStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc,
                        sfe_otos_pose2d_t &posStdDev, sfe_otos_pose2d_t &velStdDev, sfe_otos_pose2d_t &accStdDev);

    // Default I2C addresses of the Qwiic OTOS
    static constexpr uint8_t kDefaultAddress = 0x17;

    // Min and max scalar values for the linear and angular scalars (8-bit
    // signed integer representing 0.1% increments)
    static constexpr float kMinScalar = 0.872f;
    static constexpr float kMaxScalar = 1.127f;

  protected:
    // Virtual function that must be implemented by the derived class to delay
    // for a given number of milliseconds
    virtual void delayMs(uint32_t ms) = 0;

    // Function to read raw pose registers and convert to specified units
    sfeTkError_t readPoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH);

    // Function to write raw pose registers and convert from specified units
    sfeTkError_t writePoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    void regsToPose(uint8_t *rawData, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH);

    void poseToRegs(uint8_t *rawData, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    // I2C bus to use for communication
    sfeTkII2C *_commBus;

    // Units to be used by the public pose functions. Everything uses meters and
    // radians internally, so this just determines what conversion factor is
    // applied to the public functions
    sfe_otos_linear_unit_t _linearUnit;
    sfe_otos_angular_unit_t _angularUnit;

    // Conversion factors from meters and radians to the current linear and
    // angular units
    float _meterToUnit;
    float _radToUnit;

    // OTOS register map
    static constexpr uint8_t kRegProductId = 0x00;
    static constexpr uint8_t kRegHwVersion = 0x01;
    static constexpr uint8_t kRegFwVersion = 0x02;
    static constexpr uint8_t kRegScalarLinear = 0x04;
    static constexpr uint8_t kRegScalarAngular = 0x05;
    static constexpr uint8_t kRegImuCalib = 0x06;
    static constexpr uint8_t kRegReset = 0x07;
    static constexpr uint8_t kRegSignalProcess = 0x0E;
    static constexpr uint8_t kRegSelfTest = 0x0F;
    static constexpr uint8_t kRegOffXL = 0x10;
    static constexpr uint8_t kRegOffXH = 0x11;
    static constexpr uint8_t kRegOffYL = 0x12;
    static constexpr uint8_t kRegOffYH = 0x13;
    static constexpr uint8_t kRegOffHL = 0x14;
    static constexpr uint8_t kRegOffHH = 0x15;
    static constexpr uint8_t kRegStatus = 0x1F;
    static constexpr uint8_t kRegPosXL = 0x20;
    static constexpr uint8_t kRegPosXH = 0x21;
    static constexpr uint8_t kRegPosYL = 0x22;
    static constexpr uint8_t kRegPosYH = 0x23;
    static constexpr uint8_t kRegPosHL = 0x24;
    static constexpr uint8_t kRegPosHH = 0x25;
    static constexpr uint8_t kRegVelXL = 0x26;
    static constexpr uint8_t kRegVelXH = 0x27;
    static constexpr uint8_t kRegVelYL = 0x28;
    static constexpr uint8_t kRegVelYH = 0x29;
    static constexpr uint8_t kRegVelHL = 0x2A;
    static constexpr uint8_t kRegVelHH = 0x2B;
    static constexpr uint8_t kRegAccXL = 0x2C;
    static constexpr uint8_t kRegAccXH = 0x2D;
    static constexpr uint8_t kRegAccYL = 0x2E;
    static constexpr uint8_t kRegAccYH = 0x2F;
    static constexpr uint8_t kRegAccHL = 0x30;
    static constexpr uint8_t kRegAccHH = 0x31;
    static constexpr uint8_t kRegPosStdXL = 0x32;
    static constexpr uint8_t kRegPosStdXH = 0x33;
    static constexpr uint8_t kRegPosStdYL = 0x34;
    static constexpr uint8_t kRegPosStdYH = 0x35;
    static constexpr uint8_t kRegPosStdHL = 0x36;
    static constexpr uint8_t kRegPosStdHH = 0x37;
    static constexpr uint8_t kRegVelStdXL = 0x38;
    static constexpr uint8_t kRegVelStdXH = 0x39;
    static constexpr uint8_t kRegVelStdYL = 0x3A;
    static constexpr uint8_t kRegVelStdYH = 0x3B;
    static constexpr uint8_t kRegVelStdHL = 0x3C;
    static constexpr uint8_t kRegVelStdHH = 0x3D;
    static constexpr uint8_t kRegAccStdXL = 0x3E;
    static constexpr uint8_t kRegAccStdXH = 0x3F;
    static constexpr uint8_t kRegAccStdYL = 0x40;
    static constexpr uint8_t kRegAccStdYH = 0x41;
    static constexpr uint8_t kRegAccStdHL = 0x42;
    static constexpr uint8_t kRegAccStdHH = 0x43;

    // Product ID register value
    static constexpr uint8_t kProductId = 0x5F;

    // Conversion factors
    static constexpr float kMeterToInch = 39.37f;
    static constexpr float kInchToMeter = 1.0f / kMeterToInch;
    static constexpr float kRadianToDegree = 180.0f / M_PI;
    static constexpr float kDegreeToRadian = M_PI / 180.0f;

    // Conversion factor for the linear position registers. 16-bit signed
    // registers with a max value of 10 meters (394 inches) gives a resolution
    // of about 0.0003 mps (0.012 ips)
    static constexpr float kMeterToInt16 = 32768.0f / 10.0f;
    static constexpr float kInt16ToMeter = 1.0f / kMeterToInt16;

    // Conversion factor for the linear velocity registers. 16-bit signed
    // registers with a max value of 5 mps (197 ips) gives a resolution of about
    // 0.00015 mps (0.006 ips)
    static constexpr float kMpsToInt16 = 32768.0f / 5.0f;
    static constexpr float kInt16ToMps = 1.0f / kMpsToInt16;

    // Conversion factor for the linear acceleration registers. 16-bit signed
    // registers with a max value of 157 mps^2 (16 g) gives a resolution of
    // about 0.0048 mps^2 (0.49 mg)
    static constexpr float kMpssToInt16 = 32768.0f / (16.0f * 9.80665f);
    static constexpr float kInt16ToMpss = 1.0f / kMpssToInt16;

    // Conversion factor for the angular position registers. 16-bit signed
    // registers with a max value of pi radians (180 degrees) gives a resolution
    // of about 0.00096 radians (0.0055 degrees)
    static constexpr float kRadToInt16 = 32768.0f / M_PI;
    static constexpr float kInt16ToRad = 1.0f / kRadToInt16;

    // Conversion factor for the angular velocity registers. 16-bit signed
    // registers with a max value of 34.9 rps (2000 dps) gives a resolution of
    // about 0.0011 rps (0.061 degrees per second)
    static constexpr float kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
    static constexpr float kInt16ToRps = 1.0f / kRpsToInt16;

    // Conversion factor for the angular acceleration registers. 16-bit signed
    // registers with a max value of 3141 rps^2 (180000 dps^2) gives a
    // resolution of about 0.096 rps^2 (5.5 dps^2)
    static constexpr float kRpssToInt16 = 32768.0f / (M_PI * 1000.0f);
    static constexpr float kInt16ToRpss = 1.0f / kRpssToInt16;
};
