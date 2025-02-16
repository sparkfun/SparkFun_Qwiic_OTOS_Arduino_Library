/**
 * @file sfDevOTOS.h
 * @brief Driver for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 * @details This file implements a C++ driver class for interfacing with the OTOS sensor.
 *          The driver uses the SparkFun Toolkit library for I2C communication,
 *          making it platform agnostic.
 *
 * Features:
 * - 2D pose tracking (position, velocity, acceleration)
 * - Configurable linear and angular units
 * - Built-in calibration and self-test functions
 * - Adjustable signal processing parameters
 * - Support for sensor offset compensation
 * - Standard deviation measurements for tracking accuracy
 *
 * @author SparkFun Electronics
 * @date February 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 *  SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_Toolkit
 */
#pragma once

#include <math.h>
#include <stdint.h>

// include the sparkfun toolkit headers
#include <sfTk/sfToolkit.h>

// Bus interfaces
#include <sfTk/sfTkII2C.h>

/// @struct sfe_otos_pose2d_t
/// @brief 2D pose structure, including x and y coordinates and heading angle
/// @note Although pose is traditionally used for position and orientation, this
/// structure is also used for velocity and acceleration by the OTOS driver
typedef struct
{
    /// @brief X value
    float x;

    /// @brief Y value
    float y;

    /// @brief Heading value
    float h;
} sfe_otos_pose2d_t;

/// @enum sfe_otos_linear_unit_t
/// @brief Enumerations for linear units used by the OTOS driver
typedef enum
{
    /// @brief Meters
    kSfeOtosLinearUnitMeters = 0,

    /// @brief Inches (default)
    kSfeOtosLinearUnitInches = 1
} sfe_otos_linear_unit_t;

/// @enum sfe_otos_angular_unit_t
/// @brief Enumerations for angular units used by the OTOS driver
typedef enum
{
    /// @brief Radians
    kSfeOtosAngularUnitRadians = 0,

    /// @brief Degrees (default)
    kSfeOtosAngularUnitDegrees = 1
} sfe_otos_angular_unit_t;

/// @union sfe_otos_version_t
/// @brief Version register bit fields
typedef union {
    struct
    {
        /// @brief Minor version number
        uint8_t minor : 4;

        /// @brief Major version number
        uint8_t major : 4;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_version_t;

/// @union sfe_otos_signal_process_config_t
/// @brief Signal process config register bit fields
typedef union {
    struct
    {
        /// @brief Whether to use the internal lookup table calibration for the
        /// optical sensor
        uint8_t enLut : 1;

        /// @brief Whether to feed the accelerometer data to the Kalman filters
        uint8_t enAcc : 1;

        /// @brief Whether to rotate the IMU and optical sensor data by the
        /// heading angle
        uint8_t enRot : 1;

        /// @brief Whether to use the correct sensor variance in the Kalman
        /// filters, or use 0 varaince to effectively disable the filters
        uint8_t enVar : 1;

        /// @brief Reserved bits, do not use
        uint8_t reserved : 4;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_signal_process_config_t;

/// @union sfe_otos_self_test_config_t
/// @brief Self test register bit fields
typedef union {
    struct
    {
        /// @brief Write 1 to start the self test
        uint8_t start : 1;

        /// @brief Returns 1 while the self test is in progress
        uint8_t inProgress : 1;

        /// @brief Returns 1 if the self test passed
        uint8_t pass : 1;

        /// @brief Returns 1 if the self test failed
        uint8_t fail : 1;

        /// @brief Reserved bits, do not use
        uint8_t reserved : 4;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_self_test_config_t;

/// @union sfe_otos_status_t
/// @brief Status register bit fields
typedef union {
    struct
    {
        /// @brief Returns 1 if the tilt angle threshold has been exceeded.
        /// While set, the accelerometer data is ignored
        uint8_t warnTiltAngle : 1;

        /// @brief Returns 1 if the optical tracking is unreliable. While set,
        /// only the IMU data is used for tracking unless warnTiltAngle is set
        uint8_t warnOpticalTracking : 1;

        /// @brief Reserved bits, do not use
        uint8_t reserved : 4;

        /// @brief Returns 1 if the optical sensor has a fatal error
        uint8_t errorPaa : 1;

        /// @brief Returns 1 if the IMU has a fatal error
        uint8_t errorLsm : 1;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_status_t;

/// @brief Class for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).
/// Includes methods to communicate with the sensor, such as getting the tracked
/// location, configuring the sensor, etc. This class is a base class that must
/// be derived to implement the delay function and I2C communication bus.
class sfDevOTOS
{
  public:
    /// @brief Default constructor, only initializes member variables
    sfDevOTOS();

    /// @brief Begins the Qwiic OTOS and verifies it is connected
    /// @param commBus I2C bus to use for communication
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t begin(sfTkII2C *commBus = nullptr);

    /// @brief Checks if the device is connected
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t isConnected();

    /// @brief Gets the hardware and firmware version numbers from the OTOS
    /// @param hwVersion Hardware version number
    /// @param fwVersion Firmware version number
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getVersionInfo(sfe_otos_version_t &hwVersion, sfe_otos_version_t &fwVersion);

    /// @brief Performs a self test of the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t selfTest();

    /// @brief Calibrates the IMU on the OTOS, which removes the accelerometer
    /// and gyroscope offsets
    /// @param numSamples Number of samples to take for calibration. Each sample
    /// takes about 2.4ms, so fewer samples can be taken for faster calibration
    /// @param waitUntilDone Whether to wait until the calibration is complete.
    /// Set false to calibrate asynchronously, see getImuCalibrationProgress()
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);

    /// @brief Gets the progress of the IMU calibration. Used for asynchronous
    /// calibration with calibrateImu()
    /// @param numSamples Number of samples remaining for calibration
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getImuCalibrationProgress(uint8_t &numSamples);

    /// @brief Gets the linear unit used by all methods using a pose
    /// @return Linear unit
    sfe_otos_linear_unit_t getLinearUnit();

    /// @brief Sets the linear unit used by all methods using a pose
    /// @param unit Linear unit
    void setLinearUnit(sfe_otos_linear_unit_t unit);

    /// @brief Gets the angular unit used by all methods using a pose
    /// @return Angular unit
    sfe_otos_angular_unit_t getAngularUnit();

    /// @brief Sets the angular unit used by all methods using a pose
    /// @param unit Angular unit
    void setAngularUnit(sfe_otos_angular_unit_t unit);

    /// @brief Gets the linear scalar used by the OTOS
    /// @param scalar Linear scalar
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getLinearScalar(float &scalar);

    /// @brief Sets the linear scalar used by the OTOS. Can be used to
    /// compensate for scaling issues with the sensor measurements
    /// @param scalar Linear scalar, must be between 0.872 and 1.127
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t setLinearScalar(float scalar);

    /// @brief Gets the angular scalar used by the OTOS
    /// @param scalar Angular scalar
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getAngularScalar(float &scalar);

    /// @brief Sets the angular scalar used by the OTOS. Can be used to
    /// compensate for scaling issues with the sensor measurements
    /// @param scalar Angular scalar, must be between 0.872 and 1.127
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t setAngularScalar(float scalar);

    /// @brief Resets the tracking algorithm, which resets the position to the
    /// origin, but can also be used to recover from some rare tracking errors
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t resetTracking();

    /// @brief Gets the signal processing configuration from the OTOS
    /// @param config Signal processing configuration
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getSignalProcessConfig(sfe_otos_signal_process_config_t &config);

    /// @brief Sets the signal processing configuration on the OTOS. This is
    /// primarily useful for creating and testing a new lookup table calibration
    /// @param config Signal processing configuration
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t setSignalProcessConfig(sfe_otos_signal_process_config_t &config);

    /// @brief Gets the status register from the OTOS, which includes warnings
    /// and errors reported by the sensor
    /// @param status Status register value
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getStatus(sfe_otos_status_t &status);

    /// @brief Gets the offset of the OTOS
    /// @param pose Offset of the sensor relative to the center of the robot
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getOffset(sfe_otos_pose2d_t &pose);

    /// @brief Sets the offset of the OTOS. This is useful if your sensor is
    /// mounted off-center from a robot. Rather than returning the position of
    /// the sensor, the OTOS will return the position of the robot
    /// @param pose Offset of the sensor relative to the center of the robot
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t setOffset(sfe_otos_pose2d_t &pose);

    /// @brief Gets the position measured by the OTOS
    /// @param pose Position measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getPosition(sfe_otos_pose2d_t &pose);

    /// @brief Sets the position measured by the OTOS. This is useful if your
    /// robot does not start at the origin, or you have another source of
    /// location information (eg. vision odometry); the OTOS will continue
    /// tracking from this position
    /// @param pose New position for the OTOS to track from
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t setPosition(sfe_otos_pose2d_t &pose);

    /// @brief Gets the velocity measured by the OTOS
    /// @param pose Velocity measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getVelocity(sfe_otos_pose2d_t &pose);

    /// @brief Gets the acceleration measured by the OTOS
    /// @param pose Acceleration measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getAcceleration(sfe_otos_pose2d_t &pose);

    /// @brief Gets the standard deviation of the measured position
    /// @param pose Standard deviation of the position measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    /// @note These values are just the square root of the diagonal elements of
    /// the covariance matrices of the Kalman filters used in the firmware, so
    /// they are just statistical quantities and do not represent actual error!
    sfTkError_t getPositionStdDev(sfe_otos_pose2d_t &pose);

    /// @brief Gets the standard deviation of the measured velocity
    /// @param pose Standard deviation of the velocity measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    /// @note These values are just the square root of the diagonal elements of
    /// the covariance matrices of the Kalman filters used in the firmware, so
    /// they are just statistical quantities and do not represent actual error!
    sfTkError_t getVelocityStdDev(sfe_otos_pose2d_t &pose);

    /// @brief Gets the standard deviation of the measured acceleration
    /// @param pose Standard deviation of the acceleration measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    /// @note These values are just the square root of the diagonal elements of
    /// the covariance matrices of the Kalman filters used in the firmware, so
    /// they are just statistical quantities and do not represent actual error!
    sfTkError_t getAccelerationStdDev(sfe_otos_pose2d_t &pose);

    /// @brief Gets the position, velocity, and acceleration measured by the
    /// OTOS in a single burst read
    /// @param pos Position measured by the OTOS
    /// @param vel Velocity measured by the OTOS
    /// @param acc Acceleration measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getPosVelAcc(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc);

    /// @brief Gets the standard deviation of the measured position, velocity,
    /// and acceleration in a single burst read
    /// @param pos Standard deviation of the position measured by the OTOS
    /// @param vel Standard deviation of the velocity measured by the OTOS
    /// @param acc Standard deviation of the acceleration measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getPosVelAccStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc);

    /// @brief Gets the position, velocity, acceleration, and standard deviation
    /// of each in a single burst read
    /// @param pos Position measured by the OTOS
    /// @param vel Velocity measured by the OTOS
    /// @param acc Acceleration measured by the OTOS
    /// @param posStdDev Standard deviation of the position measured by the OTOS
    /// @param velStdDev Standard deviation of the velocity measured by the OTOS
    /// @param accStdDev Standard deviation of the acceleration measured by the OTOS
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t getPosVelAccAndStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc,
                                      sfe_otos_pose2d_t &posStdDev, sfe_otos_pose2d_t &velStdDev,
                                      sfe_otos_pose2d_t &accStdDev);

    /// @brief Default I2C addresses of the Qwiic OTOS
    static constexpr uint8_t kDefaultAddress = 0x17;

    /// @brief Minimum scalar value for the linear and angular scalars
    static constexpr float kMinScalar = 0.872f;

    /// @brief Maximum scalar value for the linear and angular scalars
    static constexpr float kMaxScalar = 1.127f;

  protected:
    // Virtual function that must be implemented by the derived class to delay
    // for a given number of milliseconds
    virtual void delayMs(uint32_t ms) = 0;

    // Function to read raw pose registers and convert to specified units
    sfTkError_t readPoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH);

    // Function to write raw pose registers and convert from specified units
    sfTkError_t writePoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    // Function to convert raw pose registers to a pose structure
    void regsToPose(uint8_t *rawData, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH);

    // Function to convert a pose structure to raw pose registers
    void poseToRegs(uint8_t *rawData, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw);

    // I2C bus to use for communication
    sfTkII2C *_commBus;

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
