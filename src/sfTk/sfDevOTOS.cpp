/**
 * @file sfDevOTOS.cpp
 * @brief Implementation of the SparkFun Qwiic OTOS driver class
 * @details This file contains the implementation of methods for the sfDevOTOS class,
 *          which provides a C++ interface for the SparkFun Qwiic Optical Tracking
 *          Odometry Sensor (OTOS).
 *
 * Implementation Features:
 * - Platform-agnostic I2C communication using SparkFun Toolkit
 * - Unit conversion between metric/imperial and radians/degrees
 * - Register-level device interaction
 * - Data conversion between raw register values and floating point numbers
 * - Error handling and validation
 *
 * @author SparkFun Electronics
 * @date February 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_Toolkit
 */

/*******************************************************************************
    sfDevOTOS.h - C++ driver implementation for the SparkFun Qwiic Optical
    Tracking Odometry Sensor (OTOS).
*******************************************************************************/

#include "sfDevOTOS.h"

sfDevOTOS::sfDevOTOS()
    : _commBus{nullptr}, _linearUnit{kSfeOtosLinearUnitInches}, _angularUnit{kSfeOtosAngularUnitDegrees},
      _meterToUnit{kMeterToInch}, _radToUnit{kRadianToDegree}
{
    // Nothing to do here!
}

sfTkError_t sfDevOTOS::begin(sfTkII2C *commBus)
{
    // Nullptr check
    if (commBus == nullptr)
        return ksfTkErrFail;

    // Check the device address
    if (commBus->address() != kDefaultAddress)
        return ksfTkErrFail;

    // Set bus pointer
    _commBus = commBus;

    // Just check if the device is connected, no other setup is needed
    return isConnected();
}

sfTkError_t sfDevOTOS::isConnected()
{
    // First ping the device address
    sfTkError_t err = _commBus->ping();
    if (err != ksfTkErrOk)
        return err;

    // Read the product ID
    uint8_t prodId;
    err = _commBus->readRegister(kRegProductId, prodId);
    if (err != ksfTkErrOk)
        return err;

    // Check if the product ID is correct
    if (prodId != kProductId)
        return ksfTkErrFail;

    // Everything checks out, we must be connected!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::getVersionInfo(sfe_otos_version_t &hwVersion, sfe_otos_version_t &fwVersion)
{
    // Read hardware and firmware version registers
    uint8_t rawData[2];
    size_t readBytes;
    sfTkError_t err = _commBus->readRegister(kRegHwVersion, rawData, sizeof(rawData), readBytes);
    if (err != ksfTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (readBytes != 2)
        return ksfTkErrFail;

    // Store the version info
    hwVersion.value = rawData[0];
    fwVersion.value = rawData[1];

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::selfTest()
{
    // Write the self-test register to start the test
    sfe_otos_self_test_config_t selfTest;
    selfTest.start = 1;
    sfTkError_t err = _commBus->writeRegister(kRegSelfTest, selfTest.value);
    if (err != ksfTkErrOk)
        return err;

    // Loop until self-test is done, should only take ~20ms as of firmware v1.0
    for (int i = 0; i < 10; i++)
    {
        // Give a short delay between reads
        delayMs(5);

        // Read the self-test register
        err = _commBus->readRegister(kRegSelfTest, selfTest.value);
        if (err != ksfTkErrOk)
            return err;

        // Check if the self-test is done
        if (selfTest.inProgress == 0)
        {
            break;
        }
    }

    // Check if the self-test passed
    return (selfTest.pass == 1) ? ksfTkErrOk : ksfTkErrFail;
}

sfTkError_t sfDevOTOS::calibrateImu(uint8_t numSamples, bool waitUntilDone)
{
    // Write the number of samples to the device
    sfTkError_t err = _commBus->writeRegister(kRegImuCalib, numSamples);
    if (err != ksfTkErrOk)
        return err;

    // Wait 1 sample period (2.4ms) to ensure the register updates
    delayMs(3);

    // Do we need to wait until the calibration finishes?
    if (!waitUntilDone)
        return ksfTkErrOk;

    // Wait for the calibration to finish, which is indicated by the IMU
    // calibration register reading zero, or until we reach the maximum number
    // of read attempts
    for (uint8_t numAttempts = numSamples; numAttempts > 0; numAttempts--)
    {
        // Read the gryo calibration register value
        uint8_t calibrationValue;
        err = _commBus->readRegister(kRegImuCalib, calibrationValue);
        if (err != ksfTkErrOk)
            return err;

        // Check if calibration is done
        if (calibrationValue == 0)
            return ksfTkErrOk;

        // Give a short delay between reads. As of firmware v1.0, samples take
        // 2.4ms each, so 3ms should guarantee the next sample is done. This
        // also ensures the max attempts is not exceeded in normal operation
        delayMs(3);
    }

    // Max number of attempts reached, calibration failed
    return ksfTkErrFail;
}

sfTkError_t sfDevOTOS::getImuCalibrationProgress(uint8_t &numSamples)
{
    // Read the IMU calibration register
    return _commBus->readRegister(kRegImuCalib, numSamples);
}

sfe_otos_linear_unit_t sfDevOTOS::getLinearUnit()
{
    return _linearUnit;
}

void sfDevOTOS::setLinearUnit(sfe_otos_linear_unit_t unit)
{
    // Check if this unit is already set
    if (unit == _linearUnit)
        return;

    // Store new unit
    _linearUnit = unit;

    // Compute conversion factor to new units
    _meterToUnit = (unit == kSfeOtosLinearUnitMeters) ? 1.0f : kMeterToInch;
}

sfe_otos_angular_unit_t sfDevOTOS::getAngularUnit()
{
    return _angularUnit;
}

void sfDevOTOS::setAngularUnit(sfe_otos_angular_unit_t unit)
{
    // Check if this unit is already set
    if (unit == _angularUnit)
        return;

    // Store new unit
    _angularUnit = unit;

    // Compute conversion factor to new units
    _radToUnit = (unit == kSfeOtosAngularUnitRadians) ? 1.0f : kRadianToDegree;
}

sfTkError_t sfDevOTOS::getLinearScalar(float &scalar)
{
    // Read the linear scalar from the device
    uint8_t rawScalar;
    sfTkError_t err = _commBus->readRegister(kRegScalarLinear, rawScalar);
    if (err != ksfTkErrOk)
        return ksfTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::setLinearScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if (scalar < kMinScalar || scalar > kMaxScalar)
        return ksfTkErrFail;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return _commBus->writeRegister(kRegScalarLinear, rawScalar);
}

sfTkError_t sfDevOTOS::getAngularScalar(float &scalar)
{
    // Read the angular scalar from the device
    uint8_t rawScalar;
    sfTkError_t err = _commBus->readRegister(kRegScalarAngular, rawScalar);
    if (err != ksfTkErrOk)
        return ksfTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::setAngularScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if (scalar < kMinScalar || scalar > kMaxScalar)
        return ksfTkErrFail;

    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return _commBus->writeRegister(kRegScalarAngular, rawScalar);
}

sfTkError_t sfDevOTOS::resetTracking()
{
    // Set tracking reset bit
    return _commBus->writeRegister(kRegReset, (uint8_t)0x01);
}

sfTkError_t sfDevOTOS::getSignalProcessConfig(sfe_otos_signal_process_config_t &config)
{
    // Read the signal process register
    return _commBus->readRegister(kRegSignalProcess, config.value);
}

sfTkError_t sfDevOTOS::setSignalProcessConfig(sfe_otos_signal_process_config_t &config)
{
    // Write the signal process register
    return _commBus->writeRegister(kRegSignalProcess, config.value);
}

sfTkError_t sfDevOTOS::getStatus(sfe_otos_status_t &status)
{
    return _commBus->readRegister(kRegStatus, status.value);
}

sfTkError_t sfDevOTOS::getOffset(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegOffXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfTkError_t sfDevOTOS::setOffset(sfe_otos_pose2d_t &pose)
{
    return writePoseRegs(kRegOffXL, pose, kMeterToInt16, kRadToInt16);
}

sfTkError_t sfDevOTOS::getPosition(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfTkError_t sfDevOTOS::setPosition(sfe_otos_pose2d_t &pose)
{
    return writePoseRegs(kRegPosXL, pose, kMeterToInt16, kRadToInt16);
}

sfTkError_t sfDevOTOS::getVelocity(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelXL, pose, kInt16ToMps, kInt16ToRps);
}

sfTkError_t sfDevOTOS::getAcceleration(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccXL, pose, kInt16ToMpss, kInt16ToRpss);
}

sfTkError_t sfDevOTOS::getPositionStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosStdXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfTkError_t sfDevOTOS::getVelocityStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelStdXL, pose, kInt16ToMps, kInt16ToRps);
}

sfTkError_t sfDevOTOS::getAccelerationStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccStdXL, pose, kInt16ToMpss, kInt16ToRpss);
}

sfTkError_t sfDevOTOS::getPosVelAcc(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead;
    sfTkError_t err = _commBus->readRegister(kRegPosXL, rawData, 18, bytesRead);
    if (err != ksfTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 18)
        return ksfTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::getPosVelAccStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead;
    sfTkError_t err = _commBus->readRegister(kRegPosStdXL, rawData, 18, bytesRead);
    if (err != ksfTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 18)
        return ksfTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::getPosVelAccAndStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc,
                                             sfe_otos_pose2d_t &posStdDev, sfe_otos_pose2d_t &velStdDev,
                                             sfe_otos_pose2d_t &accStdDev)
{
    // Read all pose registers
    uint8_t rawData[36];
    size_t bytesRead;
    sfTkError_t err = _commBus->readRegister(kRegPosXL, rawData, 36, bytesRead);
    if (err != ksfTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 36)
        return ksfTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);
    regsToPose(rawData + 18, posStdDev, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 24, velStdDev, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 30, accStdDev, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::readPoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    size_t bytesRead;
    uint8_t rawData[6];

    // Attempt to read the raw pose data
    sfTkError_t err = _commBus->readRegister(reg, rawData, 6, bytesRead);
    if (err != ksfTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 6)
        return ksfTkErrFail;

    regsToPose(rawData, pose, rawToXY, rawToH);

    // Done!
    return ksfTkErrOk;
}

sfTkError_t sfDevOTOS::writePoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Store raw data in a temporary buffer
    uint8_t rawData[6];
    poseToRegs(rawData, pose, xyToRaw, hToRaw);

    // Write the raw data to the device
    return _commBus->writeRegister(reg, rawData, 6);
}

void sfDevOTOS::regsToPose(uint8_t *rawData, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    // Store raw data
    int16_t rawX = (rawData[1] << 8) | rawData[0];
    int16_t rawY = (rawData[3] << 8) | rawData[2];
    int16_t rawH = (rawData[5] << 8) | rawData[4];

    // Store in pose and convert to units
    pose.x = rawX * rawToXY * _meterToUnit;
    pose.y = rawY * rawToXY * _meterToUnit;
    pose.h = rawH * rawToH * _radToUnit;
}

void sfDevOTOS::poseToRegs(uint8_t *rawData, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Convert pose units to raw data
    int16_t rawX = pose.x * xyToRaw / _meterToUnit;
    int16_t rawY = pose.y * xyToRaw / _meterToUnit;
    int16_t rawH = pose.h * hToRaw / _radToUnit;

    // Store raw data in buffer
    rawData[0] = rawX & 0xFF;
    rawData[1] = (rawX >> 8) & 0xFF;
    rawData[2] = rawY & 0xFF;
    rawData[3] = (rawY >> 8) & 0xFF;
    rawData[4] = rawH & 0xFF;
    rawData[5] = (rawH >> 8) & 0xFF;
}
