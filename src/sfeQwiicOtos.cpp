/*
    SPDX-License-Identifier: MIT
    
    Copyright (c) 2024 SparkFun Electronics
*/

#include "sfeQwiicOtos.h"

sfeQwiicOtos::sfeQwiicOtos()
    : _commBus{nullptr}, _linearUnit{kSfeOtosLinearUnitInches}, _angularUnit{kSfeOtosAngularUnitDegrees},
      _meterToUnit{kMeterToInch}, _radToUnit{kRadianToDegree}
{
    // Nothing to do here!
}

sfeTkError_t sfeQwiicOtos::begin(sfeTkII2C *commBus)
{
    // Nullptr check
    if (commBus == nullptr)
        return kSTkErrFail;

    // Check the device address
    if (commBus->address() != kDefaultAddress)
        return kSTkErrFail;

    // Set bus pointer
    _commBus = commBus;

    // Just check if the device is connected, no other setup is needed
    return isConnected();
}

sfeTkError_t sfeQwiicOtos::isConnected()
{
    // First ping the device address
    sfeTkError_t err = _commBus->ping();
    if(err != kSTkErrOk)
        return err;
    
    // Read the product ID
    uint8_t prodId;
    err = _commBus->readRegisterByte(kRegProductId, prodId);
    if(err != kSTkErrOk)
        return err;

    // Check if the product ID is correct
    if(prodId != kProductId)
        return kSTkErrFail;

    // Everything checks out, we must be connected!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getVersionInfo(sfe_otos_version_t &hwVersion, sfe_otos_version_t &fwVersion)
{
    // Read hardware and firmware version registers
    uint8_t rawData[2];
    size_t readBytes;
    sfeTkError_t err = _commBus->readRegisterRegion(kRegHwVersion, rawData, 2, readBytes);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(readBytes != 2)
        return kSTkErrFail;

    // Store the version info
    hwVersion.value = rawData[0];
    fwVersion.value = rawData[1];

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::selfTest()
{
    // Write the self-test register to start the test
    sfe_otos_self_test_config_t selfTest;
    selfTest.start = 1;
    sfeTkError_t err = _commBus->writeRegisterByte(kRegSelfTest, selfTest.value);
    if(err != kSTkErrOk)
        return err;

    // Loop until self-test is done, should only take ~20ms as of firmware v1.0
    for(int i = 0; i < 10; i++)
    {
        // Give a short delay between reads
        delayMs(5);

        // Read the self-test register
        err = _commBus->readRegisterByte(kRegSelfTest, selfTest.value);
        if(err != kSTkErrOk)
            return err;
        
        // Check if the self-test is done
        if(selfTest.inProgress == 0)
        {
            break;
        }
    }

    // Check if the self-test passed
    return (selfTest.pass == 1) ? kSTkErrOk : kSTkErrFail;
}

sfeTkError_t sfeQwiicOtos::calibrateImu(uint8_t numSamples, bool waitUntilDone)
{
    // Write the number of samples to the device
    sfeTkError_t err = _commBus->writeRegisterByte(kRegImuCalib, numSamples);
    if(err != kSTkErrOk)
        return err;
    
    // Do we need to wait until the calibration finishes?
    if(!waitUntilDone)
        return kSTkErrOk;
    
    // Wait for the calibration to finish, which is indicated by the IMU
    // calibration register reading zero, or until we reach the maximum number
    // of read attempts
    for(uint8_t numAttempts = numSamples; numAttempts > 0; numAttempts--)
    {
        // Give a short delay between reads. As of firmware v1.0, samples take
        // 2.4ms each, so 3ms should guarantee the next sample is done. This
        // also ensures the max attempts is not exceeded in normal operation
        delayMs(3);

        // Read the gryo calibration register value
        uint8_t calibrationValue;
        err = _commBus->readRegisterByte(kRegImuCalib, calibrationValue);
        if(err != kSTkErrOk)
            return err;
        
        // Check if calibration is done
        if(calibrationValue == 0)
            return kSTkErrOk;
    }

    // Max number of attempts reached, calibration failed
    return kSTkErrFail;
}

sfe_otos_linear_unit_t sfeQwiicOtos::getLinearUnit()
{
    return _linearUnit;
}

void sfeQwiicOtos::setLinearUnit(sfe_otos_linear_unit_t unit)
{
    // Check if this unit is already set
    if(unit == _linearUnit)
        return;

    // Store new unit
    _linearUnit = unit;
    
    // Compute conversion factor to new units
    _meterToUnit = (unit == kSfeOtosLinearUnitMeters) ? 1.0f : kMeterToInch;
}

sfe_otos_angular_unit_t sfeQwiicOtos::getAngularUnit()
{
    return _angularUnit;
}

void sfeQwiicOtos::setAngularUnit(sfe_otos_angular_unit_t unit)
{
    // Check if this unit is already set
    if(unit == _angularUnit)
        return;

    // Store new unit
    _angularUnit = unit;

    // Compute conversion factor to new units
    _radToUnit = (unit == kSfeOtosAngularUnitRadians) ? 1.0f : kRadianToDegree;
}

sfeTkError_t sfeQwiicOtos::getLinearScalar(float &scalar)
{
    // Read the linear scalar from the device
    uint8_t rawScalar;
    sfeTkError_t err = _commBus->readRegisterByte(kRegScalarLinear, rawScalar);
    if(err != kSTkErrOk)
        return kSTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::setLinearScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if(scalar < kMinScalar || scalar > kMaxScalar)
        return kSTkErrFail;
    
    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return _commBus->writeRegisterByte(kRegScalarLinear, rawScalar);
}

sfeTkError_t sfeQwiicOtos::getAngularScalar(float &scalar)
{
    // Read the angular scalar from the device
    uint8_t rawScalar;
    sfeTkError_t err = _commBus->readRegisterByte(kRegScalarAngular, rawScalar);
    if(err != kSTkErrOk)
        return kSTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::setAngularScalar(float scalar)
{
    // Check if the scalar is out of bounds
    if(scalar < kMinScalar || scalar > kMaxScalar)
        return kSTkErrFail;
    
    // Convert to integer, multiples of 0.1% (+0.5 to round instead of truncate)
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    // Write the scalar to the device
    return _commBus->writeRegisterByte(kRegScalarAngular, rawScalar);
}

sfeTkError_t sfeQwiicOtos::resetTracking()
{
    // Set tracking reset bit
    return _commBus->writeRegisterByte(kRegReset, 0x01);
}

sfeTkError_t sfeQwiicOtos::getSignalProcess(sfe_otos_signal_process_config_t &config)
{
    // Read the signal process register
    return _commBus->readRegisterByte(kRegSignalProcess, config.value);
}

sfeTkError_t sfeQwiicOtos::setSignalProcess(sfe_otos_signal_process_config_t &config)
{
    // Write the signal process register
    return _commBus->writeRegisterByte(kRegSignalProcess, config.value);
}

sfeTkError_t sfeQwiicOtos::getOffset(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegOffXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::getStatus(sfe_otos_status_t &status)
{
    return _commBus->readRegisterByte(kRegStatus, status.value);
}

sfeTkError_t sfeQwiicOtos::setOffset(sfe_otos_pose2d_t &pose)
{
    return writePoseRegs(kRegOffXL, pose, kMeterToInt16, kRadToInt16);
}

sfeTkError_t sfeQwiicOtos::getPosition(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::setPosition(sfe_otos_pose2d_t &pose)
{
    return writePoseRegs(kRegPosXL, pose, kMeterToInt16, kRadToInt16);
}

sfeTkError_t sfeQwiicOtos::getVelocity(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelXL, pose, kInt16ToMps, kInt16ToRps);
}

sfeTkError_t sfeQwiicOtos::getAcceleration(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccXL, pose, kInt16ToMpss, kInt16ToRpss);
}

sfeTkError_t sfeQwiicOtos::getPositionStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegPosStdXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::getVelocityStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegVelStdXL, pose, kInt16ToMps, kInt16ToRps);
}

sfeTkError_t sfeQwiicOtos::getAccelerationStdDev(sfe_otos_pose2d_t &pose)
{
    return readPoseRegs(kRegAccStdXL, pose, kInt16ToMpss, kInt16ToRpss);
}

sfeTkError_t sfeQwiicOtos::getPosVelAcc(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead;
    sfeTkError_t err = _commBus->readRegisterRegion(kRegPosXL, rawData, 18, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 18)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getPosVelAccStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead;
    sfeTkError_t err = _commBus->readRegisterRegion(kRegPosStdXL, rawData, 18, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 18)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getPosVelAccAndStdDev(sfe_otos_pose2d_t &pos, sfe_otos_pose2d_t &vel, sfe_otos_pose2d_t &acc,
                                    sfe_otos_pose2d_t &posStdDev, sfe_otos_pose2d_t &velStdDev, sfe_otos_pose2d_t &accStdDev)
{
    // Read all pose registers
    uint8_t rawData[36];
    size_t bytesRead;
    sfeTkError_t err = _commBus->readRegisterRegion(kRegPosXL, rawData, 36, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 36)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMpss, kInt16ToRpss);
    regsToPose(rawData + 18, posStdDev, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 24, velStdDev, kInt16ToMps, kInt16ToRps);
    regsToPose(rawData + 30, accStdDev, kInt16ToMpss, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::readPoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    size_t bytesRead;
    uint8_t rawData[6];

    // Attempt to read the raw pose data
    sfeTkError_t err = _commBus->readRegisterRegion(reg, rawData, 6, bytesRead);
    if (err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if (bytesRead != 6)
        return kSTkErrFail;

    regsToPose(rawData, pose, rawToXY, rawToH);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::writePoseRegs(uint8_t reg, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Store raw data in a temporary buffer
    uint8_t rawData[6];
    poseToRegs(rawData, pose, xyToRaw, hToRaw);

    // Write the raw data to the device
    return _commBus->writeRegisterRegion(reg, rawData, 6);
}

void sfeQwiicOtos::regsToPose(uint8_t *rawData, sfe_otos_pose2d_t &pose, float rawToXY, float rawToH)
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

void sfeQwiicOtos::poseToRegs(uint8_t *rawData, sfe_otos_pose2d_t &pose, float xyToRaw, float hToRaw)
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
