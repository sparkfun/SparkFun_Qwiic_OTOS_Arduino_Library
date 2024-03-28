#include "sfeQwiicOtos.h"

sfeQwiicOtos::sfeQwiicOtos()
{
    // Set communication bus to nullptr
    _commBus = nullptr;

    // Set default units to inches and degrees
    _linearUnit = kOtosLinearUnitInches;
    _angularUnit = kOtosAngularUnitDegrees;

    // Set conversion factors to default units
    _meterToUnit = kMeterToInch;
    _radToUnit = kRadianToDegree;
}

sfeTkError_t sfeQwiicOtos::begin(sfeTkII2C *commBus)
{
    // Nullptr check
    if (commBus == nullptr)
        return kSTkErrFail;

    // Check the device address
    if (commBus->address() < 0x08 || commBus->address() > 0x77)
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
    uint8_t prodId = 0;
    err = _commBus->readRegisterByte(kOtosRegProductId, prodId);
    if(err != kSTkErrOk)
        return err;

    // Check if the product ID is correct
    if(prodId != kOtosProductId)
        return kSTkErrFail;

    // Everything checks out, we must be connected!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getVersionInfo(otos_version_t &hwVersion, otos_version_t &fwVersion)
{
    // Read hardware and firmware version registers
    uint8_t rawData[2];
    size_t readBytes = 0;
    sfeTkError_t err = _commBus->readRegisterRegion(kOtosRegHwVersion, rawData, 2, readBytes);
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

sfeTkError_t sfeQwiicOtos::calibrateImu(uint8_t numSamples, bool waitUntilDone)
{
    // Write the number of samples to the device
    sfeTkError_t err = _commBus->writeRegisterByte(kOtosRegImuCalib, numSamples);
    if(err != kSTkErrOk)
        return err;
    
    // Do we need to wait until the calibration finishes?
    if(!waitUntilDone)
        return kSTkErrOk;
    
    // Wait for the calibration to finish, which is indicated by the gyro
    // calibration register reading zero
    uint8_t calibrationValue = numSamples;
    while(calibrationValue != 0)
    {
        // Give a short delay between reads
        delayMs(2);

        // Read the gryo calibration register value
        err = _commBus->readRegisterByte(kOtosRegImuCalib, calibrationValue);
        if(err != kSTkErrOk)
            return err;
    }

    // Done!
    return kSTkErrOk;
}

otos_linear_unit_t sfeQwiicOtos::getLinearUnit()
{
    return _linearUnit;
}

void sfeQwiicOtos::setLinearUnit(otos_linear_unit_t unit)
{
    // Check if this unit is already set
    if(unit == _linearUnit)
        return;

    // Store new unit
    _linearUnit = unit;
    
    // Compute conversion factor to new units
    _meterToUnit = (unit == kOtosLinearUnitMeters) ? 1.0f : kMeterToInch;
}

otos_angular_unit_t sfeQwiicOtos::getAngularUnit()
{
    return _angularUnit;
}

void sfeQwiicOtos::setAngularUnit(otos_angular_unit_t unit)
{
    // Check if this unit is already set
    if(unit == _angularUnit)
        return;

    // Store new unit
    _angularUnit = unit;

    // Compute conversion factor to new units
    _radToUnit = (unit == kOtosAngularUnitRadians) ? 1.0f : kRadianToDegree;
}

sfeTkError_t sfeQwiicOtos::getLinearScalar(float &scalar)
{
    // Read the linear scalar from the device
    uint8_t rawScalar = 0;
    sfeTkError_t err = _commBus->readRegisterByte(kOtosRegScalarXY, rawScalar);
    if(err != kSTkErrOk)
        return kSTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::setLinearScalar(float scalar)
{
    // Check if the scalar is out of bounds, can only be 0.872 to 1.127
    if(scalar < 0.872f || scalar > 1.127f)
        return kSTkErrFail;
    
    // Convert to raw integer, multiples of 0.1%
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000);

    // Write the scalar to the device
    return _commBus->writeRegisterByte(kOtosRegScalarXY, rawScalar);
}

sfeTkError_t sfeQwiicOtos::getAngularScalar(float &scalar)
{
    // Read the angular scalar from the device
    uint8_t rawScalar = 0;
    sfeTkError_t err = _commBus->readRegisterByte(kOtosRegScalarH, rawScalar);
    if(err != kSTkErrOk)
        return kSTkErrFail;

    // Convert to float, multiples of 0.1%
    scalar = (((int8_t)rawScalar) * 0.001f) + 1.0f;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::setAngularScalar(float scalar)
{
    // Check if the scalar is out of bounds, can only be 0.872 to 1.127
    if(scalar < 0.872f || scalar > 1.127f)
        return kSTkErrFail;
    
    // Convert to raw integer, multiples of 0.1%
    uint8_t rawScalar = (int8_t)((scalar - 1.0f) * 1000);

    // Write the scalar to the device
    return _commBus->writeRegisterByte(kOtosRegScalarH, rawScalar);
}

sfeTkError_t sfeQwiicOtos::resetTracking()
{
    // Set tracking reset bit
    return _commBus->writeRegisterByte(kOtosRegReset, 0x01);
}

sfeTkError_t sfeQwiicOtos::getOffset(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegOffXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::setOffset(otos_pose2d_t &pose)
{
    return writePoseRegs(kOtosRegOffXL, pose, kMeterToInt16, kRadToInt16);
}

sfeTkError_t sfeQwiicOtos::getPosition(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegPosXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::setPosition(otos_pose2d_t &pose)
{
    return writePoseRegs(kOtosRegPosXL, pose, kMeterToInt16, kRadToInt16);
}

sfeTkError_t sfeQwiicOtos::getVelocity(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegVelXL, pose, kInt16ToMeter, kInt16ToRps);
}

sfeTkError_t sfeQwiicOtos::getAccerlation(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegAccXL, pose, kInt16ToMeter, kInt16ToRpss);
}

sfeTkError_t sfeQwiicOtos::getPositionStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegPosStdXL, pose, kInt16ToMeter, kInt16ToRad);
}

sfeTkError_t sfeQwiicOtos::getVelocityStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegVelStdXL, pose, kInt16ToMeter, kInt16ToRps);
}

sfeTkError_t sfeQwiicOtos::getAccerlationStdDev(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegAccStdXL, pose, kInt16ToMeter, kInt16ToRpss);
}

sfeTkError_t sfeQwiicOtos::getPosVelAcc(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead = 0;
    sfeTkError_t err = _commBus->readRegisterRegion(kOtosRegPosXL, rawData, 18, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 18)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMeter, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMeter, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getPosVelAccStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc)
{
    // Read all pose registers
    uint8_t rawData[18];
    size_t bytesRead = 0;
    sfeTkError_t err = _commBus->readRegisterRegion(kOtosRegPosStdXL, rawData, 18, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 18)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMeter, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMeter, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::getPosVelAccAndStdDev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc, otos_pose2d_t &posStdDev, otos_pose2d_t &velStdDev, otos_pose2d_t &accStdDev)
{
    // Read all pose registers
    uint8_t rawData[36];
    size_t bytesRead = 0;
    sfeTkError_t err = _commBus->readRegisterRegion(kOtosRegPosXL, rawData, 36, bytesRead);
    if(err != kSTkErrOk)
        return err;

    // Check if we read the correct number of bytes
    if(bytesRead != 36)
        return kSTkErrFail;

    // Convert raw data to pose units
    regsToPose(rawData, pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 6, vel, kInt16ToMeter, kInt16ToRps);
    regsToPose(rawData + 12, acc, kInt16ToMeter, kInt16ToRpss);
    regsToPose(rawData + 18, posStdDev, kInt16ToMeter, kInt16ToRad);
    regsToPose(rawData + 24, velStdDev, kInt16ToMeter, kInt16ToRps);
    regsToPose(rawData + 30, accStdDev, kInt16ToMeter, kInt16ToRpss);

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::readPoseRegs(uint8_t reg, otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    size_t bytesRead = 0;
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

sfeTkError_t sfeQwiicOtos::writePoseRegs(uint8_t reg, otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Store raw data in a temporary buffer
    uint8_t rawData[6];
    poseToRegs(rawData, pose, xyToRaw, hToRaw);

    // Write the raw data to the device
    return _commBus->writeRegisterRegion(reg, rawData, 6);
}

void sfeQwiicOtos::regsToPose(uint8_t *rawData, otos_pose2d_t &pose, float rawToXY, float rawToH)
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

void sfeQwiicOtos::poseToRegs(uint8_t *rawData, otos_pose2d_t &pose, float xyToRaw, float hToRaw)
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
