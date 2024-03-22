#include "sfeQwiicOtos.h"

sfeQwiicOtos::sfeQwiicOtos()
{
    // Set communication bus to nullptr
    _commBus = nullptr;

    // Reset offset pose and its inverse to zero
    _offsetPose.x = 0;
    _offsetPose.y = 0;
    _offsetPose.h = 0;
    _offsetPoseInv.x = 0;
    _offsetPoseInv.y = 0;
    _offsetPoseInv.h = 0;

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
    int8_t rawScalar = (scalar - 1.0f) * 1000;

    // Write the scalar to the device
    return _commBus->writeRegisterByte(kOtosRegScalarH, (uint8_t)rawScalar);
}

void sfeQwiicOtos::setOffset(otos_pose2d_t &pose)
{
    // Copy the pose to the offset
    _offsetPose = pose;

    // Convert to meters and radians
    _offsetPose.x /= _meterToUnit;
    _offsetPose.y /= _meterToUnit;
    _offsetPose.h /= _radToUnit;

    // Compute the inverse of the offset
    _offsetPoseInv = invertPose(_offsetPose);
}

void sfeQwiicOtos::getOffset(otos_pose2d_t &pose)
{
    // Copy the offset to the pose
    pose = _offsetPose;

    // Convert to current units
    pose.x *= _meterToUnit;
    pose.y *= _meterToUnit;
    pose.h *= _radToUnit;
}

sfeTkError_t sfeQwiicOtos::resetTracking()
{
    // Set tracking reset bit
    return _commBus->writeRegisterByte(kOtosRegReset, 0x01);
}

sfeTkError_t sfeQwiicOtos::getPosition(otos_pose2d_t &pose)
{
    // Read the raw pose data
    otos_pose2d_t sensorPose;
    sfeTkError_t err = readPoseRegs(kOtosRegPosXL, sensorPose, kInt16ToMeter, kInt16ToRad);
    if(err != kSTkErrOk)
        return err;
    
    // Apply the offset
    pose = transformPose(sensorPose, _offsetPoseInv);

    // Convert to current units
    pose.x *= _meterToUnit;
    pose.y *= _meterToUnit;
    pose.h *= _radToUnit;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::setPosition(otos_pose2d_t &pose)
{
    otos_pose2d_t sensorPose;
    
    // Convert to meters and radians
    sensorPose.x = pose.x / _meterToUnit;
    sensorPose.y = pose.y / _meterToUnit;
    sensorPose.h = pose.h / _radToUnit;

    // Apply the offset
    sensorPose = transformPose(sensorPose, _offsetPose);

    return writePoseRegs(kOtosRegPosXL, sensorPose, kMeterToInt16, kRadToInt16);
}

sfeTkError_t sfeQwiicOtos::getVelocity(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegVelXL, pose, kInt16ToMeter, kInt16ToRps);
}

sfeTkError_t sfeQwiicOtos::setVelocity(otos_pose2d_t &pose)
{
    return writePoseRegs(kOtosRegVelXL, pose, kMeterToInt16, kRpsToInt16);
}

sfeTkError_t sfeQwiicOtos::getAccerlation(otos_pose2d_t &pose)
{
    return readPoseRegs(kOtosRegAccXL, pose, kInt16ToMeter, kInt16ToRpss);
}

sfeTkError_t sfeQwiicOtos::setAccerlation(otos_pose2d_t &pose)
{
    return writePoseRegs(kOtosRegAccXL, pose, kMeterToInt16, kRpssToInt16);
}

sfeTkError_t sfeQwiicOtos::readPoseRegs(uint8_t reg, otos_pose2d_t &pose, float rawToXY, float rawToH)
{
    size_t bytesRead = 0;
    uint8_t rawData[6];

    // Attempt to read the raw pose data
    sfeTkError_t err = _commBus->readRegisterRegion(reg, rawData, 6, bytesRead);

    // Check whether the read was successful
    if (err != kSTkErrOk)
        return err;

    // Store raw data
    int16_t rawX = (rawData[1] << 8) | rawData[0];
    int16_t rawY = (rawData[3] << 8) | rawData[2];
    int16_t rawH = (rawData[5] << 8) | rawData[4];

    // Store in pose
    pose.x = rawX * rawToXY;
    pose.y = rawY * rawToXY;
    pose.h = rawH * rawToH;

    // Done!
    return kSTkErrOk;
}

sfeTkError_t sfeQwiicOtos::writePoseRegs(uint8_t reg, otos_pose2d_t &pose, float xyToRaw, float hToRaw)
{
    // Convert pose units to raw data
    int16_t rawX = pose.x * xyToRaw;
    int16_t rawY = pose.y * xyToRaw;
    int16_t rawH = pose.h * hToRaw;
    
    // Store raw data in a temporary buffer
    uint8_t rawData[8];
    rawData[0] = rawX & 0xFF;
    rawData[1] = (rawX >> 8) & 0xFF;
    rawData[2] = rawY & 0xFF;
    rawData[3] = (rawY >> 8) & 0xFF;
    rawData[4] = rawH & 0xFF;
    rawData[5] = (rawH >> 8) & 0xFF;

    // Write the raw data to the device
    return _commBus->writeRegisterRegion(reg, rawData, 6);
}

otos_pose2d_t sfeQwiicOtos::transformPose(otos_pose2d_t &pose12, otos_pose2d_t &pose23)
{
    // We're going to compute this transformation as if the provided poses are
    // homogeneous transformation matrices. For the 2D case, the transformation
    // matrix is a 3x3 matrix T that looks like this:
    //
    //             [R11 R12 x]   [cos(h) -sin(h) x]
    // T = [R d] = [R21 R22 y] = [sin(h)  cos(h) y]
    //     [0 1]   [0   0   1]   [0       0      1]
    //
    // Where h is the heading and (x, y) is the translation from one frame to
    // another. If we know the transformation from frame 1 to frame 2 (T12), and
    // from frame 2 to frame 3 (T23), we can compute the transformation from
    // frame 1 to frame 3 (T13) by multiplying the transformation matrices:
    //
    // T13 = T12 * T23
    //
    // Which expands to:
    //
    // [cos(h13) -sin(h13) x13]   [cos(h12) -sin(h12) x12]   [cos(h23) -sin(h23) x23]
    // [sin(h13)  cos(h13) y13] = [sin(h12)  cos(h12) y12] * [sin(h23)  cos(h23) y23]
    // [0         0        1  ]   [0         0        1  ]   [0         0        1  ]
    //
    // For the x13 and y13 components, this expands to:
    //
    // x13 = x23 * cos(h12) - y23 * sin(h12) + x12
    // y13 = x23 * sin(h12) + y23 * cos(h12) + y12
    //
    // The upper left 2x2 corner is the resulting rotation matrix, where each
    // component depends only on h13. The expansion for each element is one of:
    //
    // cos(h13) = cos(h12) * cos(h23) - sin(h12) * sin(h23)
    // sin(h13) = sin(h12) * cos(h23) + cos(h12) * sin(h23)
    //
    // Instead of computing the inverse trigonometric functions, we can notice
    // that these are just the angle addition identities, so we can simplify by
    // just adding the angles:
    //
    // h1 = h2 + h3

    // Create struct to hold output
    otos_pose2d_t pose13;

    // Pre-compute sin and cos
    float cosh12 = cosf(pose12.h);
    float sinh12 = sinf(pose12.h);

    // Compute transformation following equations above
    pose13.x = pose23.x * cosh12 - pose23.y * sinh12 + pose12.x;
    pose13.y = pose23.x * sinh12 + pose23.y * cosh12 + pose12.y;
    pose13.h = pose23.h + pose12.h;

    // Wrap heading to +/- pi with fmod
    pose13.h = wrapAngle(pose13.h);

    // Done!
    return pose13;
}

otos_pose2d_t sfeQwiicOtos::invertPose(otos_pose2d_t &pose)
{
    // Similar to transformPose(), we're going to compute this transformation as
    // if the provided pose is a homogeneous transformation matrix T. The
    // inverse of T is given by:
    //
    //                               [ cos(h) sin(h) -x*cos(h)-y*sin(h)]
    // T^-1 = [R' d'] = [RT -RT*d] = [-sin(h) cos(h)  x*sin(h)-y*cos(h)]
    //        [0  1 ]   [0   1   ]   [0       0       1                ]
    //
    // Where R' and d' are the inverse rotation matrix and position vector, and
    // RT is the transpose of the rotation matrix. Solving for the inverse
    // position vector, we get:
    //
    // x' = -x * cos(h) - y * sin(h)
    // y' =  x * sin(h) - y * cos(h)
    //
    // The expansion for the inverse rotation matrix elements are one of:
    //
    // cos(h') =  cos(h)
    // sin(h') = -sin(h)
    //
    // Instead of computing the inverse trigonometric functions, we can notice
    // that h' and h must be opposite angles, so we can simplify:
    //
    // h' = -h

    // Create struct to hold output
    otos_pose2d_t poseInv;

    // Pre-compute sin and cos
    float cosh = cosf(pose.h);
    float sinh = sinf(pose.h);

    // Compute inverse following equations above
    poseInv.x = -pose.x * cosh - pose.y * sinh;
    poseInv.y = pose.x * sinh - pose.y * cosh;
    poseInv.h = -pose.h;

    // Done!
    return poseInv;
}

float sfeQwiicOtos::wrapAngle(float angle, otos_angular_unit_t unit)
{
    float bound = (unit == kOtosAngularUnitRadians) ? M_PI : 180.0;
    return fmodf(angle + bound, 2 * bound) - bound;
}
