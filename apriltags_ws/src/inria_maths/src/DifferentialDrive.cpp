#include <stdexcept>
#include <inria_maths/DifferentialDrive.hpp>
#include <inria_maths/Clamp.h>

namespace inria {

DifferentialDrive::DifferentialDrive() :
    _wheelRadius(0.0),
    _wheelDistance(0.0),
    _maxVelLin(0.0),
    _maxVelAng(0.0),
    _gainLinP(0.0),
    _gainLinI(0.0),
    _gainAngP(0.0),
    _gainAngI(0.0),
    _readVelWheelLeft(0.0),
    _readVelWheelRight(0.0),
    _integralVelLin(0.0),
    _integralVelAng(0.0),
    _effortVelLin(0.0),
    _effortVelAng(0.0),
    _filterLowpassLin(),
    _filterBangbangLin(),
    _filterLowpassAng(),
    _filterBangbangAng()
{
}

void DifferentialDrive::setParameters(
    double wheelRadius,
    double wheelDistance,
    double cutoffFreq,
    double maxVelLin,
    double maxVelAng,
    double maxAccLin,
    double maxAccAng,
    double gainLinP,
    double gainLinI,
    double gainAngP,
    double gainAngI)
{
    //Check parameters
    if (
        wheelRadius <= 0.0 ||
        wheelDistance <= 0.0 ||
        cutoffFreq <= 0.0 ||
        maxVelLin <= 0.0 ||
        maxVelAng <= 0.0 ||
        maxAccLin <= 0.0 ||
        maxAccAng <= 0.0 ||
        gainLinP < 0.0 ||
        gainLinI < 0.0 ||
        gainAngP < 0.0 ||
        gainAngI < 0.0
    ) {
        throw std::logic_error(
            "inria::DifferentialDrive::setParameters: "
            "Invalid parameters.");
    }

    //Assign parameters
    _wheelRadius = wheelRadius;
    _wheelDistance = wheelDistance;
    _filterLowpassLin.cutoffFrequency() = cutoffFreq;
    _filterLowpassAng.cutoffFrequency() = cutoffFreq;
    _maxVelLin = maxVelLin;
    _maxVelAng = maxVelAng;
    _gainLinP = gainLinP;
    _gainLinI = gainLinI;
    _gainAngP = gainAngP;
    _gainAngI = gainAngI;
    _filterBangbangLin.maxVel() = maxAccLin;
    _filterBangbangAng.maxVel() = maxAccAng;
    _filterBangbangLin.maxAcc() = 2.0*maxAccLin;
    _filterBangbangAng.maxAcc() = 2.0*maxAccAng;
}

void DifferentialDrive::update(
    double dt,
    double cmdVelLin,
    double cmdVelAng,
    double readVelWheelLeft,
    double readVelWheelRight)
{
    //Check time step
    if (dt < 0.0) {
        throw std::logic_error(
            "inria::DifferentialDrive::update: "
            "Invalid time step.");
    }

    //Assign measured wheel velocities
    _readVelWheelLeft = readVelWheelLeft;
    _readVelWheelRight = readVelWheelRight;

    //Clamp maximum velocity
    cmdVelLin = ClampAbsolute(cmdVelLin, _maxVelLin);
    cmdVelAng = ClampAbsolute(cmdVelAng, _maxVelAng);

    //Filter command
    _filterLowpassLin.update(cmdVelLin, dt);
    _filterLowpassAng.update(cmdVelAng, dt);
    _filterBangbangLin.update(_filterLowpassLin.value(), dt);
    _filterBangbangAng.update(_filterLowpassAng.value(), dt);

    //Compute control effort
    double errorLin = _filterBangbangLin.value() - getReadVelLin();
    double errorAng = _filterBangbangAng.value() - getReadVelAng();
    _integralVelLin = dt*errorLin;
    _integralVelAng = dt*errorAng;
    _effortVelLin = 
        _filterBangbangLin.value() + 
        _gainLinP*errorLin +
        _gainLinI*_integralVelLin;
    _effortVelAng = 
        _filterBangbangAng.value() + 
        _gainAngP*errorAng +
        _gainAngI*_integralVelAng;
}

double DifferentialDrive::getEffortWheelLeft() const
{
    double velLin = _filterBangbangLin.value();
    double velAng = _filterBangbangAng.value();
    return (1.0/_wheelRadius)*(velLin - velAng*_wheelDistance);
}
double DifferentialDrive::getEffortWheelRight() const
{
    double velLin = _filterBangbangLin.value();
    double velAng = _filterBangbangAng.value();
    return (1.0/_wheelRadius)*(velLin + velAng*_wheelDistance);
}

double DifferentialDrive::getReadVelLin() const
{
    return 0.5*(_readVelWheelLeft+_readVelWheelRight)*_wheelRadius;
}
double DifferentialDrive::getReadVelAng() const
{
    return 0.5*_wheelRadius*(_readVelWheelRight-_readVelWheelLeft)/_wheelDistance;
}

double DifferentialDrive::getIntegralVelLin() const
{
    return _integralVelLin;
}
double DifferentialDrive::getIntegralVelAng() const
{
    return _integralVelAng;
}

double DifferentialDrive::getEffortVelLin() const
{
    return _effortVelLin;
}
double DifferentialDrive::getEffortVelAng() const
{
    return _effortVelAng;
}

void DifferentialDrive::reset()
{
    _readVelWheelLeft = 0.0;
    _readVelWheelRight = 0.0;
    _integralVelLin = 0.0;
    _integralVelAng = 0.0;
    _effortVelLin = 0.0;
    _effortVelAng = 0.0;
    _filterLowpassLin.reset(0.0);
    _filterLowpassAng.reset(0.0);
    _filterBangbangLin.reset(0.0);
    _filterBangbangAng.reset(0.0);
}

}

