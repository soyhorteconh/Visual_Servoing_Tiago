#ifndef INRIA_MATHS_DIFFERENTIALDRIVE_HPP
#define INRIA_MATHS_DIFFERENTIALDRIVE_HPP

#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/TrajectoryBangBangAcc.hpp>

namespace inria {

/**
 * DifferentialDrive
 *
 * Controller and odometry estimation for 
 * two wheels differential drive
 */
class DifferentialDrive
{
    public:

        /**
         * Empty initialization
         */
        DifferentialDrive();

        /**
         * Set filter and controller parameters
         *
         * @param wheelRadius Geometric wheel radius.
         * @param wheelDistance Geometric lateral distance 
         * between base and wheels.
         * @param cutoffFreq Command filter cutoff frequency.
         * @param maxVelLin Command maximum linear velocity.
         * @param maxVelAng Command maximum angular velocity.
         * @param maxAccLin Command maximum linear acceleration.
         * @param maxAccAng Command maximum angular acceleration.
         * @param gainLinP Linear velocity effort P gain.
         * @param gainLinI Linear velocity effort I gain.
         * @param gainAngP Angular velocity effort P gain.
         * @param gainAngI Angular velocity effort I gain.
         */
        void setParameters(
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
            double gainAngI);

        /**
         * Update command filters and compute wheel effort.
         * 
         * @param dt Time step.
         * @param cmdVelLin Target forward linear velocity.
         * @param cmdVelAng Target planar angular velocity.
         * @param readVelWheelLeft Measured left wheel velocity.
         * @param readVelWheelRight Measured right wheel velocity.
         */
        void update(
            double dt,
            double cmdVelLin,
            double cmdVelAng,
            double readVelWheelLeft,
            double readVelWheelRight);

        /**
         * @return left and right velocity 
         * wheel effort
         */
        double getEffortWheelLeft() const;
        double getEffortWheelRight() const;
        
        /**
         * @return forward linear and planar 
         * angular measured velocity.
         */
        double getReadVelLin() const;
        double getReadVelAng() const;
        
        /**
         * @return forward linear and planar 
         * angular control integral velocity.
         */
        double getIntegralVelLin() const;
        double getIntegralVelAng() const;
        
        /**
         * @return forward linear and planar 
         * angular effort velocity.
         */
        double getEffortVelLin() const;
        double getEffortVelAng() const;

        /**
         * Reset velocity command to zero
         */
        void reset();

    private:

        /**
         * Geometric parameters
         */
        double _wheelRadius;
        double _wheelDistance;

        /**
         * Absolute maximum linear and angular 
         * velocity command
         */
        double _maxVelLin;
        double _maxVelAng;

        /**
         * Controller proportional 
         * and integral gains
         */
        double _gainLinP;
        double _gainLinI;
        double _gainAngP;
        double _gainAngI;

        /**
         * Last received wheel velocities
         */
        double _readVelWheelLeft;
        double _readVelWheelRight;

        /**
         * Integral term for linear and 
         * angular velocity
         */
        double _integralVelLin;
        double _integralVelAng;

        /**
         * Last computed linear and angular 
         * velocity effort
         */
        double _effortVelLin;
        double _effortVelAng;

        /**
         * Filters for linear and angular 
         * velocity command
         */
        FilterExponential<double> _filterLowpassLin;
        FilterBangBangAcc<double> _filterBangbangLin;
        FilterExponential<double> _filterLowpassAng;
        FilterBangBangAcc<double> _filterBangbangAng;
};

}

#endif

