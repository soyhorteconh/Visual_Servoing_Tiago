#ifndef VELOCITY_CONTROL
#define VELOCITY_CONTROL

#include <Eigen/Geometry>
#include <inria_maths/AxisAngle.h>
#include <utility>
#include <iostream>

class VelocityControl
{
    private:
        double kp;
        double dt;
        double linear_max_velocity;
        double angular_max_velocity;
        Eigen::Matrix3d estimated_rotation;
        Eigen::Vector3d estimated_position;

    public:
        VelocityControl(double kp, double dt, double linear_max_velocity, double angular_max_velocity) :
            kp(kp), 
            dt(dt),
            linear_max_velocity(linear_max_velocity),
            angular_max_velocity(angular_max_velocity)
        {}

        // saturated velocity controller
        std::pair<Eigen::Vector3d,Eigen::Vector3d> controller(
            const Eigen::Vector3d desired_position, const Eigen::Matrix3d desired_rotation,
            const Eigen::Vector3d current_position, const Eigen::Matrix3d current_rotation)
        {
            // error
            Eigen::Matrix3d rotation_error = desired_rotation * current_rotation.transpose();
            Eigen::Vector3d position_error = - desired_rotation * current_rotation.transpose() * current_position + desired_position;

            // velocity
            //Eigen::Vector3d angular_v;
            Eigen::Vector3d angular_v = kp * (inria::MatrixToAxis(rotation_error));
            Eigen::Vector3d linear_v = kp * position_error;

            // saturated control
            double angular_norm = angular_v.norm();
            angular_norm = std::min(angular_norm, angular_max_velocity);
            angular_v = angular_norm * angular_v.normalized();

            double linear_norm = linear_v.norm();
            linear_norm = std::min(linear_norm, linear_max_velocity);
            linear_v = linear_norm * linear_v.normalized();

            // velocity
            std::pair<Eigen::Vector3d,Eigen::Vector3d> velocity(linear_v, angular_v);

            // // update error
            // angular_v = angular_v * dt;
            // rotation_error = rotation_error - inria::AxisToMatrix(angular_v);
            // position_error = position_error - linear_v * dt;

            // update current position and rotation on camera frame
            // estimated_rotation = desired_rotation.transpose() * rotation_error;
            // estimated_rotation = estimated_rotation.transpose();
            // estimated_position = - estimated_rotation * position_error + desired_position;

            estimated_rotation = inria::AxisToMatrix(angular_v * dt) * current_rotation;
            estimated_position = inria::AxisToMatrix(angular_v * dt) * current_position + linear_v * dt;

            return velocity;
        }

        // estimation of next orientation, useful for kalman's filter versions
        // to improve accuracy of next current orientation
        Eigen::Matrix3d get_estimated_rotation()
        {
            return estimated_rotation;
        }

        // estimation of next position, useful for kalman's filter version 
        // to improve accuracy of next current position
        Eigen::Vector3d get_estimated_position()
        {
            return estimated_position;
        }
};

#endif
