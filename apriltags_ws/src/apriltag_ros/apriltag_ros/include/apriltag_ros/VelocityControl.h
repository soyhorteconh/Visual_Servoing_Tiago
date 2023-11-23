#ifndef VELOCITY_CONTROL
#define VELOCITY_CONTROL

#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <inria_maths/AxisAngle.h>
#include <iostream>
#include <utility>

class VelocityControl
{
    private:
        double dt;
        double linear_kp;
        double angular_kp;
        double linear_max_velocity;
        double angular_max_velocity;
        Eigen::Matrix3d estimated_rotation;
        Eigen::Vector3d estimated_position;

        Eigen::Matrix3d rotation_error;
        Eigen::Vector3d position_error;

    public:
        VelocityControl(double dt, double linear_kp, double angular_kp, double linear_max_velocity, double angular_max_velocity) :
            dt(dt),
            linear_kp(linear_kp), 
            angular_kp(angular_kp), 
            linear_max_velocity(linear_max_velocity),
            angular_max_velocity(angular_max_velocity)
        {}

        // saturated velocity controller
        std::pair<Eigen::Vector3d,Eigen::Vector3d> controller(
            const Eigen::Vector3d desired_position, const Eigen::Matrix3d desired_rotation_r,
            const Eigen::Vector3d current_position, const Eigen::Matrix3d current_rotation_r)
        {
            // normalized quaternions
            Eigen::Quaterniond current_rotation_q(current_rotation_r);
            Eigen::Matrix3d current_rotation = current_rotation_q.normalized().toRotationMatrix();

            Eigen::Quaterniond desired_rotation_q(desired_rotation_r);
            Eigen::Matrix3d desired_rotation = desired_rotation_q.normalized().toRotationMatrix();

            // error
            rotation_error = current_rotation.transpose() * desired_rotation;
            position_error = current_rotation.transpose() * (desired_position - current_position);
            // std::cout << "Error position" << std::endl;
            // std::cout << "x: " << position_error.x() << std::endl;
            // std::cout << "y: " << position_error.y() << std::endl;
            // std::cout << "z: " << position_error.z() << std::endl;

            // velocity
            Eigen::Vector3d angular_v = angular_kp * (inria::MatrixToAxis(rotation_error));
            Eigen::Vector3d linear_v = linear_kp * position_error;

            // saturated control
            double angular_norm = angular_v.norm();
            angular_norm = std::min(angular_norm, angular_max_velocity);
            angular_v = angular_norm * angular_v.normalized();

            double linear_norm = linear_v.norm();
            linear_norm = std::min(linear_norm, linear_max_velocity);
            linear_v = linear_norm * linear_v.normalized();

            // Eigen::Vector3d linear_v(0.0, 0.01 ,0.0);
            //Eigen::Vector3d angular_v(0.0, 0.0, 0.0);

            // final velocity to retun
            std::pair<Eigen::Vector3d,Eigen::Vector3d> velocity(linear_v, angular_v);

            // pose integration 
            // to predict next pose 
            estimated_rotation = current_rotation * inria::AxisToMatrix(angular_v * dt);
            estimated_position = current_rotation * (linear_v * dt) + current_position;

            return velocity;
        }

        // predicted rotation gets its value from a 
        // kalman filter computarization
        Eigen::Quaterniond get_predicted_rotation(const Eigen::Matrix3d measured_rotation, double freq)
        {
            double alpha =  get_alpha_from_freq(freq);

            Eigen::Quaterniond measured_rotation_q(measured_rotation);
            Eigen::Quaterniond estimated_rotation_q(estimated_rotation);
            Eigen::Quaterniond predicted_rotation_q = estimated_rotation_q.normalized().slerp(1 - alpha, measured_rotation_q.normalized());

            return predicted_rotation_q;
        }

        // predicted position gets its value from a 
        // kalman filter computarization
        Eigen::Vector3d get_predicted_position(const Eigen::Vector3d measured_position, double freq)
        {
            double alpha =  get_alpha_from_freq(freq);
            Eigen::Vector3d predicted_position = alpha * estimated_position + (1 -  alpha) * measured_position;
            
            return predicted_position;
        }

        // return estimated rotation value as rotation matrix
        Eigen::Matrix3d get_estimated_rotation()
        {
            return estimated_rotation;
        }

        // return estimated position value as vector
        Eigen::Vector3d get_estimated_position()
        {
            return estimated_position;
        }

        // return rotation error
        Eigen::Matrix3d get_rotation_error()
        {
            return rotation_error;
        }

        // return position error
        Eigen::Vector3d get_position_error()
        {
            return position_error;
        }

        // get alpha value for kalman filter
        double get_alpha_from_freq(double freq)
        {
            double omega = 2.0 * M_PI * freq;
            double alpha = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);

            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }

            return alpha;
        }

};

#endif
