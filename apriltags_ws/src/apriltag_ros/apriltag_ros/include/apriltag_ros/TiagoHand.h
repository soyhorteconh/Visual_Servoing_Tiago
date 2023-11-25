/*
Hortencia Alejandra Ramirez Vazquez
25th november, 2023
*/

#ifndef TIAGOHAND 
#define TIAGOHAND

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

class TiagoHand
{
    private:
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        bool detected = false;

    public:
        TiagoHand()
        {
            //initialized apriltag
        }

        Eigen::Vector3d getPosition()
        {
            return position;
        }

        Eigen::Quaterniond getOrientation()
        {
            return orientation;
        }

        void setPostion(Eigen::Vector3d update_position)
        {
            position = update_position;
        }

        void setOrientation(Eigen::Quaterniond update_orientation)
        {
            orientation = update_orientation;
        }

        bool getDetected()
        {
            return detected;
        }

        void not_detected()
        {
            detected = false;
        }

        void is_detected()
        {
            detected = true;
        }

};

#endif