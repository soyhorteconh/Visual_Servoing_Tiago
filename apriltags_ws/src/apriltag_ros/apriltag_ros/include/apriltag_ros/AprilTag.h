#ifndef APRILTAG 
#define APRILTAG

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

class AprilTag
{
    private:
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        Eigen::Vector3d past_position;
        Eigen::Quaterniond past_orientation;

        Eigen::Vector3d position_filter;
        Eigen::Quaterniond orientation_filter;

        bool detected = false;

    public:
        AprilTag()
        {
            //initialized apriltag
        }

        void activate(Eigen::Vector3d px_corner1, Eigen::Vector3d px_corner2, Eigen::Vector3d px_corner3, Eigen::Vector3d px_corner4, Eigen::Vector3d px_center, cv::Mat depth_image)
        {
            past_position = position;
            past_orientation = orientation;

            //detected set to true
            is_detected();
            // get position
            position = estimate3DPositionFromPixel(px_center, depth_image);
            //get orientation
            orientation = estimateOrientationFromCorners(px_corner1, px_corner2, px_corner3, px_corner4, depth_image);
        }

        double get_depth(Eigen::Vector3d px_corner, cv::Mat depth_image)
        {
            return depth_image.at<float>(px_corner.y(), px_corner.x())/1000;
        }

        // get position
        Eigen::Vector3d estimate3DPositionFromPixel(Eigen::Vector3d px_corner, cv::Mat depth_image)
        {
            float fx = 613.1328125;
            float fy = 613.12939453125;
            float cx = 636.8153686523438;
            float cy = 365.55419921875;

            // partial point cloud reconstruction
            Eigen::Vector3d point_cloud;
            point_cloud.z() = get_depth(px_corner, depth_image);
            point_cloud.x() = point_cloud.z() / fx * (px_corner.x() - cx);
            point_cloud.y() = point_cloud.z() / fy * (px_corner.y() - cy);

            if( point_cloud.z() == 0){ not_detected(); }

            return point_cloud;
       }

        // get orientation
        Eigen::Quaterniond estimateOrientationFromCorners(Eigen::Vector3d px_corner1, Eigen::Vector3d px_corner2, Eigen::Vector3d px_corner3, Eigen::Vector3d px_corner4, cv::Mat depth_image)
        {
            //get position of each corner
            Eigen::Vector3d coordinate_corner1 = estimate3DPositionFromPixel(px_corner1, depth_image);
            Eigen::Vector3d coordinate_corner2 = estimate3DPositionFromPixel(px_corner2, depth_image);
            Eigen::Vector3d coordinate_corner3 = estimate3DPositionFromPixel(px_corner3, depth_image);
            Eigen::Vector3d coordinate_corner4 = estimate3DPositionFromPixel(px_corner4, depth_image);

            // cross product
            Eigen::Vector3d v1 = (coordinate_corner2 - coordinate_corner1).normalized();
            Eigen::Vector3d v2 = (coordinate_corner4 - coordinate_corner1).normalized();
            Eigen::Vector3d vr = v1.cross(v2).normalized();

            // rotation matrix
            Eigen::Matrix3d rot;
            rot.col(0) = v1; //x
            rot.col(1) = v2; //y
            rot.col(2) = vr; //z

            // rotation matrix to quaternion
            Eigen::Quaterniond q(rot);
            q.normalize();

            return q;
        }

        Eigen::Vector3d getPosition()
        {
            return position;
        }

        Eigen::Quaterniond getOrientation()
        {
            return orientation;
        }

        Eigen::Vector3d getPostionFilter()
        {
            return position_filter;
        }

        Eigen::Quaterniond getOrientationFilter()
        {
            return orientation_filter;
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

        void setPostion(Eigen::Vector3d update_position)
        {
            position = update_position;
        }

        void setOrientation(Eigen::Quaterniond update_orientation)
        {
            orientation = update_orientation;
        }

        void setPostionFilter(Eigen::Vector3d position_filter_value)
        {
            position_filter = position_filter_value;
        }

        void setOrientationFilter(Eigen::Quaterniond orientation_filter_value)
        {
            orientation_filter = orientation_filter_value;
        }

};

#endif
