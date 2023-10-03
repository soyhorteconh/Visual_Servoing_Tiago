#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Geometry>
#include <iostream>

cv::Mat image_depth;
apriltag_ros::AprilTagDetectionArray tags;
// sensor_msgs::PointCloud2 cloud_msg;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// callbacks
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    tags.detections = msg -> detections;
}

void video_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    // sensor_msg to an opencv image
    cv_bridge::CvImagePtr depth;
    try{
        depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    image_depth = depth -> image;
    cv::GaussianBlur(image_depth, image_depth, cv::Size(5, 5), 0);
}

// void cloud_points_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     cloud_msg = *msg;
//     pcl::fromROSMsg(*msg, *cloud);
// }

// get point cloud information
std::vector<geometry_msgs::Point> get_point_cloud_information(apriltag_ros::AprilTagDetection tag)
{
    float fx = 613.1328125;
    float fy = 613.12939453125;
    float cx = 636.8153686523438;
    float cy = 365.55419921875;

    // get depth information
    float distance1 = image_depth.at<float>(tag.corner1.y, tag.corner1.x)/1000;
    float distance2 = image_depth.at<float>(tag.corner2.y, tag.corner2.x)/1000;
    float distance3 = image_depth.at<float>(tag.corner3.y, tag.corner3.x)/1000;
    float distance4 = image_depth.at<float>(tag.corner4.y, tag.corner4.x)/1000;
    float distance_center = image_depth.at<float>(tag.centerpx.y, tag.centerpx.x)/1000;

    // pixel1 to coordinates frame camera
    geometry_msgs::Point point1_coordinates;
    point1_coordinates.x = distance1 / fx * (tag.corner1.x - cx);
    point1_coordinates.y = distance1 / fy * (tag.corner1.y - cy);
    point1_coordinates.z = distance1;

    // pixel2 to coordinates frame camera
    geometry_msgs::Point point2_coordinates;
    point2_coordinates.x = distance2 / fx * (tag.corner2.x - cx);
    point2_coordinates.y = distance2 / fy * (tag.corner2.y - cy);
    point2_coordinates.z = distance2;
    
    //pixel3 to coordinates frame camera
    geometry_msgs::Point point3_coordinates;
    point3_coordinates.x = distance3 / fx * (tag.corner3.x - cx);
    point3_coordinates.y = distance3 / fy * (tag.corner3.y - cy);
    point3_coordinates.z = distance3;

    //pixel4 to coordinates frame 
    geometry_msgs::Point point4_coordinates;
    point4_coordinates.x= distance4 / fx * (tag.corner4.x - cx);
    point4_coordinates.y = distance4 / fy * (tag.corner4.y - cy);
    point4_coordinates.z = distance4;

    //center pixel to coordinates frame 
    geometry_msgs::Point center_coordinates;
    center_coordinates.x = distance_center / fx * (tag.centerpx.x - cx);
    center_coordinates.y = distance_center / fy * (tag.centerpx.y - cy);
    center_coordinates.z = distance_center;

    // ROS_INFO("Point cloud");
    // ROS_INFO("X: %f", point1_coordinates.x);
    // ROS_INFO("Y: %f", point1_coordinates.y);
    // ROS_INFO("Z: %f", point1_coordinates.z);

    //coordinates vector
    std::vector<geometry_msgs::Point> coordinates;
    coordinates.push_back(point1_coordinates);
    coordinates.push_back(point2_coordinates);
    coordinates.push_back(point3_coordinates);
    coordinates.push_back(point4_coordinates);
    coordinates.push_back(center_coordinates);

    
    return coordinates;
}

// get orientation
geometry_msgs::Quaternion get_orientation(apriltag_ros::AprilTagDetection tag)
{
    Eigen::Vector3d o_1(tag.coordinate1.x, tag.coordinate1.y, tag.coordinate1.z);
    Eigen::Vector3d o_2(tag.coordinate2.x, tag.coordinate2.y, tag.coordinate2.z);
    Eigen::Vector3d o_4(tag.coordinate4.x, tag.coordinate4.y, tag.coordinate4.z);

    Eigen::Vector3d v1 = (o_4 - o_1).normalized();
    Eigen::Vector3d v2 = (o_2 - o_1).normalized();

    // cross product
    Eigen::Vector3d v3 = v1.cross(v2).normalized();

    // rotation matrix
    Eigen::Matrix3d rot;
    rot.col(0) = v1;
    rot.col(1) = v2;
    rot.col(2) = v3;

    // rotation matrix to quaternion
    Eigen::Quaterniond q(rot);
    //std::cout << "Rotation Matrix =" << std::endl << rot << std::endl;

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    return orientation;
}

// // get depth information
// std::vector<float> get_depth(apriltag_ros::AprilTagDetection tag)
// {
//     std::vector<float> distances;
//     float distance1 = image_depth.at<float>(tag.corner1.y, tag.corner1.x)/1000;
//     float distance2 = image_depth.at<float>(tag.corner2.y, tag.corner2.x)/1000;
//     float distance3 = image_depth.at<float>(tag.corner3.y, tag.corner3.x)/1000;
//     float distance4 = image_depth.at<float>(tag.corner4.y, tag.corner4.x)/1000;
//     float distance_center = image_depth.at<float>(tag.centerpx.y, tag.centerpx.x)/1000;
//     distances.push_back(distance1);
//     distances.push_back(distance2);
//     distances.push_back(distance3);
//     distances.push_back(distance4);
//     distances.push_back(distance_center);
//     return distances;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_information");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher tag5_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag5", 1 );
    ros::Publisher tag6_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag6", 1 );
    ros::Publisher tag7_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag7", 1 );
    ros::Publisher tag8_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag8", 1 );
    ros::Publisher tag10_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag10", 1 );
    ros::Publisher tag20_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag20", 1 );

    // Subscribers
    ros::Subscriber sub = n.subscribe("tag_detections",1,apriltag_cb);
    ros::Subscriber depth_subscriber = n.subscribe("depth_to_rgb/image", 1, video_cb);
    //ros::Subscriber cloud_points_subscriber = n.subscribe("/depth_to_rgb/points", 1, cloud_points_cb);

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        // tag information
        apriltag_ros::AprilTagDetection tag5;
        apriltag_ros::AprilTagDetection tag6;
        apriltag_ros::AprilTagDetection tag7;
        apriltag_ros::AprilTagDetection tag8;
        apriltag_ros::AprilTagDetection tag10;
        apriltag_ros::AprilTagDetection tag20;

        tag5.detected.data = false;
        tag6.detected.data = false;
        tag7.detected.data = false;
        tag8.detected.data = false;
        tag10.detected.data = false;
        tag20.detected.data = false;

        for (int i = 0; i < tags.detections.size(); i++)
        {
            apriltag_ros::AprilTagDetection tag = tags.detections.at(i);

            //tag10
            if(tag.id[0] == 10 )
            {
                tag10.id.push_back(tag.id[0]);
                tag10.size = tag.size;
                tag10.pose = tag.pose;
                tag10.corner1 = tag.corner1;
                tag10.corner2 = tag.corner2;
                tag10.corner3 = tag.corner3;
                tag10.corner4 = tag.corner4;
                tag10.centerpx = tag.centerpx;
                tag10.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag10);
                // tag10.corner1.z = distances[0];
                // tag10.corner2.z = distances[1];
                // tag10.corner3.z = distances[2];
                // tag10.corner4.z = distances[3];
                // tag10.centerpx.z = distances[4];

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag10);
                if(coordinates.size() == 5)
                {
                    tag10.coordinate1 = coordinates.at(0);
                    tag10.coordinate2 = coordinates.at(1);
                    tag10.coordinate3 = coordinates.at(2);
                    tag10.coordinate4 = coordinates.at(3);
                    tag10.coordinate_center = coordinates.at(4);
                }

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag10);
                tag10.orientation = orientation; 
            }
            else if (tag.id[0] == 20 )
            {
                tag20.id.push_back(tag.id[0]);
                tag20.size = tag.size;
                tag20.pose = tag.pose;
                tag20.corner1 = tag.corner1;
                tag20.corner2 = tag.corner2;
                tag20.corner3 = tag.corner3;
                tag20.corner4 = tag.corner4;
                tag20.centerpx = tag.centerpx;
                tag20.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag20);
                // tag20.corner1.z = distances[0];
                // tag20.corner2.z = distances[1];
                // tag20.corner3.z = distances[3];
                // tag20.corner4.z = distances[4];
                // tag20.centerpx.z = distances[4];

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag20);
                if(coordinates.size() == 5)
                {
                    tag20.coordinate1 = coordinates.at(0);
                    tag20.coordinate2 = coordinates.at(1);
                    tag20.coordinate3 = coordinates.at(2);
                    tag20.coordinate4 = coordinates.at(3);
                    tag20.coordinate_center = coordinates.at(4);
                }

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag20);
                tag20.orientation = orientation; 
            }
            else if (tag.id[0] == 5 )
            {
                tag5.id.push_back(tag.id[0]);
                tag5.size = tag.size;
                tag5.pose = tag.pose;
                tag5.corner1 = tag.corner1;
                tag5.corner2 = tag.corner2;
                tag5.corner3 = tag.corner3;
                tag5.corner4 = tag.corner4;
                tag5.centerpx = tag.centerpx;
                tag5.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag5);
                // tag5.corner1.z = distances[0];
                // tag5.corner2.z = distances[1];
                // tag5.corner3.z = distances[2];
                // tag5.corner4.z = distances[3];
                // tag5.centerpx.z = distances[4];

                //float x = (tag5.corner1.x - (tag5.corner1.z * cx)) / fx;
                //float y = (tag5.corner1.y - (tag5.corner1.z * cy)) / fy;
                // ROS_INFO("X and Y calculated");
                // ROS_INFO("x: %f", x);
                // ROS_INFO("y: %f", y);

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag5);
                if(coordinates.size() == 5)
                {
                    tag5.coordinate1 = coordinates.at(0);
                    tag5.coordinate2 = coordinates.at(1);
                    tag5.coordinate3 = coordinates.at(2);
                    tag5.coordinate4 = coordinates.at(3);
                    tag5.coordinate_center = coordinates.at(4);
                }
                // float u = fx * (tag5.coordinate1.x / tag5.coordinate1.z) + cx;
                // float v = fy * (tag5.coordinate1.y / tag5.coordinate1.z) + cy;
                // ROS_INFO("u and v pixels");
                // ROS_INFO("u: %f", u);
                // ROS_INFO("v: %f", v);

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag5);
                tag5.orientation = orientation; 
            }
            else if (tag.id[0] == 6 )
            {
                tag6.id.push_back(tag.id[0]);
                tag6.size = tag.size;
                tag6.pose = tag.pose;
                tag6.corner1 = tag.corner1;
                tag6.corner2 = tag.corner2;
                tag6.corner3 = tag.corner3;
                tag6.corner4 = tag.corner4;
                tag6.centerpx = tag.centerpx;
                tag6.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag6);
                // tag6.corner1.z = distances[0];
                // tag6.corner2.z = distances[1];
                // tag6.corner3.z = distances[2];
                // tag6.corner4.z = distances[3];
                // tag6.centerpx.z = distances[4];

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag6);
                if(coordinates.size() == 5)
                {
                    tag6.coordinate1 = coordinates.at(0);
                    tag6.coordinate2 = coordinates.at(1);
                    tag6.coordinate3 = coordinates.at(2);
                    tag6.coordinate4 = coordinates.at(3);
                    tag6.coordinate_center = coordinates.at(4);
                }

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag6);
                tag6.orientation = orientation; 
            }
            else if (tag.id[0] == 7 )
            {
                tag7.id.push_back(tag.id[0]);
                tag7.size = tag.size;
                tag7.pose = tag.pose;
                tag7.corner1 = tag.corner1;
                tag7.corner2 = tag.corner2;
                tag7.corner3 = tag.corner3;
                tag7.corner4 = tag.corner4;
                tag7.centerpx = tag.centerpx;
                tag7.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag7);
                // tag7.corner1.z = distances[0];
                // tag7.corner2.z = distances[1];
                // tag7.corner3.z = distances[2];
                // tag7.corner4.z = distances[3];
                // tag7.centerpx.z = distances[4];

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag7);
                if(coordinates.size() == 5)
                {
                    tag7.coordinate1 = coordinates.at(0);
                    tag7.coordinate2 = coordinates.at(1);
                    tag7.coordinate3 = coordinates.at(2);
                    tag7.coordinate4 = coordinates.at(3);
                    tag7.coordinate_center = coordinates.at(4);
                }

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag7);
                tag7.orientation = orientation; 
            }
            else if (tag.id[0] == 8 )
            {
                tag8.id.push_back(tag.id[0]);
                tag8.size = tag.size;
                tag8.pose = tag.pose;
                tag8.corner1 = tag.corner1;
                tag8.corner2 = tag.corner2;
                tag8.corner3 = tag.corner3;
                tag8.corner4 = tag.corner4;
                tag8.centerpx = tag.centerpx;
                tag8.detected.data = true;

                // // getting depth information
                // std::vector<float> distances;
                // distances = get_depth(tag8);
                // tag8.corner1.z = distances[0];
                // tag8.corner2.z = distances[1];
                // tag8.corner3.z = distances[2];
                // tag8.corner4.z = distances[3];
                // tag8.centerpx.z = distances[4];

                //pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag8);
                if(coordinates.size() == 5)
                {
                    tag8.coordinate1 = coordinates.at(0);
                    tag8.coordinate2 = coordinates.at(1);
                    tag8.coordinate3 = coordinates.at(2);
                    tag8.coordinate4 = coordinates.at(3);
                    tag8.coordinate_center = coordinates.at(4);
                }

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag8);
                tag8.orientation = orientation;
            }
        }
        
        tag5_publisher.publish(tag5);
        tag6_publisher.publish(tag6);
        tag7_publisher.publish(tag7);
        tag8_publisher.publish(tag8);
        tag10_publisher.publish(tag10);
        tag20_publisher.publish(tag20);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}