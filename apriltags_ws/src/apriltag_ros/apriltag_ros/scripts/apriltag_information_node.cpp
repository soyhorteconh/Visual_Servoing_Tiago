#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>


#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/FilterExponentialRotation.hpp>
#include <inria_plot/Plot.hpp>


cv::Mat image_depth;
apriltag_ros::AprilTagDetectionArray tags;

// intrinsic camera matrix values
float fx = 613.1328125;
float fy = 613.12939453125;
float cx = 636.8153686523438;
float cy = 365.55419921875;

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

// get point cloud information
std::vector<geometry_msgs::Point> get_point_cloud_information(apriltag_ros::AprilTagDetection tag)
{

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
    point4_coordinates.x = distance4 / fx * (tag.corner4.x - cx);
    point4_coordinates.y = distance4 / fy * (tag.corner4.y - cy);
    point4_coordinates.z = distance4;

    //center pixel to coordinates frame 
    geometry_msgs::Point center_coordinates;
    center_coordinates.x = distance_center / fx * (tag.centerpx.x - cx);
    center_coordinates.y = distance_center / fy * (tag.centerpx.y - cy);
    center_coordinates.z = distance_center;

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
    Eigen::Vector3d v3 = v2.cross(v1).normalized();

    // rotation matrix
    Eigen::Matrix3d rot;
    rot.col(0) = v2;
    rot.col(1) = v1;
    rot.col(2) = v3;

    // rotation matrix to quaternion
    Eigen::Quaterniond q(rot);
    q.normalize();
    //std::cout << "Rotation Matrix =" << std::endl << rot << std::endl;

    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    return orientation;
}

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

    ros::Publisher tag5_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag5_filter", 1 );

    // Subscribers
    ros::Subscriber sub = n.subscribe("tag_detections",1,apriltag_cb);
    ros::Subscriber depth_subscriber = n.subscribe("depth_to_rgb/image", 1, video_cb);

    // tag information
    apriltag_ros::AprilTagDetection tag5;
    apriltag_ros::AprilTagDetection tag6;
    apriltag_ros::AprilTagDetection tag7;
    apriltag_ros::AprilTagDetection tag8;
    apriltag_ros::AprilTagDetection tag10;
    apriltag_ros::AprilTagDetection tag20;

    //tag filter
    apriltag_ros::AprilTagDetection tag5_filter;

    //last positions
    geometry_msgs::Pose last_tag5_position;
    last_tag5_position.position.x = 0.0;
    last_tag5_position.position.y = 0.0;
    last_tag5_position.position.z = 0.0;
    last_tag5_position.orientation.x = 0.0;
    last_tag5_position.orientation.y = 0.0;
    last_tag5_position.orientation.z = 0.0;
    last_tag5_position.orientation.w = 0.0;

    //filters
    // tag 5 (filter for position)
    inria::FilterExponential<Eigen::Vector3d> filter_tag5_position;
    filter_tag5_position.cutoffFrequency() = 1;

    // tag 5 (filter for pose)
    inria::FilterExponentialRotation filter_tag5_orientation;
    filter_tag5_orientation.cutoffFrequency() = 1;

    //time
    double tag5_begin = ros::Time::now().toSec();

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        tag5.detected.data = false;
        tag6.detected.data = false;
        tag7.detected.data = false;
        tag8.detected.data = false;
        tag10.detected.data = false;
        tag20.detected.data = false;

        tag5_filter.detected.data = false;

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
                // time
                tag5_begin = ros::Time::now().toSec();

                //last center position and orientation
                last_tag5_position.position.x = tag5_filter.coordinate_center.x;
                last_tag5_position.position.y = tag5_filter.coordinate_center.y;
                last_tag5_position.position.z = tag5_filter.coordinate_center.z;
                last_tag5_position.orientation.x = tag5_filter.orientation.x;
                last_tag5_position.orientation.y = tag5_filter.orientation.y;
                last_tag5_position.orientation.z = tag5_filter.orientation.z;
                last_tag5_position.orientation.w = tag5_filter.orientation.w;

                // tag information
                tag5.id.push_back(tag.id[0]);
                tag5.size = tag.size;
                tag5.pose = tag.pose;
                tag5.corner1 = tag.corner1;
                tag5.corner2 = tag.corner2;
                tag5.corner3 = tag.corner3;
                tag5.corner4 = tag.corner4;
                tag5.centerpx = tag.centerpx;
                tag5.detected.data = true;

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

                // tag orientation
                geometry_msgs::Quaternion orientation;
                orientation = get_orientation(tag5);
                tag5.orientation = orientation; 

                // position filtering
                Eigen::Vector3d tag5_coordinates(tag5.coordinate_center.x, tag5.coordinate_center.y, tag5.coordinate_center.z);
                filter_tag5_position.update(tag5_coordinates, 0.0333);
                Eigen::Vector3d tag5_coordinates_filter = filter_tag5_position.value();
                tag5_filter.coordinate_center.x = tag5_coordinates_filter.x();
                tag5_filter.coordinate_center.y = tag5_coordinates_filter.y();
                tag5_filter.coordinate_center.z = tag5_coordinates_filter.z();

                // orientation filtering
                Eigen::Quaterniond tag5_q(tag5.orientation.w, tag5.orientation.x, tag5.orientation.y, tag5.orientation.z);
                filter_tag5_orientation.update(tag5_q, 0.0333);
                Eigen::Quaterniond tag5_q_filter = filter_tag5_orientation.valueQuaternion();
                tag5_filter.orientation.x = tag5_q_filter.x();
                tag5_filter.orientation.y = tag5_q_filter.y();
                tag5_filter.orientation.z = tag5_q_filter.z();
                tag5_filter.orientation.w = tag5_q_filter.w();

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

        // time 
        double tag5_now = ros::Time::now().toSec();
        double tag5_time = tag5_now - tag5_begin;

        //tag5 not detected
        if(tag5.detected.data == false){
            std::cout << "*No detected" << std::endl;
            
            if (tag5_time < 1.5){
                tag5.detected.data = true;
                tag5_filter.coordinate_center.x = last_tag5_position.position.x;
                tag5_filter.coordinate_center.y = last_tag5_position.position.y;
                tag5_filter.coordinate_center.z = last_tag5_position.position.z;
                tag5_filter.orientation.x = last_tag5_position.orientation.x;
                tag5_filter.orientation.y = last_tag5_position.orientation.y;
                tag5_filter.orientation.z = last_tag5_position.orientation.z;
                tag5_filter.orientation.w = last_tag5_position.orientation.w;

            } else if (tag5_time >= 1.5){
                tag5.coordinate_center.x = 0;
                tag5.coordinate_center.y = 0;
                tag5.coordinate_center.z = 0;
                tag5.orientation.x = 0;
                tag5.orientation.y = 0;
                tag5.orientation.z = 0;
                tag5.orientation.w = 0;

                filter_tag5_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag5_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag5_coordinates_filter = filter_tag5_position.value();
                tag5_filter.coordinate_center.x = tag5_coordinates_filter.x();
                tag5_filter.coordinate_center.y = tag5_coordinates_filter.y();
                tag5_filter.coordinate_center.z = tag5_coordinates_filter.z();

                Eigen::Quaterniond tag5_q_filter = filter_tag5_orientation.valueQuaternion();
                tag5_filter.orientation.x = tag5_q_filter.x();
                tag5_filter.orientation.y = tag5_q_filter.y();
                tag5_filter.orientation.z = tag5_q_filter.z();
                tag5_filter.orientation.w = tag5_q_filter.w();

                
            } 
            std::cout << "**time: " << tag5_time << std::endl;
        } 

        //signal filtering
        tag5_filter.detected.data = tag5.detected.data;
        
        // tags 
        tag5_publisher.publish(tag5);
        tag6_publisher.publish(tag6);
        tag7_publisher.publish(tag7);
        tag8_publisher.publish(tag8);
        tag10_publisher.publish(tag10);
        tag20_publisher.publish(tag20);

        // filter tag
        tag5_filter_publisher.publish(tag5_filter);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}