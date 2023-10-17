#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseStamped.h"
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

// inria libraries
#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/FilterExponentialRotation.hpp>
#include <inria_plot/Plot.hpp>

//global variables
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
    //cv::imshow("Image Depth", image_depth);
    //cv::GaussianBlur(image_depth, image_depth, cv::Size(5, 5), 0);

    //cv::imshow("Gaussian Blur", image_depth);

    // Esperar un poco y procesar eventos de GUI (teclas)
    //cv::waitKey(10); // Espera 10 milisegundos y comprueba si se presionó una tecla

    // Salir del bucle si se presiona la tecla 'q' o si se cierra la ventana de visualización
    // if (key == 'q' || key == 27) { // 'q' o tecla Esc
    //     break;
    // }
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

    geometry_msgs::Point point1_coordinates;
    geometry_msgs::Point point2_coordinates;
    geometry_msgs::Point point3_coordinates;
    geometry_msgs::Point point4_coordinates;
    geometry_msgs::Point center_coordinates;

    // pixel1 to coordinates frame camera
    point1_coordinates.x = distance1 / fx * (tag.corner1.x - cx);
    point1_coordinates.y = distance1 / fy * (tag.corner1.y - cy);
    point1_coordinates.z = distance1;

    // pixel2 to coordinates frame camera
    point2_coordinates.x = distance2 / fx * (tag.corner2.x - cx);
    point2_coordinates.y = distance2 / fy * (tag.corner2.y - cy);
    point2_coordinates.z = distance2;
    
    //pixel3 to coordinates frame camera
    point3_coordinates.x = distance3 / fx * (tag.corner3.x - cx);
    point3_coordinates.y = distance3 / fy * (tag.corner3.y - cy);
    point3_coordinates.z = distance3;

    //pixel4 to coordinates frame 
    point4_coordinates.x = distance4 / fx * (tag.corner4.x - cx);
    point4_coordinates.y = distance4 / fy * (tag.corner4.y - cy);
    point4_coordinates.z = distance4;

    //center pixel to coordinates frame 
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
    Eigen::Vector3d o_3(tag.coordinate3.x, tag.coordinate3.y, tag.coordinate3.z);
    Eigen::Vector3d o_4(tag.coordinate4.x, tag.coordinate4.y, tag.coordinate4.z);

    Eigen::Vector3d v1 = (o_2 - o_1).normalized();
    Eigen::Vector3d v2 = (o_4 - o_1).normalized();

    Eigen:: Vector3d vr = v1.cross(v2).normalized();

    // rotation matrix
    Eigen::Matrix3d rot1;
    rot1.col(0) = v1;
    rot1.col(1) = v2;
    rot1.col(2) = vr;

    // rotation matrix to quaternion
    Eigen::Quaterniond q1(rot1);
    q1.normalize();

    geometry_msgs::Quaternion orientation;
    orientation.x = q1.x();
    orientation.y = q1.y();
    orientation.z = q1.z();
    orientation.w = q1.w();

    return orientation;
}

Eigen::Quaterniond multiply_rotation_matrix(Eigen::Quaterniond q, float radians, int option)
{
    // option = 1 -> x rotation
    // option = 2 -> y rotation
    // option = 3 -> z rotation

    Eigen::Matrix3d orientation = q.normalized().toRotationMatrix();
    Eigen::Matrix3d rotation;
    if (option == 1){
        std::cout << "X rotation" << std::endl;
        std::cout << "Rotation: " << radians << std::endl;
        // x rotation
        rotation << 1, 0, 0,
                    0, std::cos(radians), -std::sin(radians),
                    0, std::sin(radians), std::cos(radians);
    } else if (option == 2){
        // y rotation
        std::cout << "Y rotation" << std::endl;
        std::cout << "Rotation: " << radians << std::endl;
        std::cout << std::cos(radians) << std::endl;
        rotation << std::cos(radians), 0, std::sin(radians),
                    0, 1, 0,
                    -1*std::sin(radians), 0, std::cos(radians);
    } else if (option == 3){
        // z rotation
        std::cout << "z rotation" << std::endl;
        std::cout << "Rotation: " << radians << std::endl;
        rotation << std::cos(radians), -std::sin(radians), 0,
                    std::sin(radians), std::cos(radians), 0,
                    0, 0, 1;
    }

    std::cout << "Matrix: " << std::endl << rotation << std::endl;
    Eigen::Matrix3d m_r = orientation*rotation;
    Eigen::Quaterniond q_r(m_r);
    return q_r;
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
    ros::Publisher tag6_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag6_filter", 1 );
    ros::Publisher tag7_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag7_filter", 1 );
    ros::Publisher tag8_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag8_filter", 1 );
    ros::Publisher tag10_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag10_filter", 1 );
    ros::Publisher tag20_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag20_filter", 1 );

    ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/pose", 1);

    // Subscribers
    ros::Subscriber sub = n.subscribe("tag_detections",1,apriltag_cb);
    ros::Subscriber depth_subscriber = n.subscribe("depth_to_rgb/image", 1, video_cb);

    // robot hand information
    geometry_msgs::PoseStamped robot_hand;

    // tag information
    apriltag_ros::AprilTagDetection tag5;
    apriltag_ros::AprilTagDetection tag6;
    apriltag_ros::AprilTagDetection tag7;
    apriltag_ros::AprilTagDetection tag8;
    apriltag_ros::AprilTagDetection tag10;
    apriltag_ros::AprilTagDetection tag20;

    //tag filter
    apriltag_ros::AprilTagDetection tag5_filter;
    apriltag_ros::AprilTagDetection tag6_filter;
    apriltag_ros::AprilTagDetection tag7_filter;
    apriltag_ros::AprilTagDetection tag8_filter;
    apriltag_ros::AprilTagDetection tag10_filter;
    apriltag_ros::AprilTagDetection tag20_filter;

    //last positions
    geometry_msgs::Pose last_tag5_position;
    geometry_msgs::Pose last_tag6_position;
    geometry_msgs::Pose last_tag7_position;
    geometry_msgs::Pose last_tag8_position;
    geometry_msgs::Pose last_tag10_position;
    geometry_msgs::Pose last_tag20_position;

    //filters
    // tag5
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag5_position;
    filter_tag5_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag5_orientation;
    filter_tag5_orientation.cutoffFrequency() = 1;

    // tag6
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag6_position;
    filter_tag6_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag6_orientation;
    filter_tag6_orientation.cutoffFrequency() = 1;

    // tag7
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag7_position;
    filter_tag7_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag7_orientation;
    filter_tag7_orientation.cutoffFrequency() = 1;

    // tag8
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag8_position;
    filter_tag8_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag8_orientation;
    filter_tag8_orientation.cutoffFrequency() = 1;

    // tag10
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag10_position;
    filter_tag10_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag10_orientation;
    filter_tag10_orientation.cutoffFrequency() = 1;

    // tag20
    // filter for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag20_position;
    filter_tag20_position.cutoffFrequency() = 1;
    // filter for pose
    inria::FilterExponentialRotation filter_tag20_orientation;
    filter_tag20_orientation.cutoffFrequency() = 1;

    //time
    double tag5_begin = ros::Time::now().toSec();
    double tag6_begin = ros::Time::now().toSec();
    double tag7_begin = ros::Time::now().toSec();
    double tag8_begin = ros::Time::now().toSec();
    double tag10_begin = ros::Time::now().toSec();
    double tag20_begin = ros::Time::now().toSec();

    //counter
    int count_tag5 = 0;
    int count_tag6 = 0;
    int count_tag7 = 0;
    int count_tag8 = 0;

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
        tag6_filter.detected.data = false;
        tag7_filter.detected.data = false;
        tag8_filter.detected.data = false;
        tag10_filter.detected.data = false;
        tag20_filter.detected.data = false;

        // tags detected
        std::vector<int> robot_hand_estimation_tags;

        for (int i = 0; i < tags.detections.size(); i++)
        {
            apriltag_ros::AprilTagDetection tag = tags.detections.at(i);

            //tag10
            if(tag.id[0] == 10 )
            {

                //last center position and orientation
                last_tag10_position.position.x = tag10_filter.coordinate_center.x;
                last_tag10_position.position.y = tag10_filter.coordinate_center.y;
                last_tag10_position.position.z = tag10_filter.coordinate_center.z;
                last_tag10_position.orientation.x = tag10_filter.orientation.x;
                last_tag10_position.orientation.y = tag10_filter.orientation.y;
                last_tag10_position.orientation.z = tag10_filter.orientation.z;
                last_tag10_position.orientation.w = tag10_filter.orientation.w;

                // tag information
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

                    bool detected = (coordinates.at(0).z == 0 || coordinates.at(1).z == 0 || coordinates.at(2).z == 0 || coordinates.at(3).z == 0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag10_begin = ros::Time::now().toSec();
                        
                        tag10.coordinate1 = coordinates.at(0);
                        tag10.coordinate2 = coordinates.at(1);
                        tag10.coordinate3 = coordinates.at(2);
                        tag10.coordinate4 = coordinates.at(3);
                        tag10.coordinate_center = coordinates.at(4);

                        // tag orientation
                        geometry_msgs::Quaternion orientation;
                        orientation = get_orientation(tag10);
                        tag10.orientation = orientation; 

                        // position filtering
                        Eigen::Vector3d tag10_coordinates(tag10.coordinate_center.x, tag10.coordinate_center.y, tag10.coordinate_center.z);
                        filter_tag10_position.update(tag10_coordinates, 0.0333);
                        Eigen::Vector3d tag10_coordinates_filter = filter_tag10_position.value();
                        tag10_filter.coordinate_center.x = tag10_coordinates_filter.x();
                        tag10_filter.coordinate_center.y = tag10_coordinates_filter.y();
                        tag10_filter.coordinate_center.z = tag10_coordinates_filter.z();

                        // orientation filtering
                        Eigen::Quaterniond tag10_q(tag10.orientation.w, tag10.orientation.x, tag10.orientation.y, tag10.orientation.z);
                        filter_tag10_orientation.update(tag10_q, 0.0333);
                        Eigen::Quaterniond tag10_q_filter = filter_tag10_orientation.valueQuaternion();
                        tag10_filter.orientation.x = tag10_q_filter.x();
                        tag10_filter.orientation.y = tag10_q_filter.y();
                        tag10_filter.orientation.z = tag10_q_filter.z();
                        tag10_filter.orientation.w = tag10_q_filter.w();
                    }
                    else
                    {
                        tag10.detected.data = false;
                    }
                }
            }
            else if (tag.id[0] == 20 )
            {

                //last center position and orientation
                last_tag20_position.position.x = tag20_filter.coordinate_center.x;
                last_tag20_position.position.y = tag20_filter.coordinate_center.y;
                last_tag20_position.position.z = tag20_filter.coordinate_center.z;
                last_tag20_position.orientation.x = tag20_filter.orientation.x;
                last_tag20_position.orientation.y = tag20_filter.orientation.y;
                last_tag20_position.orientation.z = tag20_filter.orientation.z;
                last_tag20_position.orientation.w = tag20_filter.orientation.w;

                // tag information
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

                    bool detected = (coordinates.at(0).z == 0 || coordinates.at(1).z == 0 || coordinates.at(2).z == 0 || coordinates.at(3).z == 0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag20_begin = ros::Time::now().toSec();
                        
                        tag20.coordinate1 = coordinates.at(0);
                        tag20.coordinate2 = coordinates.at(1);
                        tag20.coordinate3 = coordinates.at(2);
                        tag20.coordinate4 = coordinates.at(3);
                        tag20.coordinate_center = coordinates.at(4);

                        // tag orientation
                        geometry_msgs::Quaternion orientation;
                        orientation = get_orientation(tag20);
                        tag20.orientation = orientation; 

                        // position filtering
                        Eigen::Vector3d tag20_coordinates(tag20.coordinate_center.x, tag20.coordinate_center.y, tag20.coordinate_center.z);
                        filter_tag20_position.update(tag20_coordinates, 0.0333);
                        Eigen::Vector3d tag20_coordinates_filter = filter_tag20_position.value();
                        tag20_filter.coordinate_center.x = tag20_coordinates_filter.x();
                        tag20_filter.coordinate_center.y = tag20_coordinates_filter.y();
                        tag20_filter.coordinate_center.z = tag20_coordinates_filter.z();

                        // orientation filtering
                        Eigen::Quaterniond tag20_q(tag20.orientation.w, tag20.orientation.x, tag20.orientation.y, tag20.orientation.z);
                        filter_tag20_orientation.update(tag20_q, 0.0333);
                        Eigen::Quaterniond tag20_q_filter = filter_tag20_orientation.valueQuaternion();
                        tag20_filter.orientation.x = tag20_q_filter.x();
                        tag20_filter.orientation.y = tag20_q_filter.y();
                        tag20_filter.orientation.z = tag20_q_filter.z();
                        tag20_filter.orientation.w = tag20_q_filter.w();
                    }
                    else
                    {
                        tag20.detected.data = false;
                    }
                }

            }
            else if (tag.id[0] == 5 )
            {
                count_tag5 += 1;

                // tag detected counter
                robot_hand_estimation_tags.push_back(5);

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

                    bool detected = (coordinates.at(0).z == 0.0 || coordinates.at(1).z == 0.0 || coordinates.at(2).z == 0.0 || coordinates.at(3).z == 0.0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag5_begin = ros::Time::now().toSec();
                        
                        tag5.coordinate1 = coordinates.at(0);
                        tag5.coordinate2 = coordinates.at(1);
                        tag5.coordinate3 = coordinates.at(2);
                        tag5.coordinate4 = coordinates.at(3);
                        tag5.coordinate_center = coordinates.at(4);

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
                    else
                    {
                        tag5.detected.data = false;
                    }
                }
            }
            else if (tag.id[0] == 6 )
            {
                count_tag6 += 1;

                // tag detected counter
                robot_hand_estimation_tags.push_back(6);

                //last center position and orientation
                last_tag6_position.position.x = tag6_filter.coordinate_center.x;
                last_tag6_position.position.y = tag6_filter.coordinate_center.y;
                last_tag6_position.position.z = tag6_filter.coordinate_center.z;
                last_tag6_position.orientation.x = tag6_filter.orientation.x;
                last_tag6_position.orientation.y = tag6_filter.orientation.y;
                last_tag6_position.orientation.z = tag6_filter.orientation.z;
                last_tag6_position.orientation.w = tag6_filter.orientation.w;

                // tag information
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

                    bool detected = (coordinates.at(0).z == 0.0 || coordinates.at(1).z == 0.0 || coordinates.at(2).z == 0.0 || coordinates.at(3).z == 0.0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag6_begin = ros::Time::now().toSec();
                    
                        tag6.coordinate1 = coordinates.at(0);
                        tag6.coordinate2 = coordinates.at(1);
                        tag6.coordinate3 = coordinates.at(2);
                        tag6.coordinate4 = coordinates.at(3);
                        tag6.coordinate_center = coordinates.at(4);

                        // tag orientation
                        geometry_msgs::Quaternion orientation;
                        orientation = get_orientation(tag6);
                        tag6.orientation = orientation; 

                        // position filtering
                        Eigen::Vector3d tag6_coordinates(tag6.coordinate_center.x, tag6.coordinate_center.y, tag6.coordinate_center.z);
                        filter_tag6_position.update(tag6_coordinates, 0.0333);
                        Eigen::Vector3d tag6_coordinates_filter = filter_tag6_position.value();
                        tag6_filter.coordinate_center.x = tag6_coordinates_filter.x();
                        tag6_filter.coordinate_center.y = tag6_coordinates_filter.y();
                        tag6_filter.coordinate_center.z = tag6_coordinates_filter.z();

                        // orientation filtering
                        Eigen::Quaterniond tag6_q(tag6.orientation.w, tag6.orientation.x, tag6.orientation.y, tag6.orientation.z);
                        filter_tag6_orientation.update(tag6_q, 0.0333);
                        Eigen::Quaterniond tag6_q_filter = filter_tag6_orientation.valueQuaternion();
                        tag6_filter.orientation.x = tag6_q_filter.x();
                        tag6_filter.orientation.y = tag6_q_filter.y();
                        tag6_filter.orientation.z = tag6_q_filter.z();
                        tag6_filter.orientation.w = tag6_q_filter.w();
                    }
                    else
                    {
                        tag6.detected.data = false;
                    }
                } 
            }
            else if (tag.id[0] == 7 )
            {
                count_tag7 += 1;

                // tag detected counter
                robot_hand_estimation_tags.push_back(7);

                //last center position and orientation
                last_tag7_position.position.x = tag7_filter.coordinate_center.x;
                last_tag7_position.position.y = tag7_filter.coordinate_center.y;
                last_tag7_position.position.z = tag7_filter.coordinate_center.z;
                last_tag7_position.orientation.x = tag7_filter.orientation.x;
                last_tag7_position.orientation.y = tag7_filter.orientation.y;
                last_tag7_position.orientation.z = tag7_filter.orientation.z;
                last_tag7_position.orientation.w = tag7_filter.orientation.w;

                // tag information
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

                    bool detected = (coordinates.at(0).z == 0 || coordinates.at(1).z == 0 || coordinates.at(2).z == 0 || coordinates.at(3).z == 0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag7_begin = ros::Time::now().toSec();
                        
                        tag7.coordinate1 = coordinates.at(0);
                        tag7.coordinate2 = coordinates.at(1);
                        tag7.coordinate3 = coordinates.at(2);
                        tag7.coordinate4 = coordinates.at(3);
                        tag7.coordinate_center = coordinates.at(4);

                        // tag orientation
                        geometry_msgs::Quaternion orientation;
                        orientation = get_orientation(tag7);
                        tag7.orientation = orientation; 

                        // position filtering
                        Eigen::Vector3d tag7_coordinates(tag7.coordinate_center.x, tag7.coordinate_center.y, tag7.coordinate_center.z);
                        filter_tag7_position.update(tag7_coordinates, 0.0333);
                        Eigen::Vector3d tag7_coordinates_filter = filter_tag7_position.value();
                        tag7_filter.coordinate_center.x = tag7_coordinates_filter.x();
                        tag7_filter.coordinate_center.y = tag7_coordinates_filter.y();
                        tag7_filter.coordinate_center.z = tag7_coordinates_filter.z();

                        // orientation filtering
                        Eigen::Quaterniond tag7_q(tag7.orientation.w, tag7.orientation.x, tag7.orientation.y, tag7.orientation.z);
                        filter_tag7_orientation.update(tag7_q, 0.0333);
                        Eigen::Quaterniond tag7_q_filter = filter_tag7_orientation.valueQuaternion();
                        tag7_filter.orientation.x = tag7_q_filter.x();
                        tag7_filter.orientation.y = tag7_q_filter.y();
                        tag7_filter.orientation.z = tag7_q_filter.z();
                        tag7_filter.orientation.w = tag7_q_filter.w();
                    }
                    else
                    {
                        tag7.detected.data = false;
                    }
                }
            }
            else if (tag.id[0] == 8 )
            {
                count_tag8 += 1;

                // tag detected counter
                robot_hand_estimation_tags.push_back(8);

                //last center position and orientation
                last_tag8_position.position.x = tag8_filter.coordinate_center.x;
                last_tag8_position.position.y = tag8_filter.coordinate_center.y;
                last_tag8_position.position.z = tag8_filter.coordinate_center.z;
                last_tag8_position.orientation.x = tag8_filter.orientation.x;
                last_tag8_position.orientation.y = tag8_filter.orientation.y;
                last_tag8_position.orientation.z = tag8_filter.orientation.z;
                last_tag8_position.orientation.w = tag8_filter.orientation.w;

                // tag information
                tag8.id.push_back(tag.id[0]);
                tag8.size = tag.size;
                tag8.pose = tag.pose;
                tag8.corner1 = tag.corner1;
                tag8.corner2 = tag.corner2;
                tag8.corner3 = tag.corner3;
                tag8.corner4 = tag.corner4;
                tag8.centerpx = tag.centerpx;
                tag8.detected.data = true;

                // pixels to point cloud information
                std::vector<geometry_msgs::Point> coordinates;
                coordinates = get_point_cloud_information(tag8);
                if(coordinates.size() == 5)
                {

                    bool detected = (coordinates.at(0).z == 0 || coordinates.at(1).z == 0 || coordinates.at(2).z == 0 || coordinates.at(3).z == 0) ? false : true;
                    std::cout << "detected: " << detected << std::endl;

                    if(detected)
                    {
                        // time
                        tag8_begin = ros::Time::now().toSec();
                        
                        tag8.coordinate1 = coordinates.at(0);
                        tag8.coordinate2 = coordinates.at(1);
                        tag8.coordinate3 = coordinates.at(2);
                        tag8.coordinate4 = coordinates.at(3);
                        tag8.coordinate_center = coordinates.at(4);

                        // tag orientation
                        geometry_msgs::Quaternion orientation;
                        orientation = get_orientation(tag8);
                        tag8.orientation = orientation; 

                        // position filtering
                        Eigen::Vector3d tag8_coordinates(tag8.coordinate_center.x, tag8.coordinate_center.y, tag8.coordinate_center.z);
                        filter_tag8_position.update(tag8_coordinates, 0.0333);
                        Eigen::Vector3d tag8_coordinates_filter = filter_tag8_position.value();
                        tag8_filter.coordinate_center.x = tag8_coordinates_filter.x();
                        tag8_filter.coordinate_center.y = tag8_coordinates_filter.y();
                        tag8_filter.coordinate_center.z = tag8_coordinates_filter.z();

                        // orientation filtering
                        Eigen::Quaterniond tag8_q(tag8.orientation.w, tag8.orientation.x, tag8.orientation.y, tag8.orientation.z);
                        filter_tag8_orientation.update(tag8_q, 0.0333);
                        Eigen::Quaterniond tag8_q_filter = filter_tag8_orientation.valueQuaternion();
                        tag8_filter.orientation.x = tag8_q_filter.x();
                        tag8_filter.orientation.y = tag8_q_filter.y();
                        tag8_filter.orientation.z = tag8_q_filter.z();
                        tag8_filter.orientation.w = tag8_q_filter.w();
                    }
                    else
                    {
                        tag8.detected.data = false;
                    }
                }
            }
        }

        // when tags are not detected

        // time 
        // tag 5
        double tag5_now = ros::Time::now().toSec();
        double tag5_time = tag5_now - tag5_begin;
        // tag 6
        double tag6_now = ros::Time::now().toSec();
        double tag6_time = tag6_now - tag6_begin;
        // tag 7
        double tag7_now = ros::Time::now().toSec();
        double tag7_time = tag7_now - tag7_begin;
        // tag 8
        double tag8_now = ros::Time::now().toSec();
        double tag8_time = tag8_now - tag8_begin;
        // tag 10
        double tag10_now = ros::Time::now().toSec();
        double tag10_time = tag10_now - tag10_begin;
        // tag 20
        double tag20_now = ros::Time::now().toSec();
        double tag20_time = tag20_now - tag20_begin;

        //tag5 not detected
        if(tag5.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
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
                count_tag5 = 0;
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
            //std::cout << "**time: " << tag5_time << std::endl;
        } 

        //tag6 not detected
        if(tag6.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
            if (tag6_time < 1.5){
                tag6.detected.data = true;
                tag6_filter.coordinate_center.x = last_tag6_position.position.x;
                tag6_filter.coordinate_center.y = last_tag6_position.position.y;
                tag6_filter.coordinate_center.z = last_tag6_position.position.z;
                tag6_filter.orientation.x = last_tag6_position.orientation.x;
                tag6_filter.orientation.y = last_tag6_position.orientation.y;
                tag6_filter.orientation.z = last_tag6_position.orientation.z;
                tag6_filter.orientation.w = last_tag6_position.orientation.w;

            } else if (tag6_time >= 1.5){
                count_tag6 = 0;
                tag6.coordinate_center.x = 0;
                tag6.coordinate_center.y = 0;
                tag6.coordinate_center.z = 0;
                tag6.orientation.x = 0;
                tag6.orientation.y = 0;
                tag6.orientation.z = 0;
                tag6.orientation.w = 0;

                filter_tag6_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag6_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag6_coordinates_filter = filter_tag6_position.value();
                tag6_filter.coordinate_center.x = tag6_coordinates_filter.x();
                tag6_filter.coordinate_center.y = tag6_coordinates_filter.y();
                tag6_filter.coordinate_center.z = tag6_coordinates_filter.z();

                Eigen::Quaterniond tag6_q_filter = filter_tag6_orientation.valueQuaternion();
                tag6_filter.orientation.x = tag6_q_filter.x();
                tag6_filter.orientation.y = tag6_q_filter.y();
                tag6_filter.orientation.z = tag6_q_filter.z();
                tag6_filter.orientation.w = tag6_q_filter.w();

                
            } 
            //std::cout << "**time: " << tag6_time << std::endl;
        } 

        //tag7 not detected
        if(tag7.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
            if (tag7_time < 1.5){
                tag7.detected.data = true;
                tag7_filter.coordinate_center.x = last_tag7_position.position.x;
                tag7_filter.coordinate_center.y = last_tag7_position.position.y;
                tag7_filter.coordinate_center.z = last_tag7_position.position.z;
                tag7_filter.orientation.x = last_tag7_position.orientation.x;
                tag7_filter.orientation.y = last_tag7_position.orientation.y;
                tag7_filter.orientation.z = last_tag7_position.orientation.z;
                tag7_filter.orientation.w = last_tag7_position.orientation.w;

            } else if (tag7_time >= 1.5){
                count_tag7 = 0;
                tag7.coordinate_center.x = 0;
                tag7.coordinate_center.y = 0;
                tag7.coordinate_center.z = 0;
                tag7.orientation.x = 0;
                tag7.orientation.y = 0;
                tag7.orientation.z = 0;
                tag7.orientation.w = 0;

                filter_tag7_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag7_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag7_coordinates_filter = filter_tag7_position.value();
                tag7_filter.coordinate_center.x = tag7_coordinates_filter.x();
                tag7_filter.coordinate_center.y = tag7_coordinates_filter.y();
                tag7_filter.coordinate_center.z = tag7_coordinates_filter.z();

                Eigen::Quaterniond tag7_q_filter = filter_tag7_orientation.valueQuaternion();
                tag7_filter.orientation.x = tag7_q_filter.x();
                tag7_filter.orientation.y = tag7_q_filter.y();
                tag7_filter.orientation.z = tag7_q_filter.z();
                tag7_filter.orientation.w = tag7_q_filter.w();

                
            } 
            //std::cout << "**time: " << tag7_time << std::endl;
        } 

        //tag8 not detected
        if(tag8.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
            if (tag8_time < 1.5){
                tag8.detected.data = true;
                tag8_filter.coordinate_center.x = last_tag8_position.position.x;
                tag8_filter.coordinate_center.y = last_tag8_position.position.y;
                tag8_filter.coordinate_center.z = last_tag8_position.position.z;
                tag8_filter.orientation.x = last_tag8_position.orientation.x;
                tag8_filter.orientation.y = last_tag8_position.orientation.y;
                tag8_filter.orientation.z = last_tag8_position.orientation.z;
                tag8_filter.orientation.w = last_tag8_position.orientation.w;

            } else if (tag8_time >= 1.5){
                count_tag8 = 0;

                tag8.coordinate_center.x = 0;
                tag8.coordinate_center.y = 0;
                tag8.coordinate_center.z = 0;
                tag8.orientation.x = 0;
                tag8.orientation.y = 0;
                tag8.orientation.z = 0;
                tag8.orientation.w = 0;

                filter_tag8_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag8_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag8_coordinates_filter = filter_tag8_position.value();
                tag8_filter.coordinate_center.x = tag8_coordinates_filter.x();
                tag8_filter.coordinate_center.y = tag8_coordinates_filter.y();
                tag8_filter.coordinate_center.z = tag8_coordinates_filter.z();

                Eigen::Quaterniond tag8_q_filter = filter_tag8_orientation.valueQuaternion();
                tag8_filter.orientation.x = tag8_q_filter.x();
                tag8_filter.orientation.y = tag8_q_filter.y();
                tag8_filter.orientation.z = tag8_q_filter.z();
                tag8_filter.orientation.w = tag8_q_filter.w();
            } 
            //std::cout << "**time: " << tag8_time << std::endl;
        } 
        
        //tag10 not detected
        if(tag10.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
            if (tag10_time < 1.5){
                tag10.detected.data = true;
                tag10_filter.coordinate_center.x = last_tag10_position.position.x;
                tag10_filter.coordinate_center.y = last_tag10_position.position.y;
                tag10_filter.coordinate_center.z = last_tag10_position.position.z;
                tag10_filter.orientation.x = last_tag10_position.orientation.x;
                tag10_filter.orientation.y = last_tag10_position.orientation.y;
                tag10_filter.orientation.z = last_tag10_position.orientation.z;
                tag10_filter.orientation.w = last_tag10_position.orientation.w;

            } else if (tag10_time >= 1.5){
                tag10.coordinate_center.x = 0;
                tag10.coordinate_center.y = 0;
                tag10.coordinate_center.z = 0;
                tag10.orientation.x = 0;
                tag10.orientation.y = 0;
                tag10.orientation.z = 0;
                tag10.orientation.w = 0;

                filter_tag10_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag10_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag10_coordinates_filter = filter_tag10_position.value();
                tag10_filter.coordinate_center.x = tag10_coordinates_filter.x();
                tag10_filter.coordinate_center.y = tag10_coordinates_filter.y();
                tag10_filter.coordinate_center.z = tag10_coordinates_filter.z();

                Eigen::Quaterniond tag10_q_filter = filter_tag10_orientation.valueQuaternion();
                tag10_filter.orientation.x = tag10_q_filter.x();
                tag10_filter.orientation.y = tag10_q_filter.y();
                tag10_filter.orientation.z = tag10_q_filter.z();
                tag10_filter.orientation.w = tag10_q_filter.w();

                
            } 
            //std::cout << "**time: " << tag10_time << std::endl;
        } 

        //tag20 not detected
        if(tag20.detected.data == false){
            //std::cout << "*No detected" << std::endl;
            
            if (tag20_time < 1.5){
                tag20.detected.data = true;
                tag20_filter.coordinate_center.x = last_tag20_position.position.x;
                tag20_filter.coordinate_center.y = last_tag20_position.position.y;
                tag20_filter.coordinate_center.z = last_tag20_position.position.z;
                tag20_filter.orientation.x = last_tag20_position.orientation.x;
                tag20_filter.orientation.y = last_tag20_position.orientation.y;
                tag20_filter.orientation.z = last_tag20_position.orientation.z;
                tag20_filter.orientation.w = last_tag20_position.orientation.w;

            } else if (tag20_time >= 1.5){
                tag20.coordinate_center.x = 0;
                tag20.coordinate_center.y = 0;
                tag20.coordinate_center.z = 0;
                tag20.orientation.x = 0;
                tag20.orientation.y = 0;
                tag20.orientation.z = 0;
                tag20.orientation.w = 0;

                filter_tag20_position.reset(Eigen::Vector3d(0,0,0));
                filter_tag20_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                Eigen::Vector3d tag20_coordinates_filter = filter_tag20_position.value();
                tag20_filter.coordinate_center.x = tag20_coordinates_filter.x();
                tag20_filter.coordinate_center.y = tag20_coordinates_filter.y();
                tag20_filter.coordinate_center.z = tag20_coordinates_filter.z();

                Eigen::Quaterniond tag20_q_filter = filter_tag20_orientation.valueQuaternion();
                tag20_filter.orientation.x = tag20_q_filter.x();
                tag20_filter.orientation.y = tag20_q_filter.y();
                tag20_filter.orientation.z = tag20_q_filter.z();
                tag20_filter.orientation.w = tag20_q_filter.w();

                
            } 
            //std::cout << "**time: " << tag20_time << std::endl;
        } 

        // robot hand estimation

        // valid filter data after estabilization time
        tag5_filter.detected.data = count_tag5 >= 18 ? true : false;
        tag6_filter.detected.data = count_tag6 >= 18 ? true : false;
        tag7_filter.detected.data = count_tag7 >= 18 ? true : false;
        tag8_filter.detected.data = count_tag8 >= 18 ? true : false;

        // tag size 
        float tag_size = 0.11;

        int robot_hand_estimation_tags_size = robot_hand_estimation_tags.size();
        // for(int i = 0; i < robot_hand_estimation_tags_size; i++){
        //     std::cout << "Tags detected on screen: " << robot_hand_estimation_tags.at(i) << std::endl;
        // }
    
        if (robot_hand_estimation_tags_size != 0)
        {
            int tag_detected = robot_hand_estimation_tags.at(0);
            //std::cout << "Tag detected: " << tag_detected << std::endl;
            if(tag_detected == 5 and tag5_filter.detected.data)
            {
                std::cout << "Tag5" << std::endl;
                Eigen::Quaterniond q_tag5;
                q_tag5.w() = tag5_filter.orientation.w;
                q_tag5.x() = tag5_filter.orientation.x;
                q_tag5.y() = tag5_filter.orientation.y;
                q_tag5.z() = tag5_filter.orientation.z;

                //finding new orientation
                //turns 90 degrees (pi) in z axis (option3)
                Eigen::Quaterniond q_robot_hand = multiply_rotation_matrix(q_tag5, M_PI/2, 3);

                //finding new position
                Eigen::Matrix3d r = q_robot_hand.toRotationMatrix();
                Eigen::Matrix<double,3,1> p_now{tag5_filter.coordinate_center.x, tag5_filter.coordinate_center.y, tag5_filter.coordinate_center.z};
                Eigen::Matrix<double,3,1> p_next{0, 0, -0.065};
                Eigen::Matrix<double,3,1> p_new = r*p_next + p_now;
                
                robot_hand.header.seq = 1;
                robot_hand.header.stamp = ros::Time::now();
                robot_hand.header.frame_id = "map";
                robot_hand.pose.position.x = p_new(0,0);
                robot_hand.pose.position.y = p_new(1,0);
                robot_hand.pose.position.z = p_new(2,0);
                robot_hand.pose.orientation.x = q_robot_hand.x();
                robot_hand.pose.orientation.y = q_robot_hand.y();
                robot_hand.pose.orientation.z = q_robot_hand.z();
                robot_hand.pose.orientation.w = q_robot_hand.w();
            }
            else if(tag_detected == 6 and tag6_filter.detected.data)
            {
                std::cout << "Tag6" << std::endl;

                Eigen::Quaterniond q_tag6;
                q_tag6.w() = tag6_filter.orientation.w;
                q_tag6.x() = tag6_filter.orientation.x;
                q_tag6.y() = tag6_filter.orientation.y;
                q_tag6.z() = tag6_filter.orientation.z;

                // finding new orientation
                //turns 90 degrees (pi) in z axis (option3)
                Eigen::Quaterniond q_robot_hand_r = multiply_rotation_matrix(q_tag6, -M_PI/2, 3);
                //turns -90 degrees (pi) in x axis (option1)
                Eigen::Quaterniond q_robot_hand = multiply_rotation_matrix(q_robot_hand_r, -M_PI/2, 1);

                //finding new position
                Eigen::Matrix3d r = q_robot_hand.toRotationMatrix();
                Eigen::Matrix<double,3,1> p_now{tag6_filter.coordinate_center.x, tag6_filter.coordinate_center.y, tag6_filter.coordinate_center.z};
                Eigen::Matrix<double,3,1> p_next{0, 0.065, 0};
                Eigen::Matrix<double,3,1> p_new = r*p_next + p_now;

                robot_hand.header.seq = 1;
                robot_hand.header.stamp = ros::Time::now();
                robot_hand.header.frame_id = "map";
                robot_hand.pose.position.x = p_new(0,0);
                robot_hand.pose.position.y = p_new(1,0);
                robot_hand.pose.position.z = p_new(2,0);
                robot_hand.pose.orientation.x = q_robot_hand.x();
                robot_hand.pose.orientation.y = q_robot_hand.y();
                robot_hand.pose.orientation.z = q_robot_hand.z();
                robot_hand.pose.orientation.w = q_robot_hand.w();
            }
            else if(tag_detected == 7 and tag7_filter.detected.data)
            {
                std::cout << "Tag7" << std::endl;
                Eigen::Quaterniond q_tag7(tag7_filter.orientation.w, tag7_filter.orientation.x, tag7_filter.orientation.y, tag7_filter.orientation.z);

                // finding new orientation
                //turns 90 degrees (pi) in z axis (option3)
                Eigen::Quaterniond q_robot_hand_r = multiply_rotation_matrix(q_tag7, -M_PI/2, 3);
                //turns 180 degrees (pi) in x axis (option1)
                Eigen::Quaterniond q_robot_hand = multiply_rotation_matrix(q_robot_hand_r, M_PI, 1);

                //finding new position
                Eigen::Matrix3d r = q_robot_hand.toRotationMatrix();
                Eigen::Matrix<double,3,1> p_now{tag7_filter.coordinate_center.x, tag7_filter.coordinate_center.y, tag7_filter.coordinate_center.z};
                Eigen::Matrix<double,3,1> p_next{0, 0, 0.065};
                Eigen::Matrix<double,3,1> p_new = r*p_next + p_now;

                robot_hand.header.seq = 1;
                robot_hand.header.stamp = ros::Time::now();
                robot_hand.header.frame_id = "map";
                robot_hand.pose.position.x = p_new(0,0);
                robot_hand.pose.position.y = p_new(1,0);
                robot_hand.pose.position.z = p_new(2,0);
                robot_hand.pose.orientation.x = q_robot_hand.x();
                robot_hand.pose.orientation.y = q_robot_hand.y();
                robot_hand.pose.orientation.z = q_robot_hand.z();
                robot_hand.pose.orientation.w = q_robot_hand.w();

            }
            else if(tag_detected == 8 and tag8_filter.detected.data)
            {
                std::cout << "Tag8" << std::endl;
                Eigen::Quaterniond q_tag8(tag8_filter.orientation.w, tag8_filter.orientation.x, tag8_filter.orientation.y, tag8_filter.orientation.z);

                //finding new position
                //turns 90 degrees (pi) in z axis (option3)
                Eigen::Quaterniond q_robot_hand_r = multiply_rotation_matrix(q_tag8, -M_PI/2, 3);
                //turns 90 degrees (pi) in x axis (option1)
                Eigen::Quaterniond q_robot_hand = multiply_rotation_matrix(q_robot_hand_r, M_PI/2, 1);

                //finding new position
                Eigen::Matrix3d r = q_robot_hand.toRotationMatrix();
                Eigen::Matrix<double,3,1> p_now{tag8_filter.coordinate_center.x, tag8_filter.coordinate_center.y, tag8_filter.coordinate_center.z};
                Eigen::Matrix<double,3,1> p_next{0, -0.065, 0};
                Eigen::Matrix<double,3,1> p_new = r*p_next + p_now;

                robot_hand.header.seq = 1;
                robot_hand.header.stamp = ros::Time::now();
                robot_hand.header.frame_id = "map";
                robot_hand.pose.position.x = p_new(0,0);
                robot_hand.pose.position.y = p_new(1,0);
                robot_hand.pose.position.z = p_new(2,0);
                robot_hand.pose.orientation.x = q_robot_hand.x();
                robot_hand.pose.orientation.y = q_robot_hand.y();
                robot_hand.pose.orientation.z = q_robot_hand.z();
                robot_hand.pose.orientation.w = q_robot_hand.w();
            }

        }

        tag10_filter.detected.data = tag10.detected.data;
        tag20_filter.detected.data = tag20.detected.data;
        
        // tags 
        tag5_publisher.publish(tag5);
        tag6_publisher.publish(tag6);
        tag7_publisher.publish(tag7);
        tag8_publisher.publish(tag8);
        tag10_publisher.publish(tag10);
        tag20_publisher.publish(tag20);

        // filter tag
        tag5_filter_publisher.publish(tag5_filter);
        tag6_filter_publisher.publish(tag6_filter);
        tag7_filter_publisher.publish(tag7_filter);
        tag8_filter_publisher.publish(tag8_filter);
        tag10_filter_publisher.publish(tag10_filter);
        tag20_filter_publisher.publish(tag20_filter);

        if(tag5_filter.detected.data || tag6_filter.detected.data || tag7_filter.detected.data || tag8_filter.detected.data ){
            //robot hand pose publisher
            pose_publisher.publish(robot_hand);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}