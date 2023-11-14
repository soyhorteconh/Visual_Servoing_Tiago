#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTag.h"
#include "apriltag_ros/RobotHand.h"
#include "apriltag_ros/TiagoHand.h"
#include "cmath"
#include "geometry_msgs/PoseStamped.h"
#include <inria_maths/FilterExponential.hpp>
#include <inria_maths/FilterExponentialRotation.hpp>
#include "ros/ros.h"
#include "vector"

cv::Mat depth_image;
apriltag_ros::AprilTagDetectionArray tags;

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

    depth_image = depth -> image;
}

// rotation matrixes
Eigen::Matrix3d x_rotation(double radians)
{
    Eigen::Matrix3d rotation;
    rotation << 1, 0, 0,
                0, std::cos(radians), -std::sin(radians),
                0, std::sin(radians), std::cos(radians);
    
    return rotation;
}

Eigen::Matrix3d y_rotation(double radians)
{
    Eigen::Matrix3d rotation;
    rotation << std::cos(radians), 0, std::sin(radians),
                0, 1, 0,
                -std::sin(radians), 0, std::cos(radians);
    
    return rotation;
}

Eigen::Matrix3d z_rotation(double radians)
{
    Eigen::Matrix3d rotation;
    rotation << std::cos(radians), -std::sin(radians), 0,
                std::sin(radians), std::cos(radians), 0,
                0, 0, 1;

    return rotation;
}

// translation vectors
Eigen::Vector3d x_translation(double distance)
{
    Eigen::Vector3d translation(distance, 0.0, 0.0);
    return translation;
}

Eigen::Vector3d y_translation(double distance)
{
    Eigen::Vector3d translation(0.0, distance, 0.0);
    return translation;
}

Eigen::Vector3d z_translation(double distance)
{
    Eigen::Vector3d translation(0.0, 0.0, distance);
    return translation;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_information");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher tag5_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag5/information", 1 );
    ros::Publisher tag6_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag6/information", 1 );
    ros::Publisher tag7_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag7/information", 1 );
    ros::Publisher tag8_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag8/information", 1 );
    ros::Publisher tag10_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag10/information", 1 );

    ros::Publisher tag5_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag5/filter", 1 );
    ros::Publisher tag6_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag6/filter", 1 );
    ros::Publisher tag7_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag7/filter", 1 );
    ros::Publisher tag8_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag8/filter", 1 );
    ros::Publisher tag10_filter_publisher = n.advertise<apriltag_ros::AprilTagDetection>( "tag10/filter", 1 );

    ros::Publisher pose_publisher = n.advertise<apriltag_ros::RobotHand>("robot_hand/pose", 1);
    ros::Publisher rh_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand", 1);

    // Subscribers
    ros::Subscriber sub = n.subscribe("tag_detections",1,apriltag_cb);
    ros::Subscriber depth_subscriber = n.subscribe("depth_to_rgb/image", 1, video_cb);

    // variables
    apriltag_ros::AprilTagDetection tag;

    AprilTag tag10;
    AprilTag tag5;
    AprilTag tag6;
    AprilTag tag7;
    AprilTag tag8;

    TiagoHand robot_hand;

    // time variables
    double begin = ros::Time::now().toSec();
    double tag10_start = ros::Time::now().toSec();
    double tag5_start = ros::Time::now().toSec();
    double tag6_start = ros::Time::now().toSec();
    double tag7_start = ros::Time::now().toSec();
    double tag8_start = ros::Time::now().toSec();

    //filters for position
    inria::FilterExponential<Eigen::Vector3d> filter_tag5_position;
    inria::FilterExponential<Eigen::Vector3d> filter_tag6_position;
    inria::FilterExponential<Eigen::Vector3d> filter_tag7_position;
    inria::FilterExponential<Eigen::Vector3d> filter_tag8_position;
    inria::FilterExponential<Eigen::Vector3d> filter_tag10_position;

    // filter for pose
    inria::FilterExponentialRotation filter_tag5_orientation;
    inria::FilterExponentialRotation filter_tag6_orientation;
    inria::FilterExponentialRotation filter_tag7_orientation;
    inria::FilterExponentialRotation filter_tag8_orientation;
    inria::FilterExponentialRotation filter_tag10_orientation;

    //cutoff Frequency
    filter_tag5_position.cutoffFrequency() = 1;
    filter_tag6_position.cutoffFrequency() = 1;
    filter_tag7_position.cutoffFrequency() = 1;
    filter_tag8_position.cutoffFrequency() = 1;
    filter_tag10_position.cutoffFrequency() = 1;

    filter_tag5_orientation.cutoffFrequency() = 1;
    filter_tag6_orientation.cutoffFrequency() = 1;
    filter_tag7_orientation.cutoffFrequency() = 1;
    filter_tag8_orientation.cutoffFrequency() = 1;
    filter_tag10_orientation.cutoffFrequency() = 1;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        double now = ros::Time::now().toSec();
        double start_time = now - begin;
        ROS_INFO_STREAM_ONCE("loading ...");

        tag10.not_detected();
        tag5.not_detected();
        tag6.not_detected();
        tag7.not_detected();
        tag8.not_detected();

        robot_hand.not_detected();

        std::vector<int> robot_hand_tags;

        // 6 seconds to start
        if( start_time >= 6)
        {
            ROS_INFO_STREAM_ONCE("Node initialized");
            std::cout << "---------------------------------------------" << std::endl;
            for (int i = 0; i < tags.detections.size(); i++)
            {
                tag  = tags.detections.at(i);
                Eigen::Vector3d c1(tag.corner1.x, tag.corner1.y, 0.0);
                Eigen::Vector3d c2(tag.corner2.x, tag.corner2.y, 0.0);
                Eigen::Vector3d c3(tag.corner3.x, tag.corner3.y, 0.0);
                Eigen::Vector3d c4(tag.corner4.x, tag.corner4.y, 0.0);
                Eigen::Vector3d center(tag.centerpx.x, tag.centerpx.y, 0.0);
                
                if(tag.id.size() > 0){
                    //tag10 detcted
                    if(tag.id[0] == 10)
                    {
                        //reset time
                        tag10_start = ros::Time::now().toSec();
                        std::cout << "Tag10 detected" << std::endl;
                        //get tag pose
                        tag10.activate(c1, c2, c3, c4, center, depth_image);

                        //filter
                        filter_tag10_position.update(tag10.getPosition(), 0.0333);
                        tag10.setPostionFilter(filter_tag10_position.value());

                        filter_tag10_orientation.update(tag10.getOrientation(), 0.0333);
                        tag10.setOrientationFilter(filter_tag10_orientation.valueQuaternion());
                    }

                    // tag6 detected
                    else if(tag.id[0] == 5)
                    {
                        //reset time
                        tag5_start = ros::Time::now().toSec();
                        //std::cout << "Tag5 detected" << std::endl;
                        //get tag pose
                        tag5.activate(c1, c2, c3, c4, center, depth_image);
                        robot_hand_tags.push_back(5);

                        //filter
                        filter_tag5_position.update(tag5.getPosition(), 0.0333);
                        tag5.setPostionFilter(filter_tag5_position.value());

                        filter_tag5_orientation.update(tag5.getOrientation(), 0.0333);
                        tag5.setOrientationFilter(filter_tag5_orientation.valueQuaternion());
                    }

                    // tag6 detected
                    else if(tag.id[0] == 6)
                    {
                        //reset time
                        tag6_start = ros::Time::now().toSec();
                        //std::cout << "Tag6 detected" << std::endl;
                        //get tag pose
                        tag6.activate(c1, c2, c3, c4, center, depth_image);
                        robot_hand_tags.push_back(6);

                        //filter
                        filter_tag6_position.update(tag6.getPosition(), 0.0333);
                        tag6.setPostionFilter(filter_tag6_position.value());

                        filter_tag6_orientation.update(tag6.getOrientation(), 0.0333);
                        tag6.setOrientationFilter(filter_tag6_orientation.valueQuaternion());
                    }

                    // tag7 detected
                    else if(tag.id[0] == 7)
                    {
                        //reset time
                        tag7_start = ros::Time::now().toSec();
                        //std::cout << "Tag7 detected" << std::endl;
                        //get tag pose
                        tag7.activate(c1, c2, c3, c4, center, depth_image);
                        robot_hand_tags.push_back(7);

                        //filter
                        filter_tag7_position.update(tag7.getPosition(), 0.0333);
                        tag7.setPostionFilter(filter_tag7_position.value());

                        filter_tag7_orientation.update(tag7.getOrientation(), 0.0333);
                        tag7.setOrientationFilter(filter_tag7_orientation.valueQuaternion());
                    }

                    // tag8 detected
                    else if(tag.id[0] == 8)
                    {
                        //reset time
                        tag8_start = ros::Time::now().toSec();
                        //std::cout << "Tag8 detected" << std::endl;
                        //get tag pose
                        tag8.activate(c1, c2, c3, c4, center, depth_image);
                        robot_hand_tags.push_back(8);

                        //filter
                        filter_tag8_position.update(tag8.getPosition(), 0.0333);
                        tag8.setPostionFilter(filter_tag8_position.value());

                        filter_tag8_orientation.update(tag8.getOrientation(), 0.0333);
                        tag8.setOrientationFilter(filter_tag8_orientation.valueQuaternion());
                    }
                }
            }

            //delay time
            double delay_time = 1.0;
            //when tags are not detcted
            if(tag10.getDetected() == false)
            {
                double tag10_time_not_detected = ros::Time::now().toSec() - tag10_start;
                if(tag10_time_not_detected <= delay_time)
                {
                    tag10.is_detected();
                }
                else if (tag10_time_not_detected > delay_time)
                {
                    tag10.setPostion(Eigen::Vector3d(0.0, 0.0, 0.0));
                    tag10.setOrientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));

                    filter_tag10_position.reset(Eigen::Vector3d(0,0,0));
                    filter_tag10_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                    tag10.setOrientationFilter(filter_tag10_orientation.valueQuaternion());
                    tag10.setPostionFilter(filter_tag10_position.value());
                }
            }

            if(tag5.getDetected() == false)
            {
                double tag5_time_not_detected = ros::Time::now().toSec() - tag5_start;
                if(tag5_time_not_detected <= delay_time)
                {
                    tag5.is_detected();
                }
                else if (tag5_time_not_detected > delay_time)
                {
                    tag5.setPostion(Eigen::Vector3d(0.0, 0.0, 0.0));
                    tag5.setOrientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));

                    filter_tag5_position.reset(Eigen::Vector3d(0,0,0));
                    filter_tag5_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                    tag5.setOrientationFilter(filter_tag5_orientation.valueQuaternion());
                    tag5.setPostionFilter(filter_tag5_position.value());
                }
            }

            if(tag6.getDetected() == false)
            {
                double tag6_time_not_detected = ros::Time::now().toSec() - tag6_start;
                if(tag6_time_not_detected <= delay_time)
                {
                    robot_hand_tags.push_back(6);
                    tag6.is_detected();
                }
                else if (tag6_time_not_detected > delay_time)
                {
                    tag6.setPostion(Eigen::Vector3d(0.0, 0.0, 0.0));
                    tag6.setOrientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));

                    filter_tag6_position.reset(Eigen::Vector3d(0,0,0));
                    filter_tag6_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                    tag6.setOrientationFilter(filter_tag6_orientation.valueQuaternion());
                    tag6.setPostionFilter(filter_tag6_position.value());
                }
            }
            
            if(tag7.getDetected() == false)
            {
                double tag7_time_not_detected = ros::Time::now().toSec() - tag7_start;
                if(tag7_time_not_detected <= delay_time)
                {
                    robot_hand_tags.push_back(7);
                    tag7.is_detected();
                }
                else if (tag7_time_not_detected > delay_time)
                {
                    tag7.setPostion(Eigen::Vector3d(0.0, 0.0, 0.0));
                    tag7.setOrientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));

                    filter_tag7_position.reset(Eigen::Vector3d(0,0,0));
                    filter_tag7_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                    tag7.setOrientationFilter(filter_tag7_orientation.valueQuaternion());
                    tag7.setPostionFilter(filter_tag7_position.value());
                }
            }

            if(tag8.getDetected() == false)
            {
                double tag8_time_not_detected = ros::Time::now().toSec() - tag8_start;
                if(tag8_time_not_detected <= delay_time)
                {
                    robot_hand_tags.push_back(8);
                    tag8.is_detected();
                }
                else if (tag8_time_not_detected > delay_time)
                {
                    tag8.setPostion(Eigen::Vector3d(0.0, 0.0, 0.0));
                    tag8.setOrientation(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0));

                    filter_tag8_position.reset(Eigen::Vector3d(0,0,0));
                    filter_tag8_orientation.reset(Eigen::Quaterniond(0,0,0,0));

                    tag8.setOrientationFilter(filter_tag8_orientation.valueQuaternion());
                    tag8.setPostionFilter(filter_tag8_position.value());
                }
            }

            // robot hand pose
            if(robot_hand_tags.size() != 0)
            {
                std::cout << "RobotHand detected" << std::endl;
                std::cout << "[";
                for(int i = 0; i < robot_hand_tags.size(); i++){
                    std::cout << robot_hand_tags[i] << " ";
                }
                std::cout << "]" << std::endl;

                int tag_detected = robot_hand_tags.at(0);
                if(tag_detected == 5)
                {
                    Eigen::Matrix3d robot_hand_orientation = tag5.getOrientationFilter().toRotationMatrix() * z_rotation(M_PI/2);
                    Eigen::Quaterniond robot_hand_q(robot_hand_orientation);
                    Eigen::Vector3d robot_hand_position = robot_hand_orientation * z_translation(-0.045) +  tag5.getPostionFilter();

                    robot_hand.setOrientation(robot_hand_q.normalized());
                    robot_hand.setPostion(robot_hand_position);
                    robot_hand.is_detected();
                }
                else if(tag_detected == 6)
                {
                    Eigen::Matrix3d robot_hand_orientation = tag6.getOrientationFilter().toRotationMatrix() * z_rotation(-M_PI/2);
                    robot_hand_orientation = robot_hand_orientation * x_rotation(-M_PI/2);
                    Eigen::Quaterniond robot_hand_q(robot_hand_orientation);
                    Eigen::Vector3d robot_hand_position = robot_hand_orientation * y_translation(0.045) +  tag6.getPostionFilter();

                    robot_hand.setOrientation(robot_hand_q.normalized());
                    robot_hand.setPostion(robot_hand_position);
                    robot_hand.is_detected();
                }
                else if(tag_detected == 7)
                {
                    Eigen::Matrix3d robot_hand_orientation = tag7.getOrientationFilter().toRotationMatrix() * z_rotation(-M_PI/2);
                    robot_hand_orientation = robot_hand_orientation * x_rotation(M_PI);
                    Eigen::Quaterniond robot_hand_q(robot_hand_orientation);
                    Eigen::Vector3d robot_hand_position = robot_hand_orientation * z_translation(0.045) +  tag7.getPostionFilter();

                    robot_hand.setOrientation(robot_hand_q.normalized());
                    robot_hand.setPostion(robot_hand_position);
                    robot_hand.is_detected();
                }
                else if(tag_detected == 8)
                {
                    Eigen::Matrix3d robot_hand_orientation = tag8.getOrientationFilter().toRotationMatrix() * z_rotation(-M_PI/2);
                    robot_hand_orientation = robot_hand_orientation * x_rotation(M_PI/2);
                    Eigen::Quaterniond robot_hand_q(robot_hand_orientation);
                    Eigen::Vector3d robot_hand_position = robot_hand_orientation * y_translation(-0.045) +  tag8.getPostionFilter();

                    robot_hand.setOrientation(robot_hand_q.normalized());
                    robot_hand.setPostion(robot_hand_position);
                    robot_hand.is_detected();
                }

            }

            // tag5
            apriltag_ros::AprilTagDetection tag5_msg;
            tag5_msg.coordinate_center.x = tag5.getPosition().x();
            tag5_msg.coordinate_center.y = tag5.getPosition().y();
            tag5_msg.coordinate_center.z = tag5.getPosition().z();
            tag5_msg.orientation.x = tag5.getOrientation().x();
            tag5_msg.orientation.y = tag5.getOrientation().y();
            tag5_msg.orientation.z = tag5.getOrientation().z();
            tag5_msg.orientation.w = tag5.getOrientation().w();
            tag5_msg.detected.data = tag5.getDetected();

            apriltag_ros::AprilTagDetection tag5_filter_msg;
            tag5_filter_msg.coordinate_center.x = tag5.getPostionFilter().x();
            tag5_filter_msg.coordinate_center.y = tag5.getPostionFilter().y();
            tag5_filter_msg.coordinate_center.z = tag5.getPostionFilter().z();
            tag5_filter_msg.orientation.x = tag5.getOrientationFilter().x();
            tag5_filter_msg.orientation.y = tag5.getOrientationFilter().y();
            tag5_filter_msg.orientation.z = tag5.getOrientationFilter().z();
            tag5_filter_msg.orientation.w = tag5.getOrientationFilter().w();
            tag5_filter_msg.detected.data = tag5.getDetected();
            
            // tag6
            apriltag_ros::AprilTagDetection tag6_msg;
            tag6_msg.coordinate_center.x = tag6.getPosition().x();
            tag6_msg.coordinate_center.y = tag6.getPosition().y();
            tag6_msg.coordinate_center.z = tag6.getPosition().z();
            tag6_msg.orientation.x = tag6.getOrientation().x();
            tag6_msg.orientation.y = tag6.getOrientation().y();
            tag6_msg.orientation.z = tag6.getOrientation().z();
            tag6_msg.orientation.w = tag6.getOrientation().w();
            tag6_msg.detected.data = tag6.getDetected();

            apriltag_ros::AprilTagDetection tag6_filter_msg;
            tag6_filter_msg.coordinate_center.x = tag6.getPostionFilter().x();
            tag6_filter_msg.coordinate_center.y = tag6.getPostionFilter().y();
            tag6_filter_msg.coordinate_center.z = tag6.getPostionFilter().z();
            tag6_filter_msg.orientation.x = tag6.getOrientationFilter().x();
            tag6_filter_msg.orientation.y = tag6.getOrientationFilter().y();
            tag6_filter_msg.orientation.z = tag6.getOrientationFilter().z();
            tag6_filter_msg.orientation.w = tag6.getOrientationFilter().w();
            tag6_filter_msg.detected.data = tag6.getDetected();

            // tag7
            apriltag_ros::AprilTagDetection tag7_msg;
            tag7_msg.coordinate_center.x = tag7.getPosition().x();
            tag7_msg.coordinate_center.y = tag7.getPosition().y();
            tag7_msg.coordinate_center.z = tag7.getPosition().z();
            tag7_msg.orientation.x = tag7.getOrientation().x();
            tag7_msg.orientation.y = tag7.getOrientation().y();
            tag7_msg.orientation.z = tag7.getOrientation().z();
            tag7_msg.orientation.w = tag7.getOrientation().w();
            tag7_msg.detected.data = tag7.getDetected();

            apriltag_ros::AprilTagDetection tag7_filter_msg;
            tag7_filter_msg.coordinate_center.x = tag7.getPostionFilter().x();
            tag7_filter_msg.coordinate_center.y = tag7.getPostionFilter().y();
            tag7_filter_msg.coordinate_center.z = tag7.getPostionFilter().z();
            tag7_filter_msg.orientation.x = tag7.getOrientationFilter().x();
            tag7_filter_msg.orientation.y = tag7.getOrientationFilter().y();
            tag7_filter_msg.orientation.z = tag7.getOrientationFilter().z();
            tag7_filter_msg.orientation.w = tag7.getOrientationFilter().w();
            tag7_filter_msg.detected.data = tag7.getDetected();

            // tag8
            apriltag_ros::AprilTagDetection tag8_msg;
            tag8_msg.coordinate_center.x = tag8.getPosition().x();
            tag8_msg.coordinate_center.y = tag8.getPosition().y();
            tag8_msg.coordinate_center.z = tag8.getPosition().z();
            tag8_msg.orientation.x = tag8.getOrientation().x();
            tag8_msg.orientation.y = tag8.getOrientation().y();
            tag8_msg.orientation.z = tag8.getOrientation().z();
            tag8_msg.orientation.w = tag8.getOrientation().w();
            tag8_msg.detected.data = tag8.getDetected();

            apriltag_ros::AprilTagDetection tag8_filter_msg;
            tag8_filter_msg.coordinate_center.x = tag8.getPostionFilter().x();
            tag8_filter_msg.coordinate_center.y = tag8.getPostionFilter().y();
            tag8_filter_msg.coordinate_center.z = tag8.getPostionFilter().z();
            tag8_filter_msg.orientation.x = tag8.getOrientationFilter().x();
            tag8_filter_msg.orientation.y = tag8.getOrientationFilter().y();
            tag8_filter_msg.orientation.z = tag8.getOrientationFilter().z();
            tag8_filter_msg.orientation.w = tag8.getOrientationFilter().w();
            tag8_filter_msg.detected.data = tag8.getDetected();

            // tag10
            apriltag_ros::AprilTagDetection tag10_msg;
            tag10_msg.coordinate_center.x = tag10.getPosition().x();
            tag10_msg.coordinate_center.y = tag10.getPosition().y();
            tag10_msg.coordinate_center.z = tag10.getPosition().z();
            tag10_msg.orientation.x = tag10.getOrientation().x();
            tag10_msg.orientation.y = tag10.getOrientation().y();
            tag10_msg.orientation.z = tag10.getOrientation().z();
            tag10_msg.orientation.w = tag10.getOrientation().w();
            tag10_msg.detected.data = tag10.getDetected();

            apriltag_ros::AprilTagDetection tag10_filter_msg;
            tag10_filter_msg.coordinate_center.x = tag10.getPostionFilter().x();
            tag10_filter_msg.coordinate_center.y = tag10.getPostionFilter().y();
            tag10_filter_msg.coordinate_center.z = tag10.getPostionFilter().z();
            tag10_filter_msg.orientation.x = tag10.getOrientationFilter().x();
            tag10_filter_msg.orientation.y = tag10.getOrientationFilter().y();
            tag10_filter_msg.orientation.z = tag10.getOrientationFilter().z();
            tag10_filter_msg.orientation.w = tag10.getOrientationFilter().w();
            tag10_filter_msg.detected.data = tag10.getDetected();

            // tiago hand 
            geometry_msgs::PoseStamped tiago_hand_msg;
            tiago_hand_msg.header.seq = 1;
            tiago_hand_msg.header.stamp = ros::Time::now();
            tiago_hand_msg.header.frame_id = "map";
            tiago_hand_msg.pose.position.x = robot_hand.getPosition().x();
            tiago_hand_msg.pose.position.y = robot_hand.getPosition().y();
            tiago_hand_msg.pose.position.z = robot_hand.getPosition().z();
            tiago_hand_msg.pose.orientation.x = robot_hand.getOrientation().x();
            tiago_hand_msg.pose.orientation.y = robot_hand.getOrientation().y();
            tiago_hand_msg.pose.orientation.z = robot_hand.getOrientation().z();
            tiago_hand_msg.pose.orientation.w = robot_hand.getOrientation().w();

            apriltag_ros::RobotHand robot_hand_msg;
            robot_hand_msg.pose = tiago_hand_msg;
            robot_hand_msg.detected.data = robot_hand.getDetected();

            //publisher
            tag5_publisher.publish(tag5_msg);
            tag6_publisher.publish(tag6_msg);
            tag7_publisher.publish(tag7_msg);
            tag8_publisher.publish(tag8_msg);
            tag10_publisher.publish(tag10_msg);

            tag5_filter_publisher.publish(tag5_filter_msg);
            tag6_filter_publisher.publish(tag6_filter_msg);
            tag7_filter_publisher.publish(tag7_filter_msg);
            tag8_filter_publisher.publish(tag8_filter_msg);
            tag10_filter_publisher.publish(tag10_filter_msg);

            rh_pose_publisher.publish(tiago_hand_msg);
            pose_publisher.publish(robot_hand_msg);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}