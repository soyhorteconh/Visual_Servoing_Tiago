#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Geometry>
#include <iostream>

geometry_msgs::PoseStamped robot_hand;

apriltag_ros::AprilTagDetection tag5;
apriltag_ros::AprilTagDetection tag6;
apriltag_ros::AprilTagDetection tag7;
apriltag_ros::AprilTagDetection tag8;
apriltag_ros::AprilTagDetection tag10;

// Callbacks
void tag5_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag5.coordinate_center = msg -> coordinate_center;
    tag5.orientation = msg -> orientation;
    tag5.detected = msg -> detected;
}

void tag6_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag6.coordinate_center = msg -> coordinate_center;
    tag6.orientation = msg -> orientation;
    tag6.detected = msg -> detected;
}

void tag7_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag7.coordinate_center = msg -> coordinate_center;
    tag7.orientation = msg -> orientation;
    tag7.detected = msg -> detected;
}

void tag8_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag8.coordinate_center = msg -> coordinate_center;
    tag8.orientation = msg -> orientation;
    tag8.detected = msg -> detected;
}

void tag10_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag10.coordinate_center = msg -> coordinate_center;
    tag10.orientation = msg -> orientation;
    tag10.detected = msg -> detected;
}

void robot_hand_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    robot_hand.pose = msg -> pose;
}

// main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_transformation_matrix");
    ros::NodeHandle n;

    //publishers
    ros::Publisher tag10_publisher = n.advertise<visualization_msgs::Marker>("tag_origin", 0);
    ros::Publisher robot_hand_publisher = n.advertise<geometry_msgs::PoseStamped>("hand", 1);

    // subscribers
    ros::Subscriber robot_hand_subscriber = n.subscribe("robot_hand/pose", 1, robot_hand_cb); 

    ros::Subscriber tag5_subscriber = n.subscribe("tag5/filter", 1, tag5_cb);
    ros::Subscriber tag6_subscriber = n.subscribe("tag6/filter", 1, tag6_cb);
    ros::Subscriber tag7_subscriber = n.subscribe("tag7/filter", 1, tag7_cb); 
    ros::Subscriber tag8_subscriber = n.subscribe("tag8/filter", 1, tag8_cb);
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/filter", 1, tag10_cb); 

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        // converting rotation to a quaternion form
        Eigen::Quaterniond q_10(tag10.orientation.w, tag10.orientation.x, tag10.orientation.y, tag10.orientation.z);
        Eigen::Quaterniond q_robot_hand(robot_hand.pose.orientation.w, robot_hand.pose.orientation.x, robot_hand.pose.orientation.y, robot_hand.pose.orientation.z);

        // quaternion to rotation matrix
        Eigen::Matrix3d r_10 = q_10.normalized().toRotationMatrix();
        Eigen::Matrix3d r_robot_hand = q_robot_hand.normalized().toRotationMatrix();

        //translation matrix
        Eigen::Matrix<double,3,1> p_10{tag10.coordinate_center.x,tag10.coordinate_center.y,tag10.coordinate_center.z};
        Eigen::Matrix<double,3,1> p_robot_hand{robot_hand.pose.position.x, robot_hand.pose.position.y, robot_hand.pose.position.z};

        // new rotation
        Eigen::Matrix3d r_rh_10 = r_robot_hand.transpose()*r_10;
        Eigen::Quaterniond q(r_rh_10);
        //new position
        Eigen::Matrix<double,3,1> p_rh_10 = r_robot_hand.transpose()*(p_10 - p_robot_hand);

        geometry_msgs::PoseStamped hand;
        hand.header.seq = 1;
        hand.header.stamp = ros::Time::now();
        hand.header.frame_id = "map";
        hand.pose.position.x = 0;
        hand.pose.position.y = 0;
        hand.pose.position.z = 0;
        hand.pose.orientation.x = 0;
        hand.pose.orientation.y = 0;
        hand.pose.orientation.z = 0;
        hand.pose.orientation.w = 1;

        // Marker
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time();
        marker1.ns = "tag10";
        marker1.id = 10;
        marker1.mesh_resource = "package://apriltag_ros/scripts/mesh/tag10.dae";
        marker1.mesh_use_embedded_materials = true;
        marker1.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = p_rh_10(0,0);
        marker1.pose.position.y = p_rh_10(1,0);
        marker1.pose.position.z = p_rh_10(2,0);
        marker1.pose.orientation.x = q.x();
        marker1.pose.orientation.y = q.y();
        marker1.pose.orientation.z = q.z();
        marker1.pose.orientation.w = q.w();
        marker1.scale.x = 1;
        marker1.scale.y = 1;
        marker1.scale.z = 0.25;
        marker1.color.a = 1.0; //alpha
        marker1.color.r = 0.0;
        marker1.color.g = 0.0;
        marker1.color.b = 0.0;

        if (tag5.detected.data || tag6.detected.data || tag7.detected.data || tag8.detected.data)
        {
            robot_hand_publisher.publish(hand);
            tag10_publisher.publish(marker1);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }    
    return 0;
}
