#include <ros/ros.h>
#include <iostream>
#include <string>
#include <inria_utils/TransportValueUDP.hpp>
#include "apriltag_ros/AprilTagDetection.h"

apriltag_ros::AprilTagDetection tag5;

// callbacks
void tag5_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag5.coordinate_center = msg -> coordinate_center;
    tag5.orientation = msg -> orientation;
    tag5.detected = msg -> detected;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    // subscribers
    ros::Subscriber sub_tag5 = n.subscribe("tag5_filter", 1, tag5_cb);

    // network
    inria::TransportValueUDPClient client;
    std::string host = "192.168.1.162"; // set a valid host
    client.connect(host, 9997);

    // effectors configurations
    std::string prefix_vel_lin = "/tasks/seiko/gripper_tip_frame/cmd_vel_lin_";

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        if(tag5.detected.data)
        {
            client.setFloat(prefix_vel_lin+"x", 0.05);
            std::cout << "Detected" << std::endl;
        } else 
        {
            client.setFloat(prefix_vel_lin+"x", 0.0);
            std::cout << "Not detected" << std::endl;
        }

        client.send();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

