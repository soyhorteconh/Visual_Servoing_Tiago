#include <apriltag_ros/VelocityControl.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/RobotHand.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <utility>

apriltag_ros::AprilTagDetection tag10_detected;
apriltag_ros::RobotHand robot_hand_detected;

// callbacks
void tag10_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag10_detected.coordinate_center = msg -> coordinate_center;
    tag10_detected.orientation = msg -> orientation;
    tag10_detected.detected = msg -> detected;
}

void robot_hand_cb(const apriltag_ros::RobotHand::ConstPtr &msg)
{
    robot_hand_detected.pose = msg -> pose;
    robot_hand_detected.detected = msg -> detected;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "velocity_control");
    ros::NodeHandle n;

    // publishers
    ros::Publisher robot_hand_controller_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/controller", 1);
    ros::Publisher robot_hand_init_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/init", 1);
    ros::Publisher tag_publisher = n.advertise<geometry_msgs::PoseStamped>("tag/pose", 1);
    ros::Publisher camera_publisher = n.advertise<geometry_msgs::PoseStamped>("camera/pose", 1);

    // subscribers
    ros::Subscriber robot_hand_subscriber = n.subscribe("robot_hand/pose", 1, robot_hand_cb); 
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/filter", 1, tag10_cb);

    // variables
    bool flag = false;

    Eigen::Vector3d robot_hand_position;
    Eigen::Matrix3d robot_hand_rotation;

    Eigen::Vector3d tag_position;
    Eigen::Matrix3d tag_rotation;

    int count = 0;

    std::string start;
    std::cout << "PRESS any key to continue: ";
    std::cin >> start;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        // std::cout << "Flag: " << flag << std::endl;
        // std::cout << "Robot Hand: " << robot_hand_detected.detected;
        // std::cout << "Tag: " << tag10_detected.detected;

        // first time detected
        if (flag == false and robot_hand_detected.detected.data == true and tag10_detected.detected.data == true)
        {
            // robot hand 
            //      position
            robot_hand_position.x() = robot_hand_detected.pose.pose.position.x;
            robot_hand_position.y() = robot_hand_detected.pose.pose.position.y;
            robot_hand_position.z() = robot_hand_detected.pose.pose.position.z;
            //      orientation
            Eigen::Quaterniond rh_q (robot_hand_detected.pose.pose.orientation.w, robot_hand_detected.pose.pose.orientation.x, robot_hand_detected.pose.pose.orientation.y, robot_hand_detected.pose.pose.orientation.z);
            robot_hand_rotation = rh_q.toRotationMatrix();

            // tag information
            //      position
            tag_position.x() = tag10_detected.coordinate_center.x;
            tag_position.y() = tag10_detected.coordinate_center.y;
            tag_position.z() = tag10_detected.coordinate_center.z;
            //      orientation
            Eigen::Quaterniond t_q (tag10_detected.orientation.w, tag10_detected.orientation.x, tag10_detected.orientation.y, tag10_detected.orientation.z);
            tag_rotation = t_q.toRotationMatrix();

            flag = true;
        }

        if(flag == true)
        {

            std::cout << "----------------------------" << std::endl;
            std::cout << count << std::endl;
            std::cout << "Tag position " << std::endl << tag_position << std::endl;
            std::cout << "Tag rotation " << std::endl << tag_rotation << std::endl;
            std::cout << "Robot Hand position " << std::endl << robot_hand_position << std::endl;
            std::cout << "Robot Hand rotation " << std::endl << robot_hand_rotation << std::endl;
            std::cout << "----------------------------" << std::endl;

            count += 1;
            // updating tag information
            //      position
            tag_position.x() = tag10_detected.coordinate_center.x;
            tag_position.y() = tag10_detected.coordinate_center.y;
            tag_position.z() = tag10_detected.coordinate_center.z;
            //      orientation
            Eigen::Quaterniond t_q (tag10_detected.orientation.w, tag10_detected.orientation.x, tag10_detected.orientation.y, tag10_detected.orientation.z);
            tag_rotation = t_q.toRotationMatrix();

            // parameters
            double kp = 0.3;
            double linear_max_velocity = 0.05;
            double angular_max_velocity = 0.05;
            double dt = 0.0333;

            // controller
            VelocityControl velocity_control(kp, dt, linear_max_velocity, angular_max_velocity);
            std::pair <Eigen::Vector3d, Eigen::Vector3d> velocity = velocity_control.controller(
                tag_position, tag_rotation, robot_hand_position, robot_hand_rotation);
            
            Eigen::Matrix3d estimated_rotation = velocity_control.get_estimated_rotation();
            //Eigen::Quaterniond estimated_rotation_q(estimated_rotation);
            Eigen::Vector3d estimated_position = velocity_control.get_estimated_position();

            // feedback
            robot_hand_position = estimated_position;
            robot_hand_rotation = estimated_rotation;
            Eigen::Quaterniond rh_q(robot_hand_rotation);

            // simulation
            //      robot hand controller
            geometry_msgs::PoseStamped rh_controller;
            rh_controller.header.seq = 1;
            rh_controller.header.stamp = ros::Time::now();
            rh_controller.header.frame_id = "map";
            rh_controller.pose.position.x = estimated_position.x();
            rh_controller.pose.position.y = estimated_position.y();
            rh_controller.pose.position.z = estimated_position.z();
            rh_controller.pose.orientation.x = rh_q.x();
            rh_controller.pose.orientation.y = rh_q.y();
            rh_controller.pose.orientation.z = rh_q.z();
            rh_controller.pose.orientation.w = rh_q.w();
            robot_hand_controller_publisher.publish(rh_controller);
            
            //      tag
            geometry_msgs::PoseStamped tag;
            tag.header.seq = 1;
            tag.header.stamp = ros::Time::now();
            tag.header.frame_id = "map";
            tag.pose.position.x = tag_position.x();
            tag.pose.position.y = tag_position.y();
            tag.pose.position.z = tag_position.z();
            tag.pose.orientation.x = t_q.x();
            tag.pose.orientation.y = t_q.y();
            tag.pose.orientation.z = t_q.z();
            tag.pose.orientation.w = t_q.w();
            tag_publisher.publish(tag);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}