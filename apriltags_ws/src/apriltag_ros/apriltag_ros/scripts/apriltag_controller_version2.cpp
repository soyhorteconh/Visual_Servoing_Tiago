/*
Hortencia Alejandra Ramirez Vazquez
23rd november, 2023
*/

#include <apriltag_ros/VelocityControl.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/RobotHand.h>
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <inria_maths/AxisAngle.h>
#include <inria_utils/TransportValueUDP.hpp>
#include <ros/ros.h>
#include <string>
#include <utility>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_publisher;
apriltag_ros::AprilTagDetection tag10_detected;
apriltag_ros::AprilTagDetection tag20_detected;
apriltag_ros::RobotHand robot_hand_detected;

// callbacks
void tag10_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag10_detected.coordinate_center = msg -> coordinate_center;
    tag10_detected.orientation = msg -> orientation;
    tag10_detected.detected = msg -> detected;
}

void tag20_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag20_detected.coordinate_center = msg -> coordinate_center;
    tag20_detected.orientation = msg -> orientation;
    tag20_detected.detected = msg -> detected;
}

void robot_hand_cb(const apriltag_ros::RobotHand::ConstPtr &msg)
{
    robot_hand_detected.pose = msg -> pose;
    robot_hand_detected.detected = msg -> detected;
}

// rotation matrixes
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

// to get target point from tag point
std::pair<Eigen::Vector3d,Eigen::Matrix3d> tag_2_targetpoint(Eigen::Vector3d position, Eigen::Matrix3d rotation, Eigen::Vector3d distance2tag)
{
    Eigen::Matrix3d target_rotation = rotation * y_rotation(M_PI/2);
    Eigen::Vector3d target_position = position + rotation * distance2tag;

    std::pair<Eigen::Vector3d,Eigen::Matrix3d> target_pose(target_position, target_rotation);

    return target_pose;
}

// publishing markers
void publish_marker(std::string ns, int id, std::string color, Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag10.dae";
    marker.mesh_use_embedded_materials = true;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.25;
    marker.color.a = 1.0; //alpha

    if (color == "red") {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else if (color == "blue"){
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    } else if (color == "green"){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    marker_publisher.publish(marker);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "velocity_control");
    ros::NodeHandle n;

    // publishers
    ros::Publisher robot_hand_controller_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/controller", 1);
    ros::Publisher tag_publisher = n.advertise<geometry_msgs::PoseStamped>("tag/pose", 1);
    marker_publisher = n.advertise<visualization_msgs::Marker>("markers", 0);
    ros::Publisher camera_publisher = n.advertise<geometry_msgs::PoseStamped>("camera/pose", 1);

    // for help
    ros::Publisher controller_error = n.advertise<geometry_msgs::Pose>("controller/error", 1);
    ros::Publisher controller_velocity = n.advertise<geometry_msgs::Twist>("controller/velocity", 1);
    ros::Publisher controller_estimatedPose = n.advertise<geometry_msgs::Pose>("controller/estimated_pose", 1);

    // subscribers
    ros::Subscriber robot_hand_subscriber = n.subscribe("robot_hand/pose", 1, robot_hand_cb); 
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/filter", 1, tag10_cb);
    ros::Subscriber tag20_subscriber = n.subscribe("tag20/filter", 1, tag20_cb);

    // variables
    bool flag = false;

    Eigen::Vector3d robot_hand_position;
    Eigen::Matrix3d robot_hand_rotation;
    Eigen::Quaterniond rh_q;

    Eigen::Vector3d tag_position;
    Eigen::Matrix3d tag_rotation;

    // robot connection
    // network
    std::string host = "192.168.1.162";
    inria::TransportValueUDPClient client;
    client.connect(host, 9997);
    // effectors configurations
    std::string prefix_vel_lin = "/tasks/seiko/gripper_tip_frame/cmd_vel_lin_";
    std::string prefix_vel_ang = "/tasks/seiko/gripper_tip_frame/cmd_vel_ang_";

    // state machine variables
    int reset = 0;
    int state = 0;
    float gripper = 0.0; 
    double angular_error;
    double linear_error;

    // time 
    double third_state = ros::Time::now().toSec();

    // to start simulation 
    std::string start;
    std::cout << "PRESS any key to continue: ";
    std::cin >> start;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        std::cout << "----------------------------" << std::endl;

        double distance_open_drawer = tag10_detected.coordinate_center.z - tag20_detected.coordinate_center.z;
        std::cout << "Drawer openning distance: " << distance_open_drawer << std::endl;

        double distance_hand_robot = tag10_detected.coordinate_center.z - robot_hand_detected.pose.pose.position.z;
        std::cout << "Robot hand distance: " << distance_hand_robot;
        //std::cout << "real distance: " << tag10_detected.coordinate_center.z << std::endl;
        //double new_distance = tag10_detected.coordinate_center.z - distance_open_drawer;
        //std::cout << "new distance: " << new_distance << std::endl;

        // first time detected
        if (flag == false and robot_hand_detected.detected.data == true and tag10_detected.detected.data == true)
        {
            // robot hand 
            //      position
            robot_hand_position.x() = robot_hand_detected.pose.pose.position.x;
            robot_hand_position.y() = robot_hand_detected.pose.pose.position.y;
            robot_hand_position.z() = robot_hand_detected.pose.pose.position.z;
            //      orientation
            rh_q = Eigen::Quaterniond(robot_hand_detected.pose.pose.orientation.w, robot_hand_detected.pose.pose.orientation.x, robot_hand_detected.pose.pose.orientation.y, robot_hand_detected.pose.pose.orientation.z);
            robot_hand_rotation = rh_q.normalized().toRotationMatrix();

            // tag information
            //      position
            tag_position.x() = tag10_detected.coordinate_center.x;
            tag_position.y() = tag10_detected.coordinate_center.y;
            tag_position.z() = tag10_detected.coordinate_center.z - distance_open_drawer;
            //      orientation
            Eigen::Quaterniond t_q (tag10_detected.orientation.w, tag10_detected.orientation.x, tag10_detected.orientation.y, tag10_detected.orientation.z);
            tag_rotation = t_q.toRotationMatrix();

            flag = true;
        }

        if(flag == true)
        {

            // updating tag information
            //      position
            tag_position.x() = tag10_detected.coordinate_center.x;
            tag_position.y() = tag10_detected.coordinate_center.y;
            tag_position.z() = tag10_detected.coordinate_center.z;
            //      orientation
            Eigen::Quaterniond t_q(tag10_detected.orientation.w, tag10_detected.orientation.x, tag10_detected.orientation.y, tag10_detected.orientation.z);
            tag_rotation = t_q.normalized().toRotationMatrix();

            // calculating target pose from tag pose
            Eigen::Vector3d distance2tag;
            if (state == 0)
            {
                // bottle configuration
                // distance2tag.x() = -0.18;
                // distance2tag.y() = 0.01;
                // distance2tag.z() = 0.35;

                // dish washer configuration
                distance2tag.x() = 0.03;
                distance2tag.y() = -0.15;
                distance2tag.z() = 0.35;
            }
            else if (state == 1)
            {
                //bottle configuration
                // distance2tag.x() = -0.18;
                // distance2tag.y() = 0.01;
                // distance2tag.z() = 0.07;

                // dish washer configuration
                distance2tag.x() = 0.0;
                distance2tag.y() = -0.16;
                distance2tag.z() = 0.12;
            }
            else if (state == 3)
            {
                // dish washer configuration
                distance2tag.x() = 0.0;
                distance2tag.y() = -0.11;
                distance2tag.z() = 0.62;
            }
            //newwwww
            else if (state == 4)
            {
                // dish washer configuration
                distance2tag.x() = -0.15;
                distance2tag.y() = -0.03;
                distance2tag.z() = 0.62;
            }
            //newwww
            else if (state == 5)
            {
                // dish washer configuration
                distance2tag.x() = 0.0;
                distance2tag.y() = -0.11;
                distance2tag.z() = 0.62;
            }
            else if (state == 6)
            {
                // dish washer configuration
                distance2tag.x() = 0.0;
                distance2tag.y() = -0.13;
                distance2tag.z() = 0.5;
            }
            else if (state == 7)
            {
                
                // dish washer configuration
                distance2tag.x() = 0.0;
                distance2tag.y() = -0.16;
                distance2tag.z() = 0.13;
            }
            else if (state == 8)
            {
                // dish washer configuration
                distance2tag.x() = 0.03;
                distance2tag.y() = -0.14;
                distance2tag.z() = 0.35;
            }
            else if (state == 9)
            {
                // dish washer configuration
                distance2tag.x() = -0.14;
                distance2tag.y() = -0.1;
                distance2tag.z() = 0.45;
            }


            std::pair<Eigen::Vector3d, Eigen::Matrix3d> target_point = tag_2_targetpoint (
                tag_position, tag_rotation, distance2tag);

            Eigen::Matrix3d target_rotation = target_point.second;
            Eigen::Quaterniond target_rotation_q (target_rotation);
            Eigen::Vector3d target_position = target_point.first;

            // velocity controller parameters
            double linear_kp = 0.3;
            double angular_kp = 0.4;
            double linear_max_velocity = 0.04;
            double angular_max_velocity = 0.1;
            double dt = 0.0333;

            Eigen::Vector3d linear_v;
            Eigen::Vector3d angular_v;
            // hand open

            Eigen::Vector3d position_error;
            Eigen::Quaterniond rotation_error;

            Eigen::Matrix3d estimated_rotation;
            Eigen::Vector3d estimated_position;

            double time = ros::Time::now().toSec() - third_state;
            std::cout << "Time: " << time << std::endl;

            // action states
            // first and second state 
            //controller
            if (robot_hand_detected.detected.data == true and tag10_detected.detected.data == true)
            {

                // controller
                VelocityControl velocity_control(dt, linear_kp, angular_kp, linear_max_velocity, angular_max_velocity);
                std::pair <Eigen::Vector3d, Eigen::Vector3d> velocity = velocity_control.controller(
                    target_position, target_rotation, robot_hand_position, robot_hand_rotation);

                position_error = velocity_control.get_position_error();
                rotation_error = Eigen::Quaterniond(velocity_control.get_rotation_error());
                
                //velocity
                linear_v = velocity.first;
                angular_v = velocity.second;
                //linear_v.setZero();
                //angular_v.setZero();
                
                // getting estimated values
                estimated_rotation = velocity_control.get_estimated_rotation();
                estimated_position = velocity_control.get_estimated_position();
                //std::cout << "estimated_rotation" << std::endl << estimated_rotation << std::endl;

                // getting messured robot hand pose
                Eigen::Vector3d measured_position(robot_hand_detected.pose.pose.position.x, robot_hand_detected.pose.pose.position.y, robot_hand_detected.pose.pose.position.z);
                Eigen::Quaterniond measured_q(robot_hand_detected.pose.pose.orientation.w, robot_hand_detected.pose.pose.orientation.x, robot_hand_detected.pose.pose.orientation.y, robot_hand_detected.pose.pose.orientation.z);
                Eigen::Matrix3d measured_rotation = measured_q.normalized().toRotationMatrix();

                // feedback
                // (aqui)
                robot_hand_position = velocity_control.get_predicted_position(measured_position, 0.1);
                rh_q = velocity_control.get_predicted_rotation(measured_rotation, 0.5);
                robot_hand_rotation = rh_q.normalized().toRotationMatrix();

            } 
            
            
            if (state == 2 and time > 1 and distance_open_drawer < 0.18)
            {
                linear_v.x() = -0.04;
                linear_v.y() = 0.02;
                linear_v.z() = 0.001;
                //angular_v.setZero();
            } 
            else if (state == 3 and time < 1.5)
            {
                linear_v.setZero();
                angular_v.setZero();
                gripper = 0.0;
            }
            // else if (state == 5 and time < 1)
            // {
            //     // linear_v.x() = 0.04;
            //     // linear_v.y() = 0.0;
            //     // linear_v.z() = 0.003;
            //     linear_v.setZero();
            //     angular_v.setZero();
            // }
            else if (state == 10)
            {
                linear_v.setZero();
                angular_v.setZero();
            }

            angular_error = inria::MatrixToAxis(rotation_error.toRotationMatrix()).norm();
            linear_error = position_error.norm();

            // transitions states
            // second state (transition)
            if(state == 0 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 1;
            } 
            // third state (transition)
            else if (state == 1 and angular_error < 0.045 and linear_error < 0.01)
            {
                // hand close
                //std::cout << "Gripper has closed" << std::endl;
                gripper = 1.0;
                state = 2;
                third_state = ros::Time::now().toSec();
            }
            else if (state == 2 and distance_open_drawer > 0.18)
            {
                state = 3;
                third_state = ros::Time::now().toSec();
            }
            else if (state == 3 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 4;
            }
            else if (state == 4 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 5;
            }
            else if (state == 5 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 6;
            }
            else if (state == 6 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 7;
                //gripper = 1.0;
                third_state = ros::Time::now().toSec();
            }
            else if (state == 7 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 8;
            }
            else if (state == 8 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 9;
            }
            else if (state == 9 and angular_error < 0.045 and linear_error < 0.01)
            {
                state = 10;
            }


            std::cout << "Linear error: " << linear_error << std::endl;
            std::cout << "Angular error: " <<  angular_error << std::endl;
            // std::cout << "Gripper: " << gripper << std::endl;

            std::cout << "STATE**: " << state << std::endl;
            Eigen::Quaterniond estimated_rotation_q(estimated_rotation);
            geometry_msgs::Pose error;
            error.position.x = position_error.x();
            error.position.y = position_error.y();
            error.position.z = position_error.z();
            error.orientation.x = rotation_error.x();
            error.orientation.y = rotation_error.y();
            error.orientation.z = rotation_error.z();
            error.orientation.w = rotation_error.w();
            controller_error.publish(error);

            geometry_msgs::Twist velocity_msg;
            velocity_msg.linear.y = linear_v.y();
            velocity_msg.linear.z = linear_v.z();
            velocity_msg.linear.x = linear_v.x();
            velocity_msg.angular.x = angular_v.x();
            velocity_msg.angular.y = angular_v.y();
            velocity_msg.angular.z = angular_v.z();
            controller_velocity.publish(velocity_msg);

            geometry_msgs::Pose estimated_pose;
            estimated_pose.position.x = estimated_position.x();
            estimated_pose.position.y = estimated_position.y();
            estimated_pose.position.z = estimated_position.z();
            estimated_pose.orientation.x = estimated_rotation_q.x();
            estimated_pose.orientation.y = estimated_rotation_q.y();
            estimated_pose.orientation.z = estimated_rotation_q.z();
            estimated_pose.orientation.w = estimated_rotation_q.w();
            controller_estimatedPose.publish(estimated_pose);

            std::cout << "Linear vel" << std::endl;
            std::cout << "x: " << linear_v.x() << std::endl;
            std::cout << "y: " << linear_v.y() << std::endl;
            std::cout << "z: " << linear_v.z() << std::endl;
            std::cout << "Angular vel" << std::endl;
            std::cout << "x: " << angular_v.x() << std::endl;
            std::cout << "y: " << angular_v.y() << std::endl;
            std::cout << "z: " << angular_v.z() << std::endl;
            
            // sending velocity to Tiago
            client.setFloat(prefix_vel_lin+"x", linear_v.x());
            client.setFloat(prefix_vel_lin+"y", linear_v.y());
            client.setFloat(prefix_vel_lin+"z", linear_v.z());
            client.setFloat(prefix_vel_ang+"x", angular_v.x());
            client.setFloat(prefix_vel_ang+"y", angular_v.y());
            client.setFloat(prefix_vel_ang+"z", angular_v.z());
            client.setFloat("/hand/pos_ratio", gripper);
            client.send();
            // (aqui)

            // simulation
            //      robot hand controller
            geometry_msgs::PoseStamped rh_controller;
            rh_controller.header.seq = 1;
            rh_controller.header.stamp = ros::Time::now();
            rh_controller.header.frame_id = "map";
            rh_controller.pose.position.x = robot_hand_position.x();
            rh_controller.pose.position.y = robot_hand_position.y();
            rh_controller.pose.position.z = robot_hand_position.z();
            rh_controller.pose.orientation.x = rh_q.x();
            rh_controller.pose.orientation.y = rh_q.y();
            rh_controller.pose.orientation.z = rh_q.z();
            rh_controller.pose.orientation.w = rh_q.w();
            robot_hand_controller_publisher.publish(rh_controller);

            //  tag
            publish_marker("target", 1, "blue", tag_position, t_q); 

            // target
            geometry_msgs::PoseStamped target;
            target.header.seq = 1;
            target.header.stamp = ros::Time::now();
            target.header.frame_id = "map";
            target.pose.position.x = target_position.x();
            target.pose.position.y = target_position.y();
            target.pose.position.z = target_position.z();
            target.pose.orientation.x = target_rotation_q.x();
            target.pose.orientation.y = target_rotation_q.y();
            target.pose.orientation.z = target_rotation_q.z();
            target.pose.orientation.w = target_rotation_q.w();
            tag_publisher.publish(target);

            //camera
            Eigen::Quaterniond camera_q(1,0,0,0);
            Eigen::Matrix3d camera_rotation = camera_q.toRotationMatrix();
            camera_rotation = camera_rotation * z_rotation(-M_PI/2);
            camera_q = Eigen::Quaterniond(camera_rotation);

            geometry_msgs::PoseStamped camera;
            camera.header.seq = 1;
            camera.header.stamp = ros::Time::now();
            camera.header.frame_id = "map";
            camera.pose.position.x = 0;
            camera.pose.position.y = 0;
            camera.pose.position.z = 0;
            camera.pose.orientation.x = camera_q.x();
            camera.pose.orientation.y = camera_q.y();
            camera.pose.orientation.z = camera_q.z();
            camera.pose.orientation.w = camera_q.w();
            camera_publisher.publish(camera);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}