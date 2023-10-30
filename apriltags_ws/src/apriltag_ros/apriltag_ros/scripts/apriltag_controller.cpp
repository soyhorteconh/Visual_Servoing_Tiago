// libraries
#include <algorithm>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/RobotHand.h>
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <inria_utils/TransportValueUDP.hpp>
#include <inria_maths/FilterExponential.hpp>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <vector>

ros::Publisher marker_publisher;
apriltag_ros::AprilTagDetection tag10;
apriltag_ros::RobotHand robot_hand;

//------------------------------------------------------------------------------------------------
//PARAMETERS
//Time step
double dt = 0.0333;

//Stop distance before tag position
double stop_distance = 0.30;

//Controller
double kp = 0.5;
double max_velocity = 0.05;

//robot network
std::string host = "192.168.1.162";
//------------------------------------------------------------------------------------------------

//class apriltag controller
class Apriltag_Controller
{
    private:
        double kp;
        double max_velocity;

    public:
        Apriltag_Controller(float kp, float max_velocity):kp(kp), max_velocity(max_velocity){}

        // saturated proportional controller
        Eigen::Vector3d controller(Eigen::Vector3d target_position, Eigen::Vector3d real_position){
            
            // getting error
            Eigen::Vector3d error = target_position - real_position;

            // getting p controller
            Eigen::Vector3d control = kp * error;

            // getting norm
            double norm = control.norm();
            // std::cout << "----------------------------------" << std::endl;
            // std::cout << "Norm: " << norm << std::endl;
            norm = std::min(norm, max_velocity);

            control = norm * control.normalized();  

            // std::cout << "Controller final values " << std::endl;
            // std::cout << "x: " << control(0)<< std::endl;
            // std::cout << "y: " << control(1) << std::endl;
            // std::cout << "z: " << control(2) << std::endl;

            // std::cout << "RobotHand Position XYZ" << std::endl;
            // std::cout << "x: " << real_position(0)<< std::endl;
            // std::cout << "y: " << real_position(1) << std::endl;
            // std::cout << "z: " << real_position(2) << std::endl;
            // std::cout << "----------------------------------" << std::endl;

            return control;
        }
};

void tag10_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag10.coordinate_center = msg -> coordinate_center;
    tag10.orientation = msg -> orientation;
    tag10.detected = msg -> detected;
}

void robot_hand_cb(const apriltag_ros::RobotHand::ConstPtr &msg)
{
    robot_hand.pose = msg -> pose;
    robot_hand.detected = msg -> detected;
}

// publish mqrkers
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
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
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

Eigen::Quaterniond multiply_rotation_matrix(Eigen::Quaterniond q, float radians, int option)
{
    // option = 1 -> x rotation
    // option = 2 -> y rotation
    // option = 3 -> z rotation

    Eigen::Matrix3d orientation = q.normalized().toRotationMatrix();
    Eigen::Matrix3d rotation;
    if (option == 1){
        // x rotation
        rotation << 1, 0, 0,
                    0, std::cos(radians), -std::sin(radians),
                    0, std::sin(radians), std::cos(radians);
    } else if (option == 2){
        // y rotation
        rotation << std::cos(radians), 0, std::sin(radians),
                    0, 1, 0,
                    -1*std::sin(radians), 0, std::cos(radians);
    } else if (option == 3){
        // z rotation
        rotation << std::cos(radians), -std::sin(radians), 0,
                    std::sin(radians), std::cos(radians), 0,
                    0, 0, 1;
    }
    Eigen::Matrix3d m_r = orientation*rotation;
    Eigen::Quaterniond q_r(m_r);
    return q_r;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    // publishers
    ros::Publisher controller_publisher = n.advertise<geometry_msgs::Twist>("velocity_control", 1);
    marker_publisher = n.advertise<visualization_msgs::Marker>("markers", 0);
    ros::Publisher robot_hand_controller_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/controller", 1);
    ros::Publisher robot_hand_init_publisher = n.advertise<geometry_msgs::PoseStamped>("robot_hand/init", 1);
    ros::Publisher target_publisher = n.advertise<geometry_msgs::PoseStamped>("target", 1);
    ros::Publisher camera_publisher = n.advertise<geometry_msgs::PoseStamped>("camera/pose", 1);

    //subscribers
    ros::Subscriber robot_hand_subscriber = n.subscribe("robot_hand/pose", 1, robot_hand_cb); 
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/filter", 1, tag10_cb);

    // network
    inria::TransportValueUDPClient client;
    client.connect(host, 9997);

    // effectors configurations
    std::string prefix_vel_lin = "/tasks/seiko/gripper_tip_frame/cmd_vel_lin_";

    // acronyms
    // cf: camera frame
    // rhf: robot hand frame

    //robot hand Pose on cf
    Eigen::Matrix<double, 3, 1> robot_hand_position_cf{1.0,-1.5,1.0};
    Eigen::Quaterniond robot_hand_quaternion_cf(1.0, 0.0, 0.0, 0.0);
    Eigen::Matrix3d robot_hand_orientation_cf = robot_hand_quaternion_cf.toRotationMatrix();

    // target pose on cf
    Eigen::Matrix<double, 3, 1> tag_position_cf{1.5, -0.05, 2.0};
    Eigen::Quaterniond tag_quaternion_cf(1.0, 0.0, 0.0, 0.0);
    Eigen::Matrix3d tag_orientation_cf = tag_quaternion_cf.toRotationMatrix();

    //for simulations
    Eigen::Vector3d init(0.0, 0.0, 0.0);
    Eigen::Vector3d robot_hand_position_cf_sim (robot_hand_position_cf(0,0),robot_hand_position_cf(1,0),robot_hand_position_cf(2,0));
    Eigen::Vector3d tag_position_cf_sim(tag_position_cf(0,0), tag_position_cf(1,0), tag_position_cf(2,0));

    ros::Rate loop_rate(30);

    std::string start;
    std::cout << "PRESS any key to continue: ";
    std::cin >> start;

    bool flag = false;

    while(ros::ok())
    {
        // converting target pose on rhf
        // std::cout << "----------------------------------------------" << std::endl;
        // std::cout << "INIT" << std::endl;
        // std::cout << "TARGET" << std::endl << tag_position_cf_sim << std::endl;
        // std::cout << "START position" << std::endl << robot_hand_position_cf_sim << std::endl; 

        std::cout << "Flag: " << flag << std::endl;
        std::cout << "Robot Hand: " << robot_hand.detected;
        std::cout << "Tag: " << tag10.detected;

        // ---------------------------------------------------------------------------------------------------- //
        // detecting values from camera for first time
        if(robot_hand.detected.data == true and tag10.detected.data == true and flag == false){

            //robot hand pose
            robot_hand_position_cf(0,0) = robot_hand.pose.pose.position.x;
            robot_hand_position_cf(1,0) = robot_hand.pose.pose.position.y;
            robot_hand_position_cf(2,0) = robot_hand.pose.pose.position.z;

            robot_hand_quaternion_cf.x() = robot_hand.pose.pose.orientation.x;
            robot_hand_quaternion_cf.y() = robot_hand.pose.pose.orientation.y;
            robot_hand_quaternion_cf.z() = robot_hand.pose.pose.orientation.z;
            robot_hand_quaternion_cf.w() = robot_hand.pose.pose.orientation.w;

            robot_hand_orientation_cf = robot_hand_quaternion_cf.toRotationMatrix();

            //tag pose
            tag_position_cf(0,0) = tag10.coordinate_center.x;
            tag_position_cf(1,0) = tag10.coordinate_center.y;
            tag_position_cf(2,0) = tag10.coordinate_center.z;

            tag_quaternion_cf.x() = tag10.orientation.x;
            tag_quaternion_cf.y() = tag10.orientation.y;
            tag_quaternion_cf.z() = tag10.orientation.z;
            tag_quaternion_cf.w() = tag10.orientation.w;

            tag_orientation_cf = tag_quaternion_cf.toRotationMatrix();

            //converting values into vecotr
            robot_hand_position_cf_sim.x() = robot_hand_position_cf(0,0);
            robot_hand_position_cf_sim.y() = robot_hand_position_cf(1,0);
            robot_hand_position_cf_sim.z() = robot_hand_position_cf(2,0);

            tag_position_cf_sim.x() = tag_position_cf(0,0);
            tag_position_cf_sim.y() = tag_position_cf(1,0);
            tag_position_cf_sim.z() = tag_position_cf(2,0);

            flag = true;
        }

        if(flag == true)
        {
            std::cout << "--------------------------------" << std::endl;
            std::cout << "TAG position" << std::endl;
            std::cout << "x: " << tag10.coordinate_center.x << std::endl;
            std::cout << "y: " << tag10.coordinate_center.y << std::endl;
            std::cout << "z: " << tag10.coordinate_center.z << std::endl;

            std::cout << "ROBOT HAND initial position" << std::endl;
            std::cout << "x: " << robot_hand_position_cf_sim.x() << std::endl;
            std::cout << "y: " << robot_hand_position_cf_sim.y() << std::endl;
            std::cout << "z: " << robot_hand_position_cf_sim.z() << std::endl;

            //update tag position
            tag_position_cf(0,0) = tag10.coordinate_center.x;
            tag_position_cf(1,0) = tag10.coordinate_center.y;
            tag_position_cf(2,0) = tag10.coordinate_center.z;

            tag_quaternion_cf.x() = tag10.orientation.x;
            tag_quaternion_cf.y() = tag10.orientation.y;
            tag_quaternion_cf.z() = tag10.orientation.z;
            tag_quaternion_cf.w() = tag10.orientation.w;

            tag_orientation_cf = tag_quaternion_cf.toRotationMatrix();

            // //STOP 15 cm before on z 
            Eigen::Matrix<double,3,1> p_next{0, 0.0, stop_distance};
            Eigen::Matrix<double,3,1> new_tag_position = tag_orientation_cf*p_next + tag_position_cf;
            std::cout << "TARGET position" << std::endl << new_tag_position << std::endl;

            // getting target on rhf
            Eigen::Matrix<double, 3, 1> target_position_rhf = robot_hand_orientation_cf.transpose() * (new_tag_position - robot_hand_position_cf);
            Eigen::Vector3d target_position_rhf_v(target_position_rhf(0,0), target_position_rhf(1,0), target_position_rhf(2,0));
            //std::cout << "Target position: " << std::endl << target_position_rhf << std::endl;

            // ---------------------------------------------------------------------------------------------------- //
            // CONTROLLER
            // controller initialized
            Apriltag_Controller apriltag_controller(kp, max_velocity);

            // initial position on rhf
            Eigen::Vector3d initial_position_rhf(0.0, 0.0, 0.0);
            
            // this is what i'm going to send to the tiago robot
            Eigen::Vector3d robot_hand_velocity_rhf;
            if (robot_hand.detected.data == true and tag10.detected.data == true)
            {
                robot_hand_velocity_rhf = apriltag_controller.controller(target_position_rhf_v, initial_position_rhf);
            } 
            else
            {
                robot_hand_velocity_rhf.x() = 0.0;
                robot_hand_velocity_rhf.y() = 0.0;
                robot_hand_velocity_rhf.z() = 0.0;
            }
            
            client.setFloat(prefix_vel_lin+"x", robot_hand_velocity_rhf.x());
            client.setFloat(prefix_vel_lin+"y", robot_hand_velocity_rhf.y());
            client.setFloat(prefix_vel_lin+"z", robot_hand_velocity_rhf.z());
            client.send();

            std::cout << "****VELOCITY" << std::endl;
            std::cout << "x velocity: " << robot_hand_velocity_rhf.x() << std::endl;
            std::cout << "y velocity: " << robot_hand_velocity_rhf.y() << std::endl;
            std::cout  << "z velocity: " << robot_hand_velocity_rhf.z() << std::endl;
            // ---------------------------------------------------------------------------------------------------- //
            // TRANSFORMATION MATRIX for new position
            // updating position
            target_position_rhf_v = target_position_rhf_v - robot_hand_velocity_rhf * dt;
            target_position_rhf(0,0) = target_position_rhf_v.x();
            target_position_rhf(1,0) = target_position_rhf_v.y();
            target_position_rhf(2,0) = target_position_rhf_v.z();

            // ESTIMATED POSITION (open loop controller)
            Eigen::Matrix<double, 3, 1> predicted_position = - robot_hand_orientation_cf.transpose().inverse() * target_position_rhf + new_tag_position;

            // REAL POSITION (feedback)
            Eigen::Matrix<double, 3, 1> real_position(robot_hand.pose.pose.position.x, robot_hand.pose.pose.position.y, robot_hand.pose.pose.position.z);

            // COMPLEMENTARY FILTER (kalman filter)
            //asign a valid value between 0 and 1
            inria::FilterExponential<Eigen::Vector3d> filter_alpha_value;
            double alpha = filter_alpha_value.getAlphaFromFreq(0.1, dt);
            std::cout << "****Alpha value: " << alpha << std::endl;

            robot_hand_position_cf = alpha * predicted_position + (1 -  alpha) * predicted_position;
            std::cout << "robot hand position" << std::endl << robot_hand_position_cf << std::endl;

            // ---------------------------------------------------------------------------------------------------- //
            // SIMULATION
            //initial position
            geometry_msgs::PoseStamped rh_init;
            rh_init.header.seq = 1;
            rh_init.header.stamp = ros::Time::now();
            rh_init.header.frame_id = "map";
            rh_init.pose.position.x = robot_hand_position_cf_sim.x();
            rh_init.pose.position.y = robot_hand_position_cf_sim.y();
            rh_init.pose.position.z = robot_hand_position_cf_sim.z();
            rh_init.pose.orientation.x = robot_hand_quaternion_cf.x();
            rh_init.pose.orientation.y = robot_hand_quaternion_cf.y();
            rh_init.pose.orientation.z = robot_hand_quaternion_cf.z();
            rh_init.pose.orientation.w = robot_hand_quaternion_cf.w();
            robot_hand_init_publisher.publish(rh_init);

            //controller position
            geometry_msgs::PoseStamped rh_controller;
            rh_controller.header.seq = 1;
            rh_controller.header.stamp = ros::Time::now();
            rh_controller.header.frame_id = "map";
            rh_controller.pose.position.x = robot_hand_position_cf.x();
            rh_controller.pose.position.y = robot_hand_position_cf.y();
            rh_controller.pose.position.z = robot_hand_position_cf.z();
            rh_controller.pose.orientation.x = robot_hand_quaternion_cf.x();
            rh_controller.pose.orientation.y = robot_hand_quaternion_cf.y();
            rh_controller.pose.orientation.z = robot_hand_quaternion_cf.z();
            rh_controller.pose.orientation.w = robot_hand_quaternion_cf.w();
            robot_hand_controller_publisher.publish(rh_controller);

            //target position
            geometry_msgs::PoseStamped target;
            target.header.seq = 1;
            target.header.stamp = ros::Time::now();
            target.header.frame_id = "map";
            target.pose.position.x = tag_position_cf.x();
            target.pose.position.y = tag_position_cf.y();
            target.pose.position.z = tag_position_cf.z();
            target.pose.orientation.x = tag_quaternion_cf.x();
            target.pose.orientation.y = tag_quaternion_cf.y();
            target.pose.orientation.z = tag_quaternion_cf.z();
            target.pose.orientation.w = tag_quaternion_cf.w();
            target_publisher.publish(target);

            //camera position
            Eigen::Quaterniond camera_q(1,0,0,0);
            Eigen::Quaterniond camera_orientation = multiply_rotation_matrix(camera_q, -M_PI/2, 3);

            geometry_msgs::PoseStamped camera;
            camera.header.seq = 1;
            camera.header.stamp = ros::Time::now();
            camera.header.frame_id = "map";
            camera.pose.position.x = 0;
            camera.pose.position.y = 0;
            camera.pose.position.z = 0;
            camera.pose.orientation.x = camera_orientation.x();
            camera.pose.orientation.y = camera_orientation.y();
            camera.pose.orientation.z = camera_orientation.z();
            camera.pose.orientation.w = camera_orientation.w();
            camera_publisher.publish(camera);

            //  target
            publish_marker("target", 1, "blue", tag_position_cf, tag_quaternion_cf); 
        }
        std::cout << "--------------------------------" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
