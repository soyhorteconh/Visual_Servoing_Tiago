// libraries
#include <algorithm>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/RobotHand.h>
#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <inria_maths/AxisAngle.h>
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
double kp = 0.1;
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
        std::vector<Eigen::Vector3d> controller(Eigen::Vector3d target_position, Eigen::Vector3d target_orientation, Eigen::Vector3d real_position, Eigen::Vector3d real_orientation){
            
            // position error and controller
            Eigen::Vector3d position_error = target_position - real_position;
            Eigen::Vector3d position_control = kp * position_error;

            // orientation error and controller
            Eigen::Vector3d orientation_error = target_orientation - real_orientation;
            Eigen::Vector3d orientation_control = kp * orientation_error;

            // saturation
            double position_norm = position_control.norm();
            position_norm = std::min(position_norm, max_velocity);
            position_control = position_norm * position_control.normalized();  

            double orientation_norm = orientation_control.norm();
            orientation_norm = std::min(orientation_norm, max_velocity);
            orientation_control = orientation_norm * orientation_control.normalized();

            std::vector<Eigen::Vector3d> control;
            control.push_back(position_control);
            control.push_back(orientation_control);

            return control;
        }
};

// callbacks
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
    std::string prefix_vel_ang = "/tasks/seiko/gripper_tip_frame/cmd_vel_ang_";

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
    Eigen::Vector3d robot_hand_position_cf_sim (robot_hand_position_cf(0,0),robot_hand_position_cf(1,0),robot_hand_position_cf(2,0));
    Eigen::Quaterniond robot_hand_quaternion_cf_sim (1,0,0,0);
    Eigen::Matrix3d robot_hand_rm_cf_sim = robot_hand_quaternion_cf_sim.toRotationMatrix();

    ros::Rate loop_rate(30);

    std::string start;
    std::cout << "PRESS any key to continue: ";
    std::cin >> start;

    bool flag = false;

    while(ros::ok())
    {

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

            //Simulation values
            robot_hand_position_cf_sim.x() = robot_hand_position_cf(0,0);
            robot_hand_position_cf_sim.y() = robot_hand_position_cf(1,0);
            robot_hand_position_cf_sim.z() = robot_hand_position_cf(2,0);

            robot_hand_quaternion_cf_sim.x() = robot_hand_quaternion_cf.x();
            robot_hand_quaternion_cf_sim.y() = robot_hand_quaternion_cf.y();
            robot_hand_quaternion_cf_sim.z() = robot_hand_quaternion_cf.z();
            robot_hand_quaternion_cf_sim.w() = robot_hand_quaternion_cf.w();


            flag = true;
        }

        if(flag == true)
        {
            std::cout << "--------------------------------" << std::endl;
            std::cout << "TAG position" << std::endl;
            std::cout << "x: " << tag10.coordinate_center.x << std::endl;
            std::cout << "y: " << tag10.coordinate_center.y << std::endl;
            std::cout << "z: " << tag10.coordinate_center.z << std::endl;
            std::cout << "TAG orientation" << std::endl;
            std::cout << "x: " << tag10.orientation.x << std::endl;
            std::cout << "y: " << tag10.orientation.y << std::endl;
            std::cout << "z: " << tag10.orientation.z << std::endl;
            std::cout << "w: " << tag10.orientation.w << std::endl;

            std::cout << "ROBOT HAND initial position" << std::endl;
            std::cout << "x: " << robot_hand_position_cf_sim.x() << std::endl;
            std::cout << "y: " << robot_hand_position_cf_sim.y() << std::endl;
            std::cout << "z: " << robot_hand_position_cf_sim.z() << std::endl;

            //update position and orientation
            tag_position_cf(0,0) = tag10.coordinate_center.x;
            tag_position_cf(1,0) = tag10.coordinate_center.y;
            tag_position_cf(2,0) = tag10.coordinate_center.z;

            tag_quaternion_cf.x() = tag10.orientation.x;
            tag_quaternion_cf.y() = tag10.orientation.y;
            tag_quaternion_cf.z() = tag10.orientation.z;
            tag_quaternion_cf.w() = tag10.orientation.w;
            tag_orientation_cf = tag_quaternion_cf.toRotationMatrix();

            //robot_hand_orientation_cf = robot_hand_quaternion_cf.toRotationMatrix();

            // stops before the target
            Eigen::Matrix<double,3,1> p_next{0, 0.0, stop_distance};
            Eigen::Matrix<double,3,1> new_tag_position = tag_orientation_cf*p_next + tag_position_cf;
            std::cout << "TARGET position" << std::endl << new_tag_position << std::endl;

            // target on rhf
            //     position
            Eigen::Matrix<double, 3, 1> target_position_rhf = robot_hand_orientation_cf.transpose() * (new_tag_position - robot_hand_position_cf);
            Eigen::Vector3d target_position_rhf_v(target_position_rhf(0,0), target_position_rhf(1,0), target_position_rhf(2,0));
            //      orientation
            Eigen::Matrix3d target_orientation_rhf = robot_hand_orientation_cf.transpose() * tag_orientation_cf;
            Eigen::Vector3d target_axis_orientation_rhf = inria::MatrixToAxis(target_orientation_rhf);

            // ---------------------------------------------------------------------------------------------------- //
            // CONTROLLER
            // controller initialized
            Apriltag_Controller apriltag_controller(kp, max_velocity);

            // initial position on rhf
            Eigen::Vector3d initial_position_rhf(0.0, 0.0, 0.0);
            Eigen::Vector3d initial_orientation_rhf(0.0, 0.0, 0.0);
            
            // this is what i'm going to send to the tiago robot
            std::vector<Eigen::Vector3d> robot_hand_velocity_rhf;
            if (robot_hand.detected.data == true and tag10.detected.data == true)
            {
                robot_hand_velocity_rhf = apriltag_controller.controller(target_position_rhf_v, target_axis_orientation_rhf, initial_position_rhf, initial_orientation_rhf);
            } 
            else
            {
                robot_hand_velocity_rhf[0].x() = 0.0;
                robot_hand_velocity_rhf[0].y() = 0.0;
                robot_hand_velocity_rhf[0].z() = 0.0;
                robot_hand_velocity_rhf[1].x() = 0.0;
                robot_hand_velocity_rhf[2].y() = 0.0;
                robot_hand_velocity_rhf[3].z() = 0.0;
            }
            
            client.setFloat(prefix_vel_lin+"x", robot_hand_velocity_rhf[0].x());
            client.setFloat(prefix_vel_lin+"y", robot_hand_velocity_rhf[0].y());
            client.setFloat(prefix_vel_lin+"z", robot_hand_velocity_rhf[0].z());
            client.setFloat(prefix_vel_ang+"x", robot_hand_velocity_rhf[1].x());
            client.setFloat(prefix_vel_ang+"y", robot_hand_velocity_rhf[1].y());
            client.setFloat(prefix_vel_ang+"z", robot_hand_velocity_rhf[1].z());
            client.send();

            std::cout << "****VELOCITY" << std::endl;
            std::cout << "----Linear" << std::endl;
            std::cout << "x velocity: " << robot_hand_velocity_rhf[0].x() << std::endl;
            std::cout << "y velocity: " << robot_hand_velocity_rhf[0].y() << std::endl;
            std::cout << "z velocity: " << robot_hand_velocity_rhf[0].z() << std::endl;
            std::cout << "----Angular" << std::endl;
            std::cout << "x velocity: " << robot_hand_velocity_rhf[1].x() << std::endl;
            std::cout << "y velocity: " << robot_hand_velocity_rhf[1].y() << std::endl;
            std::cout << "z velocity: " << robot_hand_velocity_rhf[1].z() << std::endl;
            // ---------------------------------------------------------------------------------------------------- //
            // TRANSFORMATION MATRIX for new position
            // updating position
            target_position_rhf_v = target_position_rhf_v - robot_hand_velocity_rhf[0] * dt;
            target_position_rhf(0,0) = target_position_rhf_v.x();
            target_position_rhf(1,0) = target_position_rhf_v.y();
            target_position_rhf(2,0) = target_position_rhf_v.z();

            // updating orientation
            // std::cout << "Axis orientation before" << std::endl << target_axis_orientation_rhf << std::endl;
            target_axis_orientation_rhf = target_axis_orientation_rhf - robot_hand_velocity_rhf[1] * dt;
            // std::cout << "Axis orientation after" << std::endl << target_axis_orientation_rhf << std::endl;
            target_orientation_rhf = inria::AxisToMatrix(target_axis_orientation_rhf);

            // ---------------------------------------------------------------------------------------------------- //
            // FEEDBACK

            // CALCULATED ORIENTATION on cf
            Eigen::Matrix3d predicted_orientation = tag_orientation_cf.inverse()*target_orientation_rhf;
            predicted_orientation = predicted_orientation.transpose();
            //std::cout << "Predicted orientation"
            // CALCULATED POSITION on cf
            Eigen::Matrix<double, 3, 1> predicted_position = - predicted_orientation.transpose().inverse() * target_position_rhf + new_tag_position;

            // REAL ORIENTATION
            Eigen::Quaterniond real_orientation_q(robot_hand.pose.pose.orientation.w, robot_hand.pose.pose.orientation.x, robot_hand.pose.pose.orientation.y, robot_hand.pose.pose.orientation.z);
            Eigen::Matrix3d real_orientation = real_orientation_q.toRotationMatrix();
            // REAL POSITION
            //Eigen::Matrix<double, 3, 1> real_position(robot_hand.pose.pose.position.x, robot_hand.pose.pose.position.y, robot_hand.pose.pose.position.z);

            // ---------------------------------------------------------------------------------------------------- //
            // COMPLEMENTARY FILTER (kalman filter)
            // alpha
            inria::FilterExponential<Eigen::Vector3d> filter_alpha_value;
            double alpha = filter_alpha_value.getAlphaFromFreq(0.1, dt);
            std::cout << "****Alpha value: " << alpha << std::endl;
            
            // position
            robot_hand_position_cf = alpha * predicted_position + (1 -  alpha) * predicted_position;
            std::cout << "robot hand position" << std::endl << robot_hand_position_cf << std::endl;

            //orientation
            //Eigen::Quaterniond q(predicted_orientation);
            robot_hand_quaternion_cf = Eigen::Quaterniond(predicted_orientation);
            robot_hand_orientation_cf = predicted_orientation;
            std::cout << "robot hand orientation" << std::endl;
            std::cout << "x: " << robot_hand_quaternion_cf.x() << std::endl;
            std::cout << "y: " << robot_hand_quaternion_cf.y() << std::endl;
            std::cout << "z: " << robot_hand_quaternion_cf.z() << std::endl;
            std::cout << "w: " << robot_hand_quaternion_cf.w() << std::endl;

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
            rh_init.pose.orientation.x = robot_hand_quaternion_cf_sim.x();
            rh_init.pose.orientation.y = robot_hand_quaternion_cf_sim.y();
            rh_init.pose.orientation.z = robot_hand_quaternion_cf_sim.z();
            rh_init.pose.orientation.w = robot_hand_quaternion_cf_sim.w();
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