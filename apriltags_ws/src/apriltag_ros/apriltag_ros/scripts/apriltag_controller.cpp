// libraries
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/RobotHand.h>

ros::Publisher marker_publisher;
apriltag_ros::AprilTagDetection tag10;
apriltag_ros::RobotHand robot_hand;

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
void publish_marker(std::string ns, int id, std::string color, Eigen::Vector3d position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.09;
    marker.scale.y = 0.09;
    marker.scale.z = 0.09;
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
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    // publishers
    ros::Publisher controller_publisher = n.advertise<geometry_msgs::Twist>("velocity_control", 1);
    marker_publisher = n.advertise<visualization_msgs::Marker>("markers", 0);

    //subscribers
    ros::Subscriber robot_hand_subscriber = n.subscribe("robot_hand/pose", 1, robot_hand_cb); 
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/filter", 1, tag10_cb);

    // acronyms
    // cf: camera frame
    // rhf: robot hand frame

    //robot hand Pose on cf
    Eigen::Matrix<double, 3, 1> robot_hand_position_cf{1.0,-1.5,1.0};
    Eigen::Quaterniond robot_hand_quaternion_cf(1.0, 0.0, 0.0, 0.0);
    Eigen::Matrix3d robot_hand_orientation_cf = robot_hand_quaternion_cf.toRotationMatrix();

    // target pose on cf
    Eigen::Matrix<double, 3, 1> tag_position_cf{1.5, -0.05, 2.0};
    Eigen::Quaterniond tag_position_quaternion_cf(1.0, 0.0, 0.0, 0.0);
    Eigen::Matrix3d tag_position_orientation_cf = tag_position_quaternion_cf.toRotationMatrix();

    Eigen::Vector3d init(0.0, 0.0, 0.0);
    Eigen::Vector3d robot_hand_position_cf_sim (robot_hand_position_cf(0,0),robot_hand_position_cf(1,0),robot_hand_position_cf(2,0));
    Eigen::Vector3d tag_position_cf_sim(tag_position_cf(0,0), tag_position_cf(1,0), tag_position_cf(2,0));

    double dt = 0.0333;
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

        if(robot_hand.detected.data == true and tag10.detected.data == true and flag == false){
            std::cout << "Aqui estoyyyyy" << std::endl;
            robot_hand_position_cf(0,0) = robot_hand.pose.pose.position.x;
            robot_hand_position_cf(1,0) = robot_hand.pose.pose.position.y;
            robot_hand_position_cf(2,0) = robot_hand.pose.pose.position.z;

            tag_position_cf(0,0) = tag10.coordinate_center.x;
            tag_position_cf(1,0) = tag10.coordinate_center.y;
            tag_position_cf(2,0) = tag10.coordinate_center.z;

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
            std::cout << "---------------------------------------------" << std::endl;
            std::cout << "TAG" << std::endl;
            std::cout << "x: " << tag10.coordinate_center.x << std::endl;
            std::cout << "y: " << tag10.coordinate_center.y << std::endl;
            std::cout << "z: " << tag10.coordinate_center.z << std::endl;

            std::cout << "ROBOT HAND" << std::endl;
            std::cout << "x: " << robot_hand.pose.pose.position.x << std::endl;
            std::cout << "y: " << robot_hand.pose.pose.position.y << std::endl;
            std::cout << "z: " << robot_hand.pose.pose.position.z << std::endl;

            // //STOP 10 cm before on z
            Eigen::Matrix<double, 3, 1> rest_5(0.0,0.0,0.1);
            Eigen::Matrix<double, 3, 1>  new_tag_position = tag_position_cf - rest_5;
            //std::cout << "New tag position" << std::endl << new_tag_position << std::endl;

            // getting target on rhf
            Eigen::Matrix<double, 3, 1> target_position_rhf = robot_hand_orientation_cf.transpose() * (new_tag_position - robot_hand_position_cf);
            Eigen::Vector3d target_position_rhf_v(target_position_rhf(0,0), target_position_rhf(1,0), target_position_rhf(2,0));
            //std::cout << "Target position: " << std::endl << target_position_rhf << std::endl;

            Eigen::Quaterniond target_orientation_rhf_q(1.0, 0.0, 0.0, 0.0);
            Eigen::Matrix3d target_orientation_rhf = target_orientation_rhf_q.toRotationMatrix();

            // controller initialized
            double kp = 0.1;
            double max_velocity = 0.05;
            Apriltag_Controller apriltag_controller(kp, max_velocity);

            // initial position on rhf
            Eigen::Vector3d initial_position_rhf(0.0, 0.0, 0.0);
            // this is what i'm going to send to the tiago robot
            Eigen::Vector3d robot_hand_velocity_rhf = apriltag_controller.controller(target_position_rhf_v, initial_position_rhf);
            
            // updating position
            target_position_rhf_v = target_position_rhf_v - robot_hand_velocity_rhf * dt;
            target_position_rhf(0,0) = target_position_rhf_v.x();
            target_position_rhf(1,0) = target_position_rhf_v.y();
            target_position_rhf(2,0) = target_position_rhf_v.z();

            //std::cout << "target position updated: " << std::endl << target_position_rhf << std::endl;

            Eigen::Matrix<double, 3, 1> new_position = - target_orientation_rhf.transpose().inverse() * target_position_rhf + new_tag_position;
            //std::cout << "new position" << std::endl << new_position << std::endl;
            robot_hand_position_cf = new_position;
            
            std::cout << "robot hand position" << std::endl << robot_hand_position_cf << std::endl;

            // simulation
            //  target
            publish_marker("target", 1, "blue", tag_position_cf_sim); 
            // initial
            publish_marker("initial_position", 2, "green", robot_hand_position_cf_sim);
            // controller
            publish_marker("robot_hand_position", 3, "red", robot_hand_position_cf);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
