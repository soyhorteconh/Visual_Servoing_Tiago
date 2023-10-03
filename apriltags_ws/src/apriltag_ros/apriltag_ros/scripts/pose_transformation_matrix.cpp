#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Geometry>
#include <iostream>

apriltag_ros::AprilTagDetection tag10;
apriltag_ros::AprilTagDetection tag20;

// Callbacks
void tag10_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag10.id = msg -> id;
    tag10.size = msg -> size;
    tag10.pose = msg -> pose;
    tag10.corner1 = msg -> corner1;
    tag10.corner2 = msg -> corner2;
    tag10.corner3 = msg -> corner3;
    tag10.corner4 = msg -> corner4;
    tag10.centerpx = msg -> centerpx;
    tag10.coordinate1 = msg -> coordinate1;
    tag10.coordinate2 = msg -> coordinate2;
    tag10.coordinate3 = msg -> coordinate3;
    tag10.coordinate4 = msg -> coordinate4;
    tag10.coordinate_center = msg -> coordinate_center;
    tag10.orientation = msg -> orientation;
    tag10.detected = msg -> detected;
}

void tag20_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag20.id = msg -> id;
    tag20.size = msg -> size;
    tag20.pose = msg -> pose;
    tag20.corner1 = msg -> corner1;
    tag20.corner2 = msg -> corner2;
    tag20.corner3 = msg -> corner3;
    tag20.corner4 = msg -> corner4;
    tag20.centerpx = msg -> centerpx;
    tag20.coordinate1 = msg -> coordinate1;
    tag20.coordinate2 = msg -> coordinate2;
    tag20.coordinate3 = msg -> coordinate3;
    tag20.coordinate4 = msg -> coordinate4;
    tag20.coordinate_center = msg -> coordinate_center;
    tag20.orientation = msg -> orientation;
    tag20.detected = msg -> detected;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_transformation_matrix");
    ros::NodeHandle n;

    // publishers
    ros::Publisher camera_publisher = n.advertise<visualization_msgs::Marker>( "camera_matrix", 0 );
    ros::Publisher tag_publisher = n.advertise<visualization_msgs::Marker>( "tag_visualization_matrix", 0 );

    // subscribers
    ros::Subscriber tag10_subscriber = n.subscribe("tag10", 1, tag10_cb); 
    ros::Subscriber tag20_subscriber = n.subscribe("tag20", 1, tag20_cb); 

    // Marker
    visualization_msgs::Marker marker1;
    marker1.header.frame_id = "camera_matrix";
    marker1.header.stamp = ros::Time();
    marker1.ns = "tag10";
    marker1.id = 10;
    marker1.mesh_resource = "package://apriltag_ros/scripts/mesh/tag10.dae";
    marker1.mesh_use_embedded_materials = true;
    marker1.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.position.x = 0;
    marker1.pose.position.y = 0;
    marker1.pose.position.z = 0;
    marker1.pose.orientation.x = 0;
    marker1.pose.orientation.y = 0;
    marker1.pose.orientation.z = 0;
    marker1.pose.orientation.w = 1;
    marker1.scale.x = 2;
    marker1.scale.y = 2;
    marker1.scale.z = 1;
    marker1.color.a = 1.0; //alpha
    marker1.color.r = 0.0;
    marker1.color.g = 0.0;
    marker1.color.b = 0.0;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        camera_publisher.publish(marker1);

        //quaternion
        Eigen::Quaterniond q_10(tag10.orientation.w, tag10.orientation.x, tag10.orientation.y, tag10.orientation.z);
        Eigen::Quaterniond q_20(tag20.orientation.w, tag20.orientation.x, tag20.orientation.y, tag20.orientation.z);

        //rotation matrix
        Eigen::Matrix3d r_10 = q_10.normalized().toRotationMatrix();
        Eigen::Matrix3d r_20 = q_20.normalized().toRotationMatrix();
        Eigen::Matrix3d r_10_20 = r_10.transpose()*r_20;
        Eigen::Quaterniond q(r_10_20);

        //translation matrix
        Eigen::Matrix<double,3,1> p_10{tag10.coordinate_center.x,tag10.coordinate_center.y,tag10.coordinate_center.z};
        Eigen::Matrix<double,3,1> p_20{tag20.coordinate_center.x,tag20.coordinate_center.y,tag20.coordinate_center.z};
        Eigen::Matrix<double,3,1> p_10_20 = r_10.transpose()*(p_20-p_10);
        
        //std::cout << "t_10_20 =\n" << t_10_0 * t_20 << std::endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_matrix";
        marker.header.stamp = ros::Time();
        marker.ns = "tag20";
        marker.id = 20;
        marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag20.dae";
        marker.mesh_use_embedded_materials = true;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p_10_20(0,0);
        marker.pose.position.y = p_10_20(1,0);
        marker.pose.position.z = p_10_20(2,0);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 1;
        marker.color.a = 1.0; //alpha
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        tag_publisher.publish(marker);


        ros::spinOnce();
        loop_rate.sleep();
    }    

    return 0;
}
