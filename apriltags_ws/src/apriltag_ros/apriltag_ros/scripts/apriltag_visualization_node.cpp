#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/RobotHand.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "map"

apriltag_ros::AprilTagDetection tag5;
apriltag_ros::AprilTagDetection tag6;
apriltag_ros::AprilTagDetection tag7;
apriltag_ros::AprilTagDetection tag8;
apriltag_ros::AprilTagDetection tag10;
apriltag_ros::AprilTagDetection tag20;

ros::Publisher tag_publisher;

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

void tag20_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag20.coordinate_center = msg -> coordinate_center;
    tag20.orientation = msg -> orientation;
    tag20.detected = msg -> detected;
}

// visualization TAG marker
void tag_publish_marker(std::string ns, int id, apriltag_ros::AprilTagDetection tag_information, std::string action)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;

    //map
    std::map<std::string, std::string> meshResources;
    meshResources["Tag5"] = "package://apriltag_ros/scripts/mesh/tag5.dae";
    meshResources["Tag6"] = "package://apriltag_ros/scripts/mesh/tag6.dae";
    meshResources["Tag7"] = "package://apriltag_ros/scripts/mesh/tag7.dae";
    meshResources["Tag8"] = "package://apriltag_ros/scripts/mesh/tag8.dae";
    meshResources["Tag10"] = "package://apriltag_ros/scripts/mesh/tag10.dae";
    meshResources["Tag20"] = "package://apriltag_ros/scripts/mesh/tag20.dae";

    if (meshResources.find(ns) != meshResources.end()) {
        marker.mesh_resource = meshResources[ns];
    }

    marker.mesh_use_embedded_materials = true;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    if(action == "add"){ marker.action = visualization_msgs::Marker::ADD; }
    else if (action == "delete"){ marker.action = visualization_msgs::Marker::DELETE; }

    marker.pose.position.x = tag_information.coordinate_center.x;
    marker.pose.position.y = tag_information.coordinate_center.y;
    marker.pose.position.z = tag_information.coordinate_center.z;
    marker.pose.orientation.x = tag_information.orientation.x;
    marker.pose.orientation.y = tag_information.orientation.y;
    marker.pose.orientation.z = tag_information.orientation.z;
    marker.pose.orientation.w = tag_information.orientation.w;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.25;
    marker.color.a = 1.0; //alpha
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    tag_publisher.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_visualization");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher camera_publisher = n.advertise<visualization_msgs::Marker>( "camera", 0 );
    tag_publisher = n.advertise<visualization_msgs::Marker>( "tag_visualization_rviz", 0 );

    // Subscribers
    ros::Subscriber tag5_subscriber = n.subscribe("tag5/information", 1, tag5_cb);
    ros::Subscriber tag6_subscriber = n.subscribe("tag6/information", 1, tag6_cb);
    ros::Subscriber tag7_subscriber = n.subscribe("tag7/information", 1, tag7_cb); 
    ros::Subscriber tag8_subscriber = n.subscribe("tag8/information", 1, tag8_cb);
    ros::Subscriber tag10_subscriber = n.subscribe("tag10/information", 1, tag10_cb); 
    ros::Subscriber tag20_subscriber = n.subscribe("tag20/filter", 1, tag20_cb);

    // camera marker
    // x
    visualization_msgs::Marker camera_x;
    camera_x.header.frame_id = "map";
    camera_x.header.stamp = ros::Time();
    camera_x.ns = "camera_x";
    camera_x.id = 1;
    camera_x.type = visualization_msgs::Marker::ARROW;
    camera_x.action = visualization_msgs::Marker::ADD;
    camera_x.pose.position.x = 0;
    camera_x.pose.position.y = 0;
    camera_x.pose.position.z = 0;
    camera_x.pose.orientation.x = 1;
    camera_x.pose.orientation.y = 0;
    camera_x.pose.orientation.z = 0;
    camera_x.pose.orientation.w = 1;
    camera_x.scale.x = 0.1;
    camera_x.scale.y = 0.01;
    camera_x.scale.z = 0.01;
    camera_x.color.a = 1.0; //alpha
    camera_x.color.r = 1.0;
    camera_x.color.g = 0.0;
    camera_x.color.b = 0.0;

    // y
    visualization_msgs::Marker camera_y;
    camera_y.header.frame_id = "map";
    camera_y.header.stamp = ros::Time();
    camera_y.ns = "camera_y";
    camera_y.id = 2;
    camera_y.type = visualization_msgs::Marker::ARROW;
    camera_y.action = visualization_msgs::Marker::ADD;
    camera_y.pose.position.x = 0;
    camera_y.pose.position.y = 0;
    camera_y.pose.position.z = 0;
    camera_y.pose.orientation.x = 0;
    camera_y.pose.orientation.y = 0;
    camera_y.pose.orientation.z = -1;
    camera_y.pose.orientation.w = 1;
    camera_y.scale.x = 0.1;
    camera_y.scale.y = 0.01;
    camera_y.scale.z = 0.01;
    camera_y.color.a = 1.0; //alpha
    camera_y.color.r = 0.0;
    camera_y.color.g = 1.0;
    camera_y.color.b = 0.0;

    // z
    visualization_msgs::Marker camera_z;
    camera_z.header.frame_id = "map";
    camera_z.header.stamp = ros::Time();
    camera_z.ns = "camera_z";
    camera_z.id = 3;
    camera_z.type = visualization_msgs::Marker::ARROW;
    camera_z.action = visualization_msgs::Marker::ADD;
    camera_z.pose.position.x = 0;
    camera_z.pose.position.y = 0;
    camera_z.pose.position.z = 0;
    camera_z.pose.orientation.x = 0;
    camera_z.pose.orientation.y = -1;
    camera_z.pose.orientation.z = 0;
    camera_z.pose.orientation.w = 1;
    camera_z.scale.x = 0.1;
    camera_z.scale.y = 0.01;
    camera_z.scale.z = 0.01;
    camera_z.color.a = 1.0; //alpha
    camera_z.color.r = 0.0;
    camera_z.color.g = 0.0;
    camera_z.color.b = 1.0;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        camera_publisher.publish(camera_x);
        camera_publisher.publish(camera_y);
        camera_publisher.publish(camera_z);

        // tag5
        if (tag5.detected.data){ tag_publish_marker("Tag5", 5, tag5, "add"); } 
        else if (tag5.detected.data == false){ tag_publish_marker("Tag5", 5, tag5, "delete"); }

        // tag6
        if (tag6.detected.data){ tag_publish_marker("Tag6", 6, tag6, "add"); } 
        else if (tag6.detected.data == false){ tag_publish_marker("Tag6", 6, tag6, "delete"); }

        // tag7
        if (tag7.detected.data){ tag_publish_marker("Tag7", 7, tag7, "add"); } 
        else if (tag7.detected.data == false){ tag_publish_marker("Tag7", 7, tag7, "delete"); }

        // tag8
        if (tag8.detected.data){ tag_publish_marker("Tag8", 8, tag8, "add"); } 
        else if (tag8.detected.data == false){ tag_publish_marker("Tag8", 8, tag8, "delete"); }

        // tag10
        if (tag10.detected.data){ tag_publish_marker("Tag10", 10, tag10, "add"); }
        else if (tag10.detected.data == false){ tag_publish_marker("Tag10", 10, tag10, "delete"); }

        // tag20
        if (tag20.detected.data){ tag_publish_marker("Tag20", 20, tag20, "add"); } 
        else if (tag20.detected.data == false){ tag_publish_marker("Tag20", 20, tag20, "delete"); }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
