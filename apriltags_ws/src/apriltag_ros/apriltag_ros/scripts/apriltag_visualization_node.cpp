#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"

apriltag_ros::AprilTagDetection tag5;
apriltag_ros::AprilTagDetection tag6;
apriltag_ros::AprilTagDetection tag7;
apriltag_ros::AprilTagDetection tag8;
apriltag_ros::AprilTagDetection tag10;
apriltag_ros::AprilTagDetection tag20;

apriltag_ros::AprilTagDetection tag5_filter;

// Callbacks
void tag5_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag5.id = msg -> id;
    tag5.size = msg -> size;
    tag5.pose = msg -> pose;
    tag5.corner1 = msg -> corner1;
    tag5.corner2 = msg -> corner2;
    tag5.corner3 = msg -> corner3;
    tag5.corner4 = msg -> corner4;
    tag5.centerpx = msg -> centerpx;
    tag5.coordinate1 = msg -> coordinate1;
    tag5.coordinate2 = msg -> coordinate2;
    tag5.coordinate3 = msg -> coordinate3;
    tag5.coordinate4 = msg -> coordinate4;
    tag5.coordinate_center = msg -> coordinate_center;
    tag5.orientation = msg -> orientation;
    tag5.detected = msg -> detected;
}

void tag6_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag6.id = msg -> id;
    tag6.size = msg -> size;
    tag6.pose = msg -> pose;
    tag6.corner1 = msg -> corner1;
    tag6.corner2 = msg -> corner2;
    tag6.corner3 = msg -> corner3;
    tag6.corner4 = msg -> corner4;
    tag6.centerpx = msg -> centerpx;
    tag6.coordinate1 = msg -> coordinate1;
    tag6.coordinate2 = msg -> coordinate2;
    tag6.coordinate3 = msg -> coordinate3;
    tag6.coordinate4 = msg -> coordinate4;
    tag6.coordinate_center = msg -> coordinate_center;
    tag6.orientation = msg -> orientation;
    tag6.detected = msg -> detected;
}

void tag7_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag7.id = msg -> id;
    tag7.size = msg -> size;
    tag7.pose = msg -> pose;
    tag7.corner1 = msg -> corner1;
    tag7.corner2 = msg -> corner2;
    tag7.corner3 = msg -> corner3;
    tag7.corner4 = msg -> corner4;
    tag7.centerpx = msg -> centerpx;
    tag7.coordinate1 = msg -> coordinate1;
    tag7.coordinate2 = msg -> coordinate2;
    tag7.coordinate3 = msg -> coordinate3;
    tag7.coordinate4 = msg -> coordinate4;
    tag7.coordinate_center = msg -> coordinate_center;
    tag7.orientation = msg -> orientation;
    tag7.detected = msg -> detected;
}

void tag8_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag8.id = msg -> id;
    tag8.size = msg -> size;
    tag8.pose = msg -> pose;
    tag8.corner1 = msg -> corner1;
    tag8.corner2 = msg -> corner2;
    tag8.corner3 = msg -> corner3;
    tag8.corner4 = msg -> corner4;
    tag8.centerpx = msg -> centerpx;
    tag8.coordinate1 = msg -> coordinate1;
    tag8.coordinate2 = msg -> coordinate2;
    tag8.coordinate3 = msg -> coordinate3;
    tag8.coordinate4 = msg -> coordinate4;
    tag8.coordinate_center = msg -> coordinate_center;
    tag8.orientation = msg -> orientation;
    tag8.detected = msg -> detected;
}

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

void tag5_filter_cb(const apriltag_ros::AprilTagDetection::ConstPtr &msg)
{
    tag5_filter.id = msg -> id;
    tag5_filter.size = msg -> size;
    tag5_filter.pose = msg -> pose;
    tag5_filter.corner1 = msg -> corner1;
    tag5_filter.corner2 = msg -> corner2;
    tag5_filter.corner3 = msg -> corner3;
    tag5_filter.corner4 = msg -> corner4;
    tag5_filter.centerpx = msg -> centerpx;
    tag5_filter.coordinate1 = msg -> coordinate1;
    tag5_filter.coordinate2 = msg -> coordinate2;
    tag5_filter.coordinate3 = msg -> coordinate3;
    tag5_filter.coordinate4 = msg -> coordinate4;
    tag5_filter.coordinate_center = msg -> coordinate_center;
    tag5_filter.orientation = msg -> orientation;
    tag5_filter.detected = msg -> detected;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_visualization");
    ros::NodeHandle n;

    // Publisher
    ros::Publisher camera_publisher = n.advertise<visualization_msgs::Marker>( "camera", 0 );
    ros::Publisher tag_publisher = n.advertise<visualization_msgs::Marker>( "tag_visualization_rviz", 0 );

    // Subscribers
    ros::Subscriber tag5_subscriber = n.subscribe("tag5", 1, tag5_cb); 
    ros::Subscriber tag6_subscriber = n.subscribe("tag6", 1, tag6_cb);
    ros::Subscriber tag7_subscriber = n.subscribe("tag7", 1, tag7_cb); 
    ros::Subscriber tag8_subscriber = n.subscribe("tag8", 1, tag8_cb);
    ros::Subscriber tag10_subscriber = n.subscribe("tag10", 1, tag10_cb); 
    ros::Subscriber tag20_subscriber = n.subscribe("tag20", 1, tag20_cb);

    ros::Subscriber tag5_subscriber_norm = n.subscribe("tag5_filter", 1, tag5_filter_cb);

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
        if (tag5.detected.data)
        {
            //ROS_INFO("Tag5 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag5";
            marker.id = 5;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag5.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag5.coordinate_center.x;
            marker.pose.position.y = tag5.coordinate_center.y;
            marker.pose.position.z = tag5.coordinate_center.z;
            marker.pose.orientation.x = tag5.orientation.x;
            marker.pose.orientation.y = tag5.orientation.y;
            marker.pose.orientation.z = tag5.orientation.z;
            marker.pose.orientation.w = tag5.orientation.w;
            // marker.pose.position.x = tag5.pose.pose.pose.position.x;
            // marker.pose.position.y = tag5.pose.pose.pose.position.y;
            // marker.pose.position.z = tag5.centerpx.z;
            // marker.pose.orientation.x = tag5.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag5.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag5.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag5.pose.pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag5.detected.data == false)
        {
            //ROS_INFO("Tag5 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag5";
            marker.id = 5;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag5 filtering data
        if (tag5_filter.detected.data)
        {
            //ROS_INFO("Tag5 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag5_filter";
            marker.id = 105;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag5.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag5_filter.coordinate_center.x;
            marker.pose.position.y = tag5_filter.coordinate_center.y;
            marker.pose.position.z = tag5_filter.coordinate_center.z;
            marker.pose.orientation.x = tag5_filter.orientation.x;
            marker.pose.orientation.y = tag5_filter.orientation.y;
            marker.pose.orientation.z = tag5_filter.orientation.z;
            marker.pose.orientation.w = tag5_filter.orientation.w;
            // marker.pose.position.x = tag5.pose.pose.pose.position.x;
            // marker.pose.position.y = tag5.pose.pose.pose.position.y;
            // marker.pose.position.z = tag5.centerpx.z;
            // marker.pose.orientation.x = tag5.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag5.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag5.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag5.pose.pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; //alpha
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag5_filter.detected.data == false)
        {
            //ROS_INFO("Tag5 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag5_filter";
            marker.id = 105;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag6
        if (tag6.detected.data)
        {
            //ROS_INFO("Tag6 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag6";
            marker.id = 6;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag6.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag6.coordinate_center.x;
            marker.pose.position.y = tag6.coordinate_center.y;
            marker.pose.position.z = tag6.coordinate_center.z;
            marker.pose.orientation.x = tag6.orientation.x;
            marker.pose.orientation.y = tag6.orientation.y;
            marker.pose.orientation.z = tag6.orientation.z;
            marker.pose.orientation.w = tag6.orientation.w;
            // marker.pose.position.x = tag6.pose.pose.pose.position.x;
            // marker.pose.position.y = tag6.pose.pose.pose.position.y;
            // marker.pose.position.z = tag6.centerpx.z;
            // marker.pose.orientation.x = tag6.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag6.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag6.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag6.pose.pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag6.detected.data == false)
        {
            //ROS_INFO("Tag6 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag6";
            marker.id = 6;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag7
        if (tag7.detected.data)
        {
            //ROS_INFO("Tag7 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag7";
            marker.id = 7;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag7.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag7.coordinate_center.x;
            marker.pose.position.y = tag7.coordinate_center.y;
            marker.pose.position.z = tag7.coordinate_center.z;
            marker.pose.orientation.x = tag7.orientation.x;
            marker.pose.orientation.y = tag7.orientation.y;
            marker.pose.orientation.z = tag7.orientation.z;
            marker.pose.orientation.w = tag7.orientation.w;
            // marker.pose.position.x = tag7.pose.pose.pose.position.x;
            // marker.pose.position.y = tag7.pose.pose.pose.position.y;
            // marker.pose.position.z = tag7.centerpx.z;
            // marker.pose.orientation.x = tag7.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag7.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag7.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag7.pose.pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag7.detected.data == false)
        {
            //ROS_INFO("Tag7 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag7";
            marker.id = 7;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag8
        if (tag8.detected.data)
        {
            //ROS_INFO("Tag8 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag8";
            marker.id = 8;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag8.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag8.coordinate_center.x;
            marker.pose.position.y = tag8.coordinate_center.y;
            marker.pose.position.z = tag8.coordinate_center.z;
            marker.pose.orientation.x = tag8.orientation.x;
            marker.pose.orientation.y = tag8.orientation.y;
            marker.pose.orientation.z = tag8.orientation.z;
            marker.pose.orientation.w = tag8.orientation.w;
            // marker.pose.position.x = tag8.pose.pose.pose.position.x;
            // marker.pose.position.y = tag8.pose.pose.pose.position.y;
            // marker.pose.position.z = tag8.centerpx.z;
            // marker.pose.orientation.x = tag8.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag8.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag8.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag8.pose.pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.25;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag8.detected.data == false)
        {
            //ROS_INFO("Tag8 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag8";
            marker.id = 8;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag10
        if (tag10.detected.data)
        {
            //ROS_INFO("Tag10 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag10";
            marker.id = 10;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag10.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag10.coordinate_center.x;
            marker.pose.position.y = tag10.coordinate_center.y;
            marker.pose.position.z = tag10.coordinate_center.z;
            marker.pose.orientation.x = tag10.orientation.x;
            marker.pose.orientation.y = tag10.orientation.y;
            marker.pose.orientation.z = tag10.orientation.z;
            marker.pose.orientation.w = tag10.orientation.w;
            // marker.pose.position.x = tag10.pose.pose.pose.position.x;
            // marker.pose.position.y = tag10.pose.pose.pose.position.y;
            // marker.pose.position.z = tag10.centerpx.z;
            // marker.pose.orientation.x = tag10.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag10.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag10.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag10.pose.pose.pose.orientation.w;
            marker.scale.x = 2;
            marker.scale.y = 2;
            marker.scale.z = 1;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag10.detected.data == false)
        {
            //ROS_INFO("Tag10 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag10";
            marker.id = 10;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        // tag20
        if (tag20.detected.data)
        {
            //ROS_INFO("Tag20 detected");

            // Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag20";
            marker.id = 20;
            marker.mesh_resource = "package://apriltag_ros/scripts/mesh/tag20.dae";
            marker.mesh_use_embedded_materials = true;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tag20.coordinate_center.x;
            marker.pose.position.y = tag20.coordinate_center.y;
            marker.pose.position.z = tag20.coordinate_center.z;
            marker.pose.orientation.x = tag20.orientation.x;
            marker.pose.orientation.y = tag20.orientation.y;
            marker.pose.orientation.z = tag20.orientation.z;
            marker.pose.orientation.w = tag20.orientation.w;
            // marker.pose.position.x = tag20.pose.pose.pose.position.x;
            // marker.pose.position.y = tag20.pose.pose.pose.position.y;
            // marker.pose.position.z = tag20.centerpx.z;
            // marker.pose.orientation.x = tag20.pose.pose.pose.orientation.x;
            // marker.pose.orientation.y = tag20.pose.pose.pose.orientation.y;
            // marker.pose.orientation.z = tag20.pose.pose.pose.orientation.z;
            // marker.pose.orientation.w = tag20.pose.pose.pose.orientation.w;
            marker.scale.x = 2;
            marker.scale.y = 2;
            marker.scale.z = 1;
            marker.color.a = 1.0; //alpha
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            tag_publisher.publish(marker);

        } else if (tag20.detected.data == false)
        {
            //ROS_INFO("Tag20 not detected");

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "tag20";
            marker.id = 20;
            marker.action = visualization_msgs::Marker::DELETE;

            tag_publisher.publish(marker);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
