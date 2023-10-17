#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

ros::Publisher video_publisher;
ros::Publisher camera_info_publisher;

void video_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    // ROS_INFO("  height: %d", msg->height);
    // ROS_INFO("  width: %d", msg->width);
    video_publisher.publish(msg);
}

void camera_info_cb( const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    camera_info_publisher.publish(msg);
}

int main(int argc, char **argv)
{
    // Set up ROS
    // Node
    ros::init(argc, argv, "azure_kinect_publisher");
    ros::NodeHandle n;

    //Publishers
    video_publisher = n.advertise<sensor_msgs::Image>("/camera_rect/image_rect",1);
    camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("/camera_rect/camera_info",1);

    // Subscribers
    ros::Subscriber video_subscriber = n.subscribe("rgb/image_raw", 1, video_cb); 
    ros::Subscriber camera_info_subscriber = n.subscribe("rgb/camera_info", 1, camera_info_cb);

    ROS_INFO("Node initialized");
    ros::spin();

    return 0;
}