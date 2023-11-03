#include <ros/ros.h>
#include <iostream>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Node initialized");

        Eigen::Matrix3d matrix; // Una matriz 3x3
        Eigen::Vector3d vector; // Un vector 3x1

        // Inicializa la matriz y el vector
        matrix << 1, 2, 3,
                4, 5, 6,
                7, 8, 9;

        vector << 2, 3, 4;

        // Realiza la multiplicación de matriz por vector
        Eigen::Vector3d result = matrix * vector;

        // Imprime el resultado
        std::cout << result << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

