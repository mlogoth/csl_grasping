// ROS headers
#include <ros/ros.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <math.h>
#include <typeinfo>

//ROS Messages
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

//Package Header Files
#include <csl_grasping/FastGrasping.h>

// Math Utils
#include <csl_grasping/math_utils.hpp>

int main(int argc, char *argv[])
{
	//init script ros node
	ros::init(argc, argv, "test_grasping_using_pcl");
	// node handler
	ros::NodeHandle nh;
	//Rate of Algorithm
	ros::Rate rate(30);
	
	FastGrasp graping(nh);
	
	//while(ros::ok())
	//{
	cout<<"I am ok!"<<endl;
	rate.sleep();
	//}
	// Rate Of Algorithm
	rate.sleep();
	return 0;
}
