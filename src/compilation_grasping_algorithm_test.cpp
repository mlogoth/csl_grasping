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

// We need boost for multi threading visualization

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


int main(int argc, char *argv[])
{
	//init script ros node
	ros::init(argc, argv, "test_grasping_using_pcl");
	// node handler
	ros::NodeHandle nh;
	//Rate of Algorithm
	ros::Rate rate(30);
	
	
	FastGrasp graping(nh);
	
	while(ros::ok())
	{
	
	 /*
	 * This function Computes The Concave Hull of Object's Partial Point Cloud 
	 * and returns: 
	 *  1)  the Source Point Cloud 
	 *  2)  Point Cloud of the corresponding Graspable Concave Hull Point Cloud
	 *  3)  Point Cloud Centroid
	 */ 
		// This structure is implemented in FastGrasping.h
		// 2 point clouds (ptr) + 1 vector4f 
		PointCloudsCentroid cloud_centroid;
		
		// Read source point cloud, concave_hull cloud and centroid
		cloud_centroid = graping.GetConhullContour();
		
		// Visualize them.
		
		
		cout<<"I am ok!"<<endl;
		
	    rate.sleep();
	}
	// Rate Of Algorithm

	return 0;
}
