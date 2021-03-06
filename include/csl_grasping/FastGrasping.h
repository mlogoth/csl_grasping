#ifndef FASTGRASPING_H_
#define FASTGRASPING_H_

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <math.h>
#include <typeinfo>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/tf.h"

// PCL headers
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>
#include "pcl_ros/transforms.h"
// Contour
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//pcl filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

using namespace std;
using namespace Eigen;

struct PointCloudsCentroid {
	pcl::PointCloud<pcl::PointXYZ>::Ptr SourcePointCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr HullPointCloud;
	Eigen::Vector4f centroid;
};

struct pcl_mask {
	Eigen::Vector4f min;
	Eigen::Vector4f max;
};

struct grasp_regions {
	double angle;
	
	//pcl::PointIndices::Ptr left_indicies;
	//pcl::PointIndices::Ptr right_indicies;
	std::vector<int> left_indicies;
	std::vector<int> right_indicies;
	pcl_mask left_mask;
	pcl_mask right_mask;
	double mask_pose;
	tf::Vector3 line_vec;
};

struct optGraspVars{
	grasp_regions areas;
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_seg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_seg;
};


struct GraspPose{
	Vector4f p;
	Matrix3d R;
};


class FastGrasp {

protected:
	ros::NodeHandle nhd;
	ros::Subscriber sub_pc, sub_obj;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
	pcl::PointXYZ min, max;
	pcl::SACSegmentation < pcl::PointXYZ > seg;


	Eigen::Vector4f centroid;
	std::string object_name;
	Eigen::Affine3d obj_transformation;

	//Rotation 
	tf::Vector3 z_axis;
	Matrix3d Ident, Sk2, v_skew,  Rot_z;
	std::vector<int> setGraspPoints;
	
	Vector4f grasp_pose;
	double mask_height,mask_length, step, gripper_length;
	int loops;
	Vector3f mask_pose;
	string kinect_frame;

	std::vector<int> set_l, set_r;
	grasp_regions optimal_grasp;
	
	pcl::PointIndices::Ptr inli_l, inli_r;
	pcl::ModelCoefficients::Ptr coeff_l, coeff_r;
	// Create the segmentation object

	
	//---------------------------------------------
	// Variables of  get_conhull_contour() 
	//Create Concave Hull Object
	pcl::ConcaveHull < pcl::PointXYZ > chull;
	//Create Projection Object
	pcl::ProjectInliers < pcl::PointXYZ > proj;
	// Min - Max Variables 
	pcl::PointXYZ src_max,src_min;
	// Inliers and Coefficients

public:
	
	
	FastGrasp(ros::NodeHandle n_);

	/*
	* CallBack Function Reads Object Point Cloud
	*/
	void getCloud(const sensor_msgs::PointCloud2 input);

	/*
	* CallBack Function Reads Object Pose
	*/
	void getPose(const geometry_msgs::PoseStamped pose_stamped);

	/*
	* Filter Pass 
	*/
	void filterPass(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZ> &result, std::string axis, double min_, double max_);

	/*
	* Line or Plane Segmentation of The object
	*/
	void Segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double threshold, std::string type);


	/*
	* Extract a set of indices from a point cloud. 
	*/
	void ExtractIndices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, boost::shared_ptr< vector<int> > &indicesptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &result);


	/*
	 * This function Computes The Concave Hull of Object's Partial Point Cloud 
	 * and returns: 
	 *  1)  the Source Point Cloud 
	 *  2)  Point Cloud of the corresponding Graspable Concave Hull Point Cloud
	 *  3)  Point Cloud Centroid
	 */ 
	PointCloudsCentroid GetConhullContour();
	
	/*
	* Calculates Optimal Grasping Areas
	*/
	optGraspVars OptimalAreasSearching(pcl::PointCloud<pcl::PointXYZ>::Ptr cld);
	
	
	/*
	 * Optimal Grasping Pose of Gripper
	*/
	GraspPose getGraspPose();

	void SetGripperHeight(double gr_height);
	
	void SetGripperLength(double gr_length);
	
	void SetGripperMaxOpen(double gr_open);
	
	void SetSearchingLoops(int loops_);
	
	void SetObjectName(std::string name_);

};


































#endif 
