#include <csl_grasping/FastGrasping.h>

FastGrasp::FastGrasp(ros::NodeHandle n_){
		nhd = n_;
		
		//Initialize the Subscribing Topic And the Object Name
		string topic_cloud_name, topic_pose_name ;
		
		// Read Topic Name From Parameter Server
		// IF not read the default topic.
		if (!n_.getParam("grasping/object/point_cloud_topic", topic_cloud_name))
		{
			topic_cloud_name = "/tracking_object_cloud";
		}
		
		if (!n_.getParam("grasping/object/pose_topic", topic_pose_name))
		{
			topic_pose_name = "/tracking_object_pose";
		}
		
		if (!n_.getParam("grasping/kinect_frame", kinect_frame))
		{
			kinect_frame = "/kinect_rgb_link";
		}

		//Initialize Grasping Algorithm Parameters
		/* Gripper Geometry */
		if (!n_.getParam("grasping/gripper/height", mask_height))
		{
			mask_height = 0.05;
		}
		
		if (!n_.getParam("grasping/gripper/length", gripper_length))
		{
			gripper_length = 0.07;
		}
		
		if (!n_.getParam("grasping/gripper/open", mask_length))
		{
			mask_length = 0.095;
		}
		
		/* Search Density */
		if (!n_.getParam("grasping/loops", loops))
		{
			loops = 20;
		}
		
		// Grasping Parameters Initialization
		ROS_INFO("> CSL Grasping Parameters Initialization:");
		ROS_INFO(" Object Point Cloud Topic: '%s'", topic_cloud_name.c_str());
		ROS_INFO(" Object Pose Topic:        '%s'", topic_pose_name.c_str());
		ROS_INFO(" RGBD Camera Frame:        '%s'", kinect_frame.c_str());
		ROS_INFO(" Gripper Characteristics:");
		ROS_INFO("   1) Gripper Height:   %3.2f", mask_height);
		ROS_INFO("   2) Gripper Length:   %3.2f", gripper_length);
		ROS_INFO("   3) Gripper MAX OPEN: %3.2f", mask_length);
		ROS_INFO(" Searching Loops: %d", loops);
		
		
		cout<< "Intialize Subscribers"<<endl;
		// Subscribe To Point Cloud Topic
		sub_pc = nhd.subscribe(topic_cloud_name, 200, &FastGrasp::getCloud, this);
		
		cout<< "Intialize Subscribers 2"<<endl;
		// Subscribe to Object Pose Topic
		sub_obj = nhd.subscribe(topic_pose_name, 200, &FastGrasp::getPose, this);
		
		// Set Default Values
		//TODO//
		step = 0.0;
		mask_pose[0] = 0.0;
		mask_pose[1] = 0.0;
		mask_pose[2] = -0.08;
		
		Ident =  MatrixXd::Identity(3, 3);
		Sk2 = v_skew = Rot_z = MatrixXd::Identity(3, 3);
		tf::Vector3 z_axis(0.0, 0.0, 1.0);
	    cout<< "Intializion OK"<<endl;
}


void FastGrasp::getCloud(const sensor_msgs::PointCloud2 input)
{
		// Define CallBack Variables
		pcl::PCLPointCloud2 pcl_pc;
		
		cout << "Just Subscribed to Point Cloud topic"<<endl;
        
        cout << "Convert Input PointCloud2 to pclXYZ"<<endl;
		// covert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::pclXYZ>
		pcl_conversions::toPCL(input, pcl_pc);
		cout << "Transformation"<<endl;
		pcl::fromPCLPointCloud2(pcl_pc, *source_cloud);

		cout<<">> Read Object Point Cloud..."<<endl;
		cout<<"* Object Cloud Size: "<<source_cloud->points.size()<<endl;
}



void FastGrasp::getPose(const geometry_msgs::PoseStamped pose_stamped)
{
		//Convert Pose to Eigen::Affine3d
		geometry_msgs::Pose pp;
		pp = pose_stamped.pose;

		obj_transformation = Eigen::Affine3d(Eigen::Translation3d(pp.position.x, pp.position.y, pp.position.z) * Eigen::Quaterniond(pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z));

		cout<<">> Read Object Pose..."<<endl;
		cout<<"* Object Transformation Matrix:\n"<<obj_transformation.matrix()<<endl;
}


/* Function Usage filterPass(cloud, *result, "z", 0.2, 10); */
void FastGrasp::filterPass(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZ> &result, std::string axis, double min_, double max_)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName (axis);
	pass.setFilterLimits (min_, max_);
	//pass.setFilterLimitsNegative (true);
	pass.filter (result);
}

/* Line or Plane Segmentation of The object */
void FastGrasp::Segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double threshold, std::string type)
{

	pcl::SACSegmentation < pcl::PointXYZ > seg;
	// Create the segmentation object
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	if (type.compare("PLANE"))
	{
		seg.setModelType(pcl::SACMODEL_PLANE);
	}

	else
	{
		seg.setModelType(pcl::SACMODEL_LINE);
	}
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

}

void FastGrasp::ExtractIndices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, boost::shared_ptr<vector<int> > &indicesptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &result)
{
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(indicesptr);
	extract.setNegative(false);
	extract.filter(*result);
}


PointCloudsCentroid FastGrasp::GetConhullContour(){

		//Initalize Variables
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud (new pcl::PointCloud<pcl::PointXYZ>);

		// Create A pointer of the source point cloud 
		/* Not Needed */
		//m_ptrCloud = source_cloud;
		cout<< "GET CONCAVE HULL CONTOUR" <<endl;
		// Transform Point Cloud To object Local Frame
		//pcl::transformPointCloud (*source_cloud, *m_ptrCloud, obj_transformation);
		
		cout<< "Input cloud Transformed to its frmae" <<endl;
		
		cout<< "cloud Size: " <<source_cloud->points.size()<<endl;
		cout<< "cloud Size: " <<m_ptrCloud->points.size()<<endl;
		/* If the cloud is not Empty Begin The Computation */
		if (m_ptrCloud->points.size() > 0.0) 
		{

			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			
		    cout<< "Begin Computations..." <<endl;
			
			//Compute Max And Min Values of the Source PCL
			pcl::getMinMax3D(*m_ptrCloud, src_min, src_max);
			
			cout<< "Filter in Y Axis..." <<endl;
			//Apply filter in y Axis (The filter depends on Gripper Length)
			FastGrasp::filterPass(m_ptrCloud, *cloud_filtered, "y", src_min.y, src_min.y + gripper_length + 0.02);
			
			cout<< "PLANE segmentation of the cloud..." <<endl;
			// Create a PLANE segmentation of the cloud
			FastGrasp::Segmentation(cloud_filtered, coefficients, inliers, 0.02, "PLANE");
			
			cout<< "Project Cloud to the computed PLANE..." <<endl;
			// Project the model inliers to the computed Plane
			proj.setModelType(pcl::SACMODEL_PLANE);
			proj.setIndices(inliers);
			proj.setInputCloud(cloud_filtered);
			proj.setModelCoefficients(coefficients);
			proj.filter(*cloud_projected);

			cout<< "Create Concave Hull of the PROJECTED CLOUD..." <<endl;
			// Create a Concave Hull representation of the projected inliers
			//chull1.setInputCloud(cloud_filtered); //Without Projection
			chull.setInputCloud(cloud_projected);
			chull.setAlpha(0.03);
			chull.reconstruct(*cloud_hull);
		
			cout<< "Filter in X Axis..." <<endl;
			//Apply Filter in X Axis
			//Filters the UN-Graspable Points (MAX GRIPPER OPENING)
			FastGrasp::filterPass(cloud_hull, *cloud_hull, "x", -mask_length/2.0, mask_length/2.0);
		}

		PointCloudsCentroid ret;
		// Return the source point 
		ret.SourcePointCloud = m_ptrCloud;
		// hull of source cloud
		ret.HullPointCloud = cloud_hull;
		// centroid of source cloud
		ret.centroid = centroid;
		return ret;
	}




optGraspVars FastGrasp::OptimalAreasSearching(pcl::PointCloud<pcl::PointXYZ>::Ptr cld) 
{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// Compute the maximum and minimum of point cloud
		pcl::getMinMax3D(*cld, min, max);

		pcl::PointIndices::Ptr inli_l(new pcl::PointIndices), inli_r(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coeff_l(new pcl::ModelCoefficients), coeff_r(new pcl::ModelCoefficients);

		
		std::cerr<< "--------------------------------------"<<endl;
		std::cerr<< "   Calculate Optimal Grasping Areas   "<<endl;
		
		//If object is smaller than gripper
		if (abs(max.z - min.z) < mask_height) {
			cout << ">I cannot Grasp it: Too Small" << endl;
			exit(1);
		}

		//if the height of the object is same with the grippers one
		if (abs(max.z - min.z) == mask_height) {
			loops = 1;
			step = 0.0;
		}

		//Find Step Size
		if (abs(max.z - min.z) > mask_height){
			cout<< ">I can grasp It"<<endl;
			step = (abs(max.z - min.z) - mask_height) / (double)(loops);
		}

		cout<<">Step Size: "<<step<<endl;

		//Initial Mask Pose
		mask_pose[2] = min.z + mask_height / 2.0;

		// Initialize minimum angle + Mask Pose

		double min_angle = 180.0;
		double min_mask_pose = mask_pose[2];
		cout<<">Loops :" <<loops<<endl;

		// Get The source point Cloud 
		src_cloud = cld->makeShared();

		/* Mask left and Right */
		Eigen::Vector4f min_bl(-mask_length / 2.0 + mask_pose[0], -0.1,-mask_height / 2.0 + mask_pose[2], 0.0), 
										max_bl(mask_pose[0], 0.1, mask_height / 2.0 + mask_pose[2],0.0);

		Eigen::Vector4f min_br(-0.00 + mask_pose[0], -0.1,-mask_height / 2.0 + mask_pose[2], 0.0), 
										max_br(mask_length / 2.0 + mask_pose[0], 0.1,mask_height / 2.0 + mask_pose[2], 0.0);
		
		while (mask_pose[2] <= (max.z-mask_height / 2.0)) {

			// Define the mask rectangules
			pcl::getPointsInBox(*cld, min_bl, max_bl, set_l);
			pcl::getPointsInBox(*cld, min_br, max_br, set_r);
			
			if ((set_l.size() > 4) && (set_r.size() > 4)) {
				// Print How many points in the contour
				
				//Create Indices 
				boost::shared_ptr<vector<int> > indicesptr(new vector<int>(set_l));
				boost::shared_ptr<vector<int> > indicesptr2(new vector<int>(set_r));
				
				// Extract the inliers of the left cloud line
				FastGrasp::ExtractIndices(cld,indicesptr,cloud_l);
				
				// Extract the inliers of the right cloud line 
				FastGrasp::ExtractIndices(cld,indicesptr2,cloud_r);
				
				// Compute Lines of the Left and Right Cloud
				//Left
				FastGrasp::Segmentation(cloud_l, coeff_l, inli_l, 0.02, "LINE");
				//Right
				FastGrasp::Segmentation(cloud_r, coeff_r, inli_r, 0.02, "LINE");

				//Compute The angle Between left and Right lines
				tf::Vector3 left_vector(coeff_l->values[3],coeff_l->values[4], coeff_l->values[5]);
				tf::Vector3 right_vector(coeff_r->values[3],coeff_r->values[4], coeff_r->values[5]);
				//normalize vector
				right_vector.normalize();		//
				left_vector.normalize();

				double angle = left_vector.angle(right_vector)* 180.0/ M_PI;
				
				//cout << "Angle Between Two Lines : " << angle << endl;

				if ((((int)(angle*10.0)/10.0) < ((int)(min_angle*10.0))/10.0    ) && (angle < 7.0)) 
				{
					min_angle = angle;
					//optimal angle
					optimal_grasp.angle = angle;
					//indices
					optimal_grasp.left_indicies = set_l;
					optimal_grasp.right_indicies = set_r;
					//mask_dimensions
					optimal_grasp.left_mask.min = min_bl;
					optimal_grasp.left_mask.max = max_bl;
					optimal_grasp.right_mask.min = min_br;
					optimal_grasp.right_mask.max = max_br;
					//mask pose
					optimal_grasp.mask_pose =mask_pose[2];
					//line vector
					optimal_grasp.line_vec = left_vector;
					//cout<<"Min Angle : " << min_angle<<endl;
				}

				else if ((abs(mask_pose[2]) < abs(min_mask_pose)) && (((int)(angle*10.0)/10.0) == ((int)(min_angle*10.0))/10.0 )) 
				{
					optimal_grasp.angle = angle;
					optimal_grasp.left_indicies = set_l;
					optimal_grasp.right_indicies = set_r;

					optimal_grasp.left_mask.min = min_bl;
					optimal_grasp.left_mask.max = max_bl;
					optimal_grasp.right_mask.min = min_br;
					optimal_grasp.right_mask.max = max_br;
					
					min_mask_pose = mask_pose[2];

					//line vector
					optimal_grasp.line_vec = left_vector;

					optimal_grasp.mask_pose = min_mask_pose;
					//cout<<"Min Angle : " << min_angle<<endl;
				}

			}

			// next step
			min_bl[2] +=step;
			max_bl[2] +=step;
			min_br[2] +=step;
			max_br[2] +=step;
			mask_pose[2] += step;
			//cout<<"Mask Pose: "<<mask_pose[2]<<endl;
		}

		// -------------------------------------------------
		//-- Optimized Grasping Areas Are Saved in Structure
		//--------------------------------------------------
		
		// Print Optimized Point 
		cout<< "***********************************\n";
		cout<< ">> Optimal Grasping Area: \n"<<"* Angle: "<<optimal_grasp.angle<<endl;
		cout<< "* Mask Pose: "<<optimal_grasp.mask_pose<<endl;
		cout<< "***********************************\n";
		
		
		boost::shared_ptr < vector<int> > indicesptr1(new vector<int>(optimal_grasp.left_indicies));
		boost::shared_ptr < vector<int> > indicesptr2(new vector<int>(optimal_grasp.right_indicies));
		
		// Extract Indices of the Cloud
		FastGrasp::ExtractIndices(cld, indicesptr1, cloud_cluster2);
		FastGrasp::ExtractIndices(cld, indicesptr2, cloud_cluster1);

		/*
		 * Computing Rotation and Position of the Optimal Grasp Region
		 *             with respect to the object Frame
		*/

		tf::Vector3 z_axis(0.0, 0.0, 1.0);
		tf::Vector3 v = optimal_grasp.line_vec.cross(z_axis);
		double s = v.length();
		double c =  optimal_grasp.line_vec.dot(z_axis);

		//Identity Matrix
		Ident =  MatrixXd::Identity(3, 3);

		//Skew symmetric matrix
		v_skew << 0.0,-v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
		Sk2 =  v_skew * v_skew;
		Rot_z = Ident + v_skew + Sk2*(1.0/(1.0+c));

		/*
		cout<<"--------------------\n";
		cout<<"Rotation Matrix: \n"<< Rot_z<<endl;
		cout<<"---------------------\n";
		Vector3d ea = Rot_z.eulerAngles(0, 1, 2); 
		cout<<"Euler Angles: \n"<<ea*180/M_PI<<endl;
		*/

		// Get Graspable points of source cloud 
		Vector4f min = optimal_grasp.left_mask.min;
		Vector4f max = optimal_grasp.right_mask.max;

		std::vector<int> setGraspPoints;
		pcl::getPointsInBox(*src_cloud, min, max, setGraspPoints);
		boost::shared_ptr < vector<int> > indicesptr(new vector<int>(setGraspPoints));

		FastGrasp::ExtractIndices(src_cloud,indicesptr,src_cloud);

		//compute centroid of the PCL 
		//wrt Object Frame
		pcl::compute3DCentroid(*src_cloud, grasp_pose);
		grasp_pose += centroid;

		//cout<<"Grasp Point WRT Object Pose: \n"<<"x: "<<grasp_pose[0]<<"\ny: "<<grasp_pose[1]<<"\nz:"<<grasp_pose[2]<<"\n";
		optGraspVars ret;
		ret.areas = optimal_grasp;
		ret.left_seg = cloud_cluster1;
		ret.right_seg = cloud_cluster2;
		return ret;

}

GraspPose FastGrasp::getGraspPose()
{
	GraspPose ret;
	ret.p = grasp_pose;
	ret.R = Rot_z;
	return ret;
}

void  FastGrasp::SetGripperHeight(double gr_height){
	mask_height = gr_height;
}

void  FastGrasp::SetGripperMaxOpen(double gr_open){
	mask_length = gr_open;
}

void  FastGrasp::SetSearchingLoops(int loops_){
	loops = loops_;
}

void  FastGrasp::SetObjectName(string name_){
	object_name = name_;
}

void  FastGrasp::SetGripperLength(double gr_length){
	gripper_length = gr_length;
};

















