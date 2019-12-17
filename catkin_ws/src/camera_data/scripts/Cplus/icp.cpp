#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/registration/transformation_estimation_2D.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/correspondence_rejection_one_to_one.h"
#include "pcl/registration/icp.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/ModelCoefficients.h"



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using  namespace  std;
using  namespace  Eigen;
using namespace std::chrono;

void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.4f %6.4f %6.4f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.4f %6.4f %6.4f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.4f %6.4f %6.4f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.4f, %6.4f, %6.4f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void pcloud_CallBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	PointCloud::Ptr cloud_template (new PointCloud);
	//PointCloud::Ptr cloud_out0 (new PointCloud);
	PointCloud::Ptr cloud_single_cluster (new PointCloud);
	PointCloud::Ptr pcl_cloud (new PointCloud);
  	pcl::fromROSMsg (*msg, *cloud_single_cluster);
	pcl::io::savePCDFile ("/home/jiongrui/catkin_ws/src/object_data/single_cluster.pcd", *cloud_single_cluster);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jiongrui/catkin_ws/src/object_data/water.pcd", 
		*cloud_template) == -1) //* load object template file
  	{
    	PCL_ERROR ("Couldn't read file object.pcd \n");
			//return (-1);
  	}
	cout<< "Size of the template cloud has :" << cloud_template->points.size()<<" points"<< endl;
	cout<< "Size of the single cluster has :" << cloud_single_cluster->points.size()<<" points"<< endl;
	
	for (std::size_t i = 0; i < cloud_single_cluster->points.size(); ++i)
		{	cloud_single_cluster->points[i].z= 0.0f;
    	}

		//for (std::size_t i = 0; i < 3; ++i)
		//	{cout<< cloud_template->points[i].z<<endl;}

	double prev_norm_err= 1.0, new_norm_err = 1.0;
	int iter = 0, convgCount = 0;
	bool convg = true;

	while (new_norm_err <= prev_norm_err && new_norm_err > 1e-5)

		{	prev_norm_err = new_norm_err;
			Matrix4f trans;
			Matrix2f rotation_matrix;
			Vector2f t_vector;
			// find correspondence pair
			pcl::Correspondences corresps;
			pcl::Correspondences corresps_result;
			PointCloud::Ptr cloud_template_match (new PointCloud);
			PointCloud::Ptr cloud_out_match (new PointCloud);

			pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
			est.setInputSource (cloud_template);
			est.setInputTarget(cloud_single_cluster); //for 2d data
			est.determineCorrespondences (corresps, 0.5f);

			pcl::registration::CorrespondenceRejectorOneToOne rejector_one_to_one;
			rejector_one_to_one.getRemainingCorrespondences(corresps, corresps_result);
			for (auto t: corresps_result)
			{	
				cloud_template_match->points.push_back(cloud_template->points[t.index_query]);
				cloud_out_match->points.push_back(cloud_single_cluster->points[t.index_match]); //2d
			}
			pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, float> icp2d;
			icp2d.estimateRigidTransformation(*cloud_template_match, *cloud_out_match, trans);

		
			rotation_matrix = trans.block(0,0,2,2);
			t_vector = trans.col(3).head(2);
		  		  
  			double err = 0.0;
			VectorXf X_in_vector, Y_in_vector, X_out_vector, Y_out_vector;

		  	for (size_t i = 0; i < cloud_template_match->points.size(); ++i)
		  	{	
				
					Vector2f cloud_template_vector, p_vector,pR_vector,err_vector;

					cloud_template_vector[0]= cloud_template_match->points[i].x;
					cloud_template_vector[1] = cloud_template_match->points[i].y;
				 
					p_vector[0] = cloud_out_match->points[i].x;
					p_vector[1]= cloud_out_match->points[i].y;

					pR_vector = rotation_matrix * p_vector; 
					err_vector = cloud_template_vector - pR_vector - t_vector;
					err = err + sqrt(pow(err_vector[0],2) + pow(err_vector[1],2));
			  }
				//printf ("test1\n");

  			new_norm_err = err/ cloud_template_match->points.size();
			cout <<"number of matched out points:"<< cloud_out_match->points.size()<<endl;

			PointCloud::Ptr cloud_out_transformed (new PointCloud);
			for (size_t i = 0; i<cloud_single_cluster->points.size(); i++)
			{	pcl::PointXYZ p;

				Vector2f vector1, vector2;
				vector2[0] = cloud_single_cluster->points[i].x;
				vector2[1] = cloud_single_cluster->points[i].y;
				vector1 = rotation_matrix * vector2 + t_vector;
				p.x = vector1[0];
				p.y = vector1[1];
				cloud_out_transformed->points.push_back(p);				
			}
			cloud_single_cluster = cloud_out_transformed;
			++iter;
			
			if (iter > 10 || double(cloud_out_match->points.size())/cloud_template->points.size() < 0.5) 
			{	convg = false;
				cout<< "matched prop "<< double(cloud_out_match->points.size())/cloud_template->points.size() 
					<<boolalpha<< convg <<endl;
				 break;
				printf("i am here");
			}
			cout <<"number of transformed:"<< cloud_out_transformed->points.size()<<endl;

			
		} //end of while loop
	cloud_single_cluster->points.clear();
	if (convg == true)
		++convgCount;
	cout<<"total matched : "<< convgCount<<endl;



	// public pcl_cloud	after filtering 
	/*PointCloud::Ptr pcl_cloud_plane (new PointCloud);
	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		pcl_cloud_plane->points.push_back(pcl_cloud->points[inliers->indices[i]]);
	}

	pcl_cloud_plane->header.frame_id = "ifm3d/camera_link";
	ros::NodeHandle np;
	ros::Publisher pub = np.advertise<PointCloud> ("plane_cloud", 10);
  	ros::Rate loop_rate(10);

	while (np.ok())
	{
		pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_plane->header.stamp);
		pub.publish(pcl_cloud_plane);
		ros::spinOnce();
		loop_rate.sleep();
	}*/

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp");
	ros::NodeHandle n;
	//ros::Rate rate(30); 
	//ros::Subscriber pcl_sub = n.subscribe("/ifm3d/camera/cloud", 10, pcloud_CallBack);
	ros::Subscriber pcl_sub = n.subscribe("/toplayer_cloud", 10, pcloud_CallBack);
	ros::spin();
	return 0;
}

