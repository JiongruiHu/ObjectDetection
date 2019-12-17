#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "camera_data/RegisterTwoPointClouds.h"

#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include "pcl/registration/transformation_estimation_2D.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/registration/correspondence_rejection_one_to_one.h"
#include "pcl/registration/icp.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl_conversions/pcl_conversions.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
using namespace Eigen;
using namespace std::chrono;

class RegisterServer
{
public:
  RegisterServer()
  {
    service_ = n_.advertiseService("/register_two_point_clouds", &RegisterServer::serv_callback, this);
  }
  
  bool serv_callback(camera_data::RegisterTwoPointClouds::Request  &req,
                     camera_data::RegisterTwoPointClouds::Response &res)
  { 
    PointCloud::Ptr cloud_template (new PointCloud);
    PointCloud::Ptr cloud_single_cluster (new PointCloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jiongrui/catkin_ws/src/object_data/box.pcd", 
		*cloud_template) == -1) //* load object template file
  	  {
    	  PCL_ERROR ("Couldn't read file object.pcd \n");
  	  }
    pcl::fromROSMsg (req.single_cluster, *cloud_single_cluster);
    

    // register points in x,y plane, assign zero to all z coordinates.
    for (std::size_t i = 0; i < cloud_single_cluster->points.size(); ++i)
		{	cloud_single_cluster->points[i].z =0.0f;
    	}

    double prev_norm_err= 1.0, new_norm_err = 1.0;
	  int iter = 0, convgCount = 0;
	  bool convg = true;
    double prop;
    
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
      PointCloud::Ptr cloud_out_transformed (new PointCloud);

      pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      pcl::registration::CorrespondenceRejectorOneToOne rejector_one_to_one;
      pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, float> icp2d;

      est.setInputSource (cloud_template);
      est.setInputTarget(cloud_single_cluster);
      est.determineCorrespondences (corresps, 0.5f);

      rejector_one_to_one.getRemainingCorrespondences(corresps, corresps_result);
      for (auto t: corresps_result)
        {	
          cloud_template_match->points.push_back(cloud_template->points[t.index_query]);
          cloud_out_match->points.push_back(cloud_single_cluster->points[t.index_match]); 
       }
    
    //estimate the rigid transformation between the template cloud and target cloud in x,y plane 
		icp2d.estimateRigidTransformation(*cloud_template_match, *cloud_out_match, trans);		
    rotation_matrix = trans.block(0,0,2,2);
    t_vector = trans.col(3).head(2);
          
    double err = 0.0;
    VectorXf X_in_vector, Y_in_vector, X_out_vector, Y_out_vector;

    for (size_t i = 0; i < cloud_template_match->points.size(); ++i)
    {	
    
      Vector2f cloud_template_vector, p_vector, pR_vector, err_vector;
      cloud_template_vector[0]= cloud_template_match->points[i].x;
      cloud_template_vector[1] = cloud_template_match->points[i].y;
      
      p_vector[0] = cloud_out_match->points[i].x;
      p_vector[1]= cloud_out_match->points[i].y;

      pR_vector = rotation_matrix * p_vector; 
      err_vector = cloud_template_vector - pR_vector - t_vector;
      err = err + sqrt(pow(err_vector[0],2) + pow(err_vector[1],2));
    }

    new_norm_err = err/ cloud_template_match->points.size();
    // transform the target cloud with the estimated rigid transformation(rotation and translation )
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
    cloud_single_cluster->points.clear();
    cloud_single_cluster = cloud_out_transformed;
    ++iter;
    //cout << "number of iteration "<< iter <<endl;
    //prop = double(cloud_out_match->points.size())/cloud_template->points.size();
    prop = double(cloud_out_match->points.size())/cloud_single_cluster->points.size();

		} //end of while loop

    if (iter > 10 || prop < 0.85) 
			{	convg = false;		
			}
    cout<< "matched prop "<< prop <<boolalpha<< convg <<endl;
    res.match = convg;
    return true;
  }


  private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceServer service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_register_server");
  RegisterServer myRegisterSever;
  ROS_INFO("Ready to align point clouds.");
  ros::spin();
  return 0;
}