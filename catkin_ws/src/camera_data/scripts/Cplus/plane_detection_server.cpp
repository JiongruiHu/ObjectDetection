#include "ros/ros.h"
#include "camera_data/PlaneDetection.h"

#include <iostream>
#include <math.h>       /* sqrt*/

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include "pcl_ros/point_cloud.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

class PlaneDetectionServer
{
public:
  PlaneDetectionServer()
  { 
    service_ = n_.advertiseService("/plane_detection", &PlaneDetectionServer::serv_callback, this);
    //pub_ = n_.advertise<sensor_msgs::PointCloud2>("concave_hull", 100);
    ros::Rate loop_rate(10);
  }


  bool serv_callback(camera_data::PlaneDetection::Request  &req,
                     camera_data::PlaneDetection::Response &res)
  { float a, b, c, d;
    PointCloud::Ptr input_cloud (new PointCloud), cloud_filtered (new PointCloud), cloud_projected(new PointCloud);
    PointCloud::Ptr output_pointcloud (new PointCloud), output_pointcloud_2d(new PointCloud);
    
    pcl::fromROSMsg (req.input_cloud, *input_cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud_filtered);


    // Plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    //cout << "number of plane inliers: "<<inliers->indices.size()<<endl;

    /* plane : ax + by + cz + d = 0
     distance from the origin to the plane : D = -d/sqrt(a^2 + b^2 + c^2)*/
    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];

    double D = d / sqrt(pow(a,2) + pow(b, 2) +pow(c,2));
    cout<< "a= "<<a<<" b="<<b<<" c= "<<c <<" d="<<d << endl; 
    

    // filter out the planar points, this gives same points as the for loop below.
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->points.size () << " data points." << std::endl;


    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    
    // filter out the planar points, gives same points as ProjectInliers above.
    /*for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    {   pcl::PointXYZ p, p1;
        p.x = cloud_filtered->points[inliers->indices[i]].x;
        p.y = cloud_filtered->points[inliers->indices[i]].y;
        p.z = cloud_filtered->points[inliers->indices[i]].z;
        output_pointcloud->points.push_back(p);

    } */

    pcl::toROSMsg(*cloud_projected, res.plane_cloud);
    res.height = D; 
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
  ros::init(argc, argv, "plane_detection_server");
  PlaneDetectionServer myPlaneDetectionServer;
  ROS_INFO("Ready to detect a plane.");
  ros::spin();

  return 0;
}
