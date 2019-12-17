#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>




void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //cloud = cloud_msg;
  pcl::fromROSMsg(*cloud_msg, *cloud);
  std::cout <<"Cloud height: "<< cloud->height<< endl;
  std::cout <<"Cloud width: "<< cloud->width<< endl;

  // estimatee the normals of all the points  
  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  /*pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize (10.0f);
  ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
  ne.setInputCloud (cloud);
  ne.compute (*normal);*/

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.02);
  ne.compute (*normal);

  //edge detetion
  pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
  oed.setInputNormals (normal);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (0.02f);
  oed.setMaxSearchNeighbors (50);
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE );
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute (labels, label_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZ>),
                                      occluded_edges (new pcl::PointCloud<pcl::PointXYZ>),
                                      boundary_edges (new pcl::PointCloud<pcl::PointXYZ>),
                                      high_curvature_edges (new pcl::PointCloud<pcl::PointXYZ>);
    
  pcl::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
  pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
  pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
  pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);

  std::cout<<"boundary edges: "<< boundary_edges->size()<<endl;
  std::cout<< "high_curvature_edges"<<high_curvature_edges->size()<<endl;
  std::cout<<"occuled edges: "<< occluded_edges->size()<<endl;
  std::cout<<"occluding edges: "<< occluding_edges->size()<<endl;


   // Display edges in PCLVisualizer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setSize (640, 480);
  viewer->addCoordinateSystem (0.2f, "global");
  //viewer.registerKeyboardCallback(&keyboard_callback);
  const int point_size = 2;
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "occluding edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "occluding edges");

  /*viewer->addPointCloud<pcl::PointXYZ> (occluded_edges, "occluded edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges");

  viewer->addPointCloud<pcl::PointXYZ> (high_curvature_edges, "high curvature edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges");
*/

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100);
    
  }  

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "edge_detection");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("top_layer_cloud", 1000, callback);
  ros::spin();
  return (0);
}
