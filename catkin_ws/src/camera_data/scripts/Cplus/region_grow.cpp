#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  std::cout <<"Cloud height: "<< cloud->height<< endl;
  std::cout <<"Cloud width: "<< cloud->width<< endl;
 

  //for (std::size_t i = 0; i < cloud->points.size (); ++i)
//			{std::cout<< cloud->points[i].z <<endl;}


  //pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

  /*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);*/

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 ne.setSearchMethod (tree);
 ne.setRadiusSearch (0.02);
 ne.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (300);
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (1.8 / 180.0 * M_PI);
  reg.setCurvatureThreshold (0.8);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
    for (std::size_t i = 0; i < clusters.size (); ++i)
		{  std::cout << "The cluster has " << clusters[i].indices.size () << " points." << std::endl;
}

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
  /*
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;*/
   //view the normals
/*  int v1(0);
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer->setBackgroundColor (0.0, 0.0, 0.5);
viewer->createViewPort (0.0, 0.0, 0.0, 1.0, v1);
viewer->initCameraParameters ();
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color,"sample cloud");

viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.02, "normals1");*/

if (clusters.size() >1){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    viewer->addPointCloud(colored_cloud);
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.02, "normals1");

    while (!viewer->wasStopped ())
    {viewer->spinOnce();
    }
} 



}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "region_grow");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("top_layer_cloud", 1000, callback);
  ros::spin();
  return (0);
}
