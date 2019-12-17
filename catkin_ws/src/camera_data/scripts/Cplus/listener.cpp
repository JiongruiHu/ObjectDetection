#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

using namespace std;
using namespace geometry_msgs;


class Listener{
  public:
      Listener(ros::NodeHandle node){
        n = node;
      }
    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
     {
      TransformStamped trans = get_transformation();
      sensor_msgs::PointCloud2 cloud_msg_trans; 
      tf2::doTransform(*cloud_msg, cloud_msg_trans, trans);
      pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
      pcl::fromROSMsg (cloud_msg_trans, cloud_pcl);
      pcl::io::savePCDFileASCII ("/home/jiongrui/catkin_ws/src/Test/build/camera_cloud_3.pcd", cloud_pcl);

     }
  private:
    ros::NodeHandle n;
    TransformStamped get_transformation(string tFrame = "world", string sFrame = "ifm3d/camera_link")
    { tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      TransformStamped transformStamped;
      while (n.ok())
      {  
         try{
            transformStamped = tfBuffer.lookupTransform(tFrame, sFrame, ros::Time(0));}
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;} 
      }
      return transformStamped;

    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  Listener myListener(n); 
  ros::Subscriber sub = n.subscribe("ifm3d/camera/cloud", 1000, &Listener::callback, &myListener);
  ros::spin();

  return 0;
}