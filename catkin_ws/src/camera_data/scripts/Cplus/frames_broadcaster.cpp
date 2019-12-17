#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>

using namespace std;

class FrameTransformation{
  tf2::Quaternion q;
  
  public: 
    geometry_msgs::TransformStamped transformStamped;
    FrameTransformation(){}
    void createTransformation(double tx, double ty, double tz, double r, double p, double y, string f, string cf){
      transformStamped.header.frame_id = f;
      transformStamped.child_frame_id = cf;
      transformStamped.transform.translation.x = tx;
      transformStamped.transform.translation.y = ty;
      transformStamped.transform.translation.z = tz;
      q.setRPY(r, p, y);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
        }
  private: 
   

};

int main(int argc, char** argv){
  ros::init(argc, argv, "world_frame_broadcaster");
  ros::NodeHandle node;

  tf2_ros::StaticTransformBroadcaster tfb1;
  tf2_ros::StaticTransformBroadcaster tfb2;

  FrameTransformation trans1, trans2;
  trans1.createTransformation(0.0,0.0,0.0,-1.5707963267948966,0.0, -1.5707963267948966,"ifm3d/camera_link", "ifm3d/camera_optical_link");
  trans2.createTransformation(-0.205,-0.025042,0.85471, 3.1331, -0.000044097, -0.0040,"ifm3d/camera_optical_link", "world");
  /*geometry_msgs::TransformStamped transformStamped1;

  
  transformStamped1.header.frame_id = "ifm3d/camera_optical_link";
  transformStamped1.child_frame_id = "world";
  transformStamped1.transform.translation.x = -0.205;
  transformStamped1.transform.translation.y = -0.025042;
  transformStamped1.transform.translation.z = 0.85471;
  tf2::Quaternion q1;
  q1.setRPY(3.1331, -0.000044097, -0.0040);
  transformStamped1.transform.rotation.x = q1.x();
  transformStamped1.transform.rotation.y = q1.y();
  transformStamped1.transform.rotation.z = q1.z();
  transformStamped1.transform.rotation.w = q1.w();*/

  ros::Rate rate(10.0);
  while (node.ok()){
    trans1.transformStamped.header.stamp = ros::Time::now();
    tfb1.sendTransform(trans1.transformStamped);
    trans2.transformStamped.header.stamp = ros::Time::now();
    tfb2.sendTransform(trans2.transformStamped);


    rate.sleep();
    printf("sending\n");
  }

};