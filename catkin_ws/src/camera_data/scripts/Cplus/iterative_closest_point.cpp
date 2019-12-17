#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/icp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cstring>
#include <cmath>
#include <thread>
 #include <chrono>
//#include "matplotlibcpp.h"

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


int main (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model_match (new pcl::PointCloud<pcl::PointXYZ>);
		PointCloud::Ptr cloud_model (new PointCloud);


  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out0 (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1 (new pcl::PointCloud<pcl::PointXYZ>);
		PointCloud::Ptr cloud_out0 (new PointCloud);
		PointCloud::Ptr cloud_out1 (new PointCloud);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_match (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_transformed (new pcl::PointCloud<pcl::PointXYZ>);



	// setup ros
	/*
 	ros::init(argc, argv, "cloud_match_points");
	ros::NodeHandle n;
	ros::Publisher cloud_pub = n.advertise<geometry_msgs::Points>("/camera/cloud_match", 100);
	*/	
	int count = 0;

	 if (pcl::io::loadPCDFile<pcl::PointXYZ> ("water.pcd", *cloud_model) == -1) //* load object template file
  	{
    	PCL_ERROR ("Couldn't read file object.pcd \n");
    	return (-1);
  	}
		cout<< "Loaded "
				//<< cloud_model->width * cloud_model->height //it is the same as points.size()
				<< "size of cloud:" << cloud_model->points.size()
				<< endl;

		for (std::size_t i = 0; i < cloud_model->points.size (); ++i)
			{cloud_model->points[i].z = 0.0f;
    	//cout << "    " << cloud_model->points[i].x
      //     << " "    << cloud_model->points[i].y
      //     << " "    << cloud_model->points[i].z << endl;
			}
		int convgCount = 0;
	ifstream watertemplate, target;
	
	/*watertemplate.open("aquador2d.dat");

	while (watertemplate.good() && !watertemplate.eof())
	{
		double x, y, z;
		//watertemplate >> x >> y>> z;  //3d
		watertemplate >> x >> y; //2d
		pcl::PointXYZ p;
		p.x = x; 
		p.y = y;
 		//p.z = z; 
		p.z = 0.0f; // for 2d data
		cloud_model->points.push_back(p);
	}
	watertemplate.close();
	cout << "Saved " << cloud_model->points.size () << " data points to input:" << endl;*/
	
	//target.open("water/3d/water0.dat");

	for (int i = 0; i< 10; ++i)
	{
		std::stringstream ss;
		ss << i;
		target.open("water/2d/water"+ss.str()+".dat");
		bool convg = true;
		cout<<"loading data from water"<<ss.str()<<".dat"<<endl;
		while (target.good() && !target.eof())
		{
			double x, y, z;
			target >> x >> y; // 2d
			//target >> x >> y>> z; // 3d
			pcl::PointXYZ p;
			p.x = x; 
			p.y = y;
	 		//p.z = z;//3d
			p.z = 0.0f; 
			cloud_out0->points.push_back(p);
		}
		target.close();
		cout<<"processing water"<<ss.str()<<".dat"<<" has "<< cloud_out0->points.size() <<" datas"<<endl;

		/*//filter the 3dpoints 
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (cloud_out0);
		seg.segment (*inliers, *coefficients);
		
		// public pcl_cloud	after filtering 
		PointCloud::Ptr filtered_cloud_out (new PointCloud);

		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			filtered_cloud_out->points.push_back(cloud_out0->points[inliers->indices[i]]);
		}*/


		double prev_norm_err= 1.0;
		double new_norm_err = 1.0;
		int iter = 0;
		//cout<< prev_norm_err<< new_norm_err<<endl;
		while (new_norm_err <= prev_norm_err && new_norm_err > 1e-5)
		{	prev_norm_err = new_norm_err;
			Matrix4f trans;
			Matrix2f rotation_matrix;
			Vector2f t_vector;
			// find correspondence pair
			pcl::Correspondences corresps;
			pcl::Correspondences corresps_result;

			PointCloud::Ptr cloud_model_match (new PointCloud);
			PointCloud::Ptr cloud_out_match (new PointCloud);

			pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
			est.setInputSource (cloud_model);
			//est.setInputTarget (filtered_cloud_out); //for 3d data
			est.setInputTarget(cloud_out0); //for 2d data
			est.determineCorrespondences (corresps, 0.5f);
			//est.determineReciprocalCorrespondences (corresps, 0.1f);
			//result: duplicate target matches;

			pcl::registration::CorrespondenceRejectorOneToOne rejector_one_to_one;
			rejector_one_to_one.getRemainingCorrespondences(corresps, corresps_result);
			//cout<< "iteration: "<< iter+1<<endl;
			for (auto t: corresps_result)
			{	
				cloud_model_match->points.push_back(cloud_model->points[t.index_query]);
				cloud_out_match->points.push_back(cloud_out0->points[t.index_match]); //2d
				//cloud_out_match->points.push_back(filtered_cloud_out->points[t.index_match]); //3d
			}

			//cout<<"Matched "<< cloud_out_match->points.size()<<" points"<<endl;

		
			// Visulization
			pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
			//		pcl::visualization::PCLVisualizer::Ptr viewer ("ICP Viewer");
			// Create two vertiaclly separated viewports
			int v1(0);
			int v2(1);
	  	viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	  	viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
			float bckgr_gray_level = 0.0;  // Black
	  	float txt_gray_lvl = 1.0 - bckgr_gray_level;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in (cloud_model, 0, 255, 0);//green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_out (cloud_out0, 255, 255, 0);//yellow, 2d
			viewer->addPointCloud(cloud_model, color_in, "cloud_model", v1);
			viewer->addPointCloud(cloud_out0, color_out, "cloud_out0", v1); // 2d
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_out (filtered_cloud_out, 255, 255, 0);//yellow, for 3d data
			viewer->setBackgroundColor(0, 0, 0, v1);
			viewer->setBackgroundColor(0, 0, 0, v2);	
			
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in_match (cloud_model_match, 0, 255, 0);//green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_out_match (cloud_out_match, 255, 255, 0);//yellow, 2d
			viewer->addPointCloud(cloud_model_match, color_in_match, "cloud_model_match", v2);
			viewer->addPointCloud(cloud_out_match, color_out_match, "cloud_out_match", v2);
			//viewer->addPointCloud(filtered_cloud_out, color2, "out0"); // 3d
			//viewer->addPointCloud(cloud_model_match, color1, "in");
			//viewer->addPointCloud(cloud_out_match, color2, "out0");
			//viewer->addPointCloud(cloud_out1, color3, "out1");	
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_model");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_out0");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_model_match");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_out_match");
			//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "out1");
			//viewer->addCoordinateSystem ();
			//viewer->initCameraParameters ();		
			//viewer->setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
			//viewer->setCameraPosition (-3.68332, 2.94092, 2.71266, 0.0, 0.0, 0.0, 0);
	  	viewer->setSize (512, 256);  // Visualiser window size
			viewer->addText ("Green: model point cloud\nYellow: to be matched point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"original", v1);
	  viewer->addText ("	After correspondence match", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"after match", v2);
			while (!viewer->wasStopped ())
			{
			viewer->spinOnce (100);
				this_thread::sleep_for(milliseconds(100));
			}	
			 //end of visualization

			// find transformation matrix in 2D, use 2d data
			pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ, float> icp2d;
			icp2d.estimateRigidTransformation(*cloud_model_match, *cloud_out_match, trans);

		
			rotation_matrix = trans.block(0,0,2,2);
			t_vector = trans.col(3).head(2);
		  	
			// error = sum(xi- pi*R -t)/N	
	  
  			double err = 0.0;
			VectorXf X_in_vector, Y_in_vector, X_out_vector, Y_out_vector;

		  	for (size_t i = 0; i < cloud_model_match->points.size(); ++i)
		  	{	
				
					Vector2f cloud_model_vector, p_vector,pR_vector,err_vector;

					cloud_model_vector[0]= cloud_model_match->points[i].x;
					cloud_model_vector[1] = cloud_model_match->points[i].y;
				 
					p_vector[0] = cloud_out_match->points[i].x;
					p_vector[1]= cloud_out_match->points[i].y;

					pR_vector = rotation_matrix * p_vector; 
					err_vector = cloud_model_vector - pR_vector - t_vector;
					err = err + sqrt(pow(err_vector[0],2) + pow(err_vector[1],2));
			  }
				//printf ("test1\n");

  			new_norm_err = err/ cloud_model_match->points.size();
			cout <<"number of matched out points:"<< cloud_out_match->points.size()<<endl;
			/*cout <<"error: "<<err<<endl;
			cout <<"number of matched model points:"<< cloud_model_match->points.size()<<endl;
			cout <<"number of matched out points:"<< cloud_out_match->points.size()<<endl;
			cout << "prev normalized error: "<< prev_norm_err<< endl; 
			cout << "new normalized error: "<< new_norm_err<< endl; */
					
			// new target cloud after the transformation
			//cout << cloud_out0->points.size()<<endl; 

			PointCloud::Ptr cloud_out_transformed (new PointCloud);
			for (size_t i = 0; i<cloud_out0->points.size(); i++)
			{	pcl::PointXYZ p;

				Vector2f vector1, vector2;
				vector2[0] = cloud_out0->points[i].x;
				vector2[1] = cloud_out0->points[i].y;
				vector1 = rotation_matrix * vector2 + t_vector;
				p.x = vector1[0];
				p.y = vector1[1];
				//p.z = 0.0f;
				cloud_out_transformed->points.push_back(p);
				
			}
			cloud_out0 = cloud_out_transformed;

			//cout << "number of cloud_out"<< cloud_out_transformed->points.size()<<endl;	  
			++iter;
		
			/*// using 3d data
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

			icp.setMaximumIterations (10);
	  		icp.setInputSource(cloud_model_match);
	  		icp.setInputTarget(cloud_out_match);
	  		pcl::PointCloud<pcl::PointXYZ> Final;
			icp.align(*cloud_out_match);
			transformation_matrix *= icp.getFinalTransformation ().cast<double>();
	  		print4x4Matrix (transformation_matrix);
	  		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	  		icp.getFitnessScore() << std::endl;
			cout<< icp.getFinalTransformation ()<<endl;*/
			if (iter > 10 || double(cloud_out_match->points.size())/cloud_model->points.size() < 0.5) 
			{	convg = false;
				cout<< "matched prop "<< double(cloud_out_match->points.size())/cloud_model->points.size() 
					<<boolalpha<< convg << endl;
				 break;
				
			}
			
		} //end of while loop
		cloud_out0->points.clear();


		if (convg==true)
			 ++convgCount;
	}
	cout<<"total matched : "<< convgCount<<endl;
	 return (0);
}

