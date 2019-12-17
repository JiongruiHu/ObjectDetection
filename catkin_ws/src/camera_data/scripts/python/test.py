#!/usr/bin/env python
import pcl
import numpy as np
import open3d
from scipy.spatial import ConvexHull


#source = open3d.io.read_point_cloud("/home/jiongrui/catkin_ws/src/object_data/cluster1.pcd")
template = open3d.io.read_point_cloud("/home/jiongrui/catkin_ws/src/Test/build/camera_cloud_3.pcd")
#template.paint_uniform_color([0.1, 0.1, 0.7])
#open3d.visualization.draw_geometries([source,template])
open3d.visualization.draw_geometries([template])



