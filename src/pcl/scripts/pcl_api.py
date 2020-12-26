#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import pcl
#import pcl_helper
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct


class pcl_helper:
	def __init__(self):
		pass

	def voxel_filter(self, cloud, leaf_sizes):
		"""
		Input parameters:
		cloud: input point cloud to be filtered
		leaf_sizes: a list of leaf_size for X, Y, Z 
		Output:
		cloud_voxel_filtered: voxel-filtered cloud
		"""
		sor = cloud.make_voxel_grid_filter()
		size_x, size_y, size_z = leaf_sizes
		sor.set_leaf_size(size_x, size_y, size_z)
		self.cloud_voxel_filtered = sor.filter()
		
		return cloud_voxel_filtered

	def roi_filter(self, cloud, x_roi, y_roi, z_roi):
		"""
		Input Parameters:
			cloud: input point cloud
			x_roi: ROI range in X
			y_roi: ROI range in Y
			z_roi: ROI range in Z
		
		Output:    
			ROI region filtered point cloud
		"""
		clipper = cloud.make_cropbox()
		self.cloud_roi_filtered= pcl.PointCloud()
		xc_min, xc_max = x_roi
		yc_min, yc_max = y_roi
		zc_min, zc_max = z_roi
		clipper.set_MinMax(xc_min, yc_min, zc_min, 0, xc_max, yc_max, zc_max, 0)
		cloud_roi_filtered =clipper.filter()
		return cloud_roi_filtered

	def plane_segmentation(self, cloud, dist_thold, max_iter):
		"""
		Input parameters:
			cloud: Input cloud
			dist_thold: distance threshold
			max_iter: maximal number of iteration
		Output:
			indices: list of indices of the PCL points that belongs to the plane
			coefficient: the coefficients of the plane-fitting (e.g., [a, b, c, d] for ax + by +cz + d =0)
		"""
		seg = cloud.make_segmenter_normals(ksearch=50)# For simplicity,hard coded
		seg.set_optimize_coefficients(True)
		seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_distance_threshold(dist_thold)
		seg.set_max_iterations(max_iter)
		indices, coefficients = seg.segment()
		return indices, coefficients

	def clustering(self, cloud, tol, min_size, max_size):
		"""
		Input parameters:
			cloud: Input cloud
			tol: tolerance
			min_size: minimal number of points to form a cluster
			max_size: maximal number of points that a cluster allows 
		Output:
			cluster_indices: a list of list. Each element list contains the indices of the points that belongs to
							 the same cluster
		"""
		tree = cloud.make_kdtree()
		ec = cloud.make_EuclideanClusterExtraction()
		ec.set_ClusterTolerance(tol)
		ec.set_MinClusterSize(min_size)
		ec.set_MaxClusterSize(max_size)
		ec.set_SearchMethod(tree)
		self.cluster_indices = ec.Extract()
		return cluster_indices

	def get_cluster_box_list(self, cluster_indices, cloud_obsts):
		"""
		Input parameters:
			cluster_indices: a list of list. Each element list contains the indices of the points that belongs to
							 the same cluster
			colud_obsts: PCL for the obstacles                 
		Output:
			cloud_cluster_list: a list for the PCL clusters: each element is a point cloud of a cluster
			box_coord_list: a list of corrdinates for bounding boxes
		"""    
		cloud_cluster_list =[]
		box_coord_list =[]
		

		for j, indices in enumerate(cluster_indices):
			points = np.zeros((len(indices), 3), dtype=np.float32)
			for i, indice in enumerate(indices):
				
				points[i][0] = cloud_obsts[indice][0]
				points[i][1] = cloud_obsts[indice][1]
				points[i][2] = cloud_obsts[indice][2]
			cloud_cluster = pcl.PointCloud()
			cloud_cluster.from_array(points)
			cloud_cluster_list.append(cloud_cluster)
			x_max, x_min = np.max(points[:, 0]), np.min(points[:, 0])
			y_max, y_min = np.max(points[:, 1]), np.min(points[:, 1])
			z_max, z_min = np.max(points[:, 2]), np.min(points[:, 2])
			box = np.zeros([8, 3])
			box[0, :] =[x_min, y_min, z_min]
			box[1, :] =[x_max, y_min, z_min]
			box[2, :] =[x_max, y_max, z_min]
			box[3, :] =[x_min, y_max, z_min]
			box[4, :] =[x_min, y_min, z_max]
			box[5, :] =[x_max, y_min, z_max]
			box[6, :] =[x_max, y_max, z_max]
			box[7, :] =[x_min, y_max, z_max]
			box = np.transpose(box)
			box_coord_list.append(box)
		return cloud_cluster_list, box_coord_list  

	def box_center(self, box):
		"""
		Calculate the centroid of a 3D bounding box
		Input: box, a 3-by-8 matrix, each coloum represents the xyz coordinate of a corner of the box
			   (e.g.  
			   array([[42.62581635, 46.09998703, 46.09998703, 42.62581635, 42.62581635, 46.09998703, 46.09998703, 42.62581635],
					  [2.64766479,  2.64766479,  4.64661026,  4.64661026,  2.64766479, 2.64766479,  4.64661026,  4.64661026],
					  [0.10515476,  0.10515476,  0.10515476,  0.10515476,  1.98793995, 1.98793995,  1.98793995,  1.98793995]])
			   )
		Output: the centroid of the box in 3D [x_cent, y_cent, z_cent]
		"""
		x_min, x_max = min(box[0]), max(box[0])
		y_min, y_max = min(box[1]), max(box[1])
		z_min, z_max = min(box[2]), max(box[2])
		
		return ((x_min + x_max)/2.0, (y_min + y_max)/2.0, (z_min + z_max)/2.0)

	def ros_to_pcl(self, ros_cloud):
		""" Converts a ROS PointCloud2 message to a pcl PointXYZRGB

			Args:
				ros_cloud (PointCloud2): ROS PointCloud2 message

			Returns:
				pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
		"""
		pc = pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z"))
		pc_list = []
		for p in pc:
			pc_list.append( [p[0],p[1],p[2]] )

		p = pcl.PointCloud()
		p.from_list(pc_list)
		return p

	def pcl_to_ros(self, pcl_array):
		""" Converts a pcl PointXYZRGB to a ROS PointCloud2 message
		
			Args:
				pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
				
			Returns:
				PointCloud2: A ROS point cloud
		"""
		ros_msg = PointCloud2()

		ros_msg.header.stamp = rospy.Time.now()
		ros_msg.header.frame_id = "camera_link"

		ros_msg.height = 1
		ros_msg.width = pcl_array.size #len(pcl_array) #pcl_array.size

		ros_msg.fields.append(PointField(
								name="x",
								offset=0,
								datatype=PointField.FLOAT32, count=1))
		ros_msg.fields.append(PointField(
								name="y",
								offset=4,
								datatype=PointField.FLOAT32, count=1))
		ros_msg.fields.append(PointField(
								name="z",
								offset=8,
								datatype=PointField.FLOAT32, count=1))
		ros_msg.fields.append(PointField(
								name="rgb",
								offset=16,
								datatype=PointField.FLOAT32, count=1))

		ros_msg.is_bigendian = False
		ros_msg.point_step = 32
		ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
		ros_msg.is_dense = False
		buffer = []
		for data in pcl_array:
			s = struct.pack('>f', data[1])
			i = struct.unpack('>l', s)[0]
			pack = ctypes.c_uint32(i).value

			r = (pack & 0x00FF0000) >> 16
			g = (pack & 0x0000FF00) >> 8
			b = (pack & 0x000000FF)

			buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

		ros_msg.data = "".join(buffer)
		#print(ros_msg)

		return ros_msg











