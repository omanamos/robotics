/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: table_object_detector.cpp 36637 2011-04-20 01:33:25Z michael.s.dixon $
 *
 */

/**

\author Radu Bogdan Rusu

@b table_object_detector detects tables and objects.

 **/

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>
// ROS messages
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include "cse481/table_cluster_detector.h"
#include "cse481/marker_generator.h"

TableClusterDetector::TableClusterDetector ()
{
  // ---[ Create all PCL objects and set their parameters
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("labels", 10);

  // Filtering parameters
  downsample_leaf_ = 0.01;                          // 1cm voxel size by default
  downsample_leaf_objects_ = 0.003;                 // 3mm voxel size by default
  grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);
  grid_objects_.setLeafSize (downsample_leaf_objects_, downsample_leaf_objects_, downsample_leaf_objects_);
  grid_.setFilterFieldName ("z");
  pass_.setFilterFieldName ("z");

  min_z_bounds_ = 0.4;                            // restrict the Z dimension between 0.4m
  max_z_bounds_ = 1.6;                            // and 1.6m
  //nh_.getParam ("min_z_bounds", min_z_bounds_);
  //nh_.getParam ("max_z_bounds", max_z_bounds_);
  grid_.setFilterLimits (min_z_bounds_, max_z_bounds_);
  pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);

  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_->setEpsilon (1);

  // Normal estimation parameters
  k_ = 10;                                // 50 k-neighbors by default
  //nh_.getParam ("search_k_closest", k_);
  n3d_.setKSearch (k_);
  //n3d_.setRadiusSearch (0.015);
  n3d_.setSearchMethod (normals_tree_);

  // Table model fitting parameters
  sac_distance_threshold_ = 0.1;               // 5cm
  //nh_.getParam ("sac_distance_threshold", sac_distance_threshold_);
  seg_.setDistanceThreshold (sac_distance_threshold_);
  seg_.setMaxIterations (10000);

  normal_distance_weight_ = 0.1;
  //nh_.getParam ("normal_distance_weight", normal_distance_weight_);
  seg_.setNormalDistanceWeight (normal_distance_weight_);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

  // Consider objects starting at 1cm from the table and ending at 0.5m
  object_min_height_ = 0.02;
  object_max_height_ = 0.5;
  //nh_.getParam ("object_min_height", object_min_height_);
  //nh_.getParam ("object_max_height", object_max_height_);
  prism_.setHeightLimits (object_min_height_, object_max_height_);

  // Clustering parameters
  object_cluster_tolerance_ = 0.05;        // 5cm between two objects
  object_cluster_min_size_  = 100;         // 100 points per object cluster
  //nh_.getParam ("object_cluster_tolerance", object_cluster_tolerance_);
  //nh_.getParam ("object_cluster_min_size", object_cluster_min_size_);
  cluster_.setClusterTolerance (object_cluster_tolerance_);
  cluster_.setMinClusterSize (object_cluster_min_size_);
  cluster_.setSearchMethod (clusters_tree_);

}

std::vector<PointCloud> TableClusterDetector::findTableClusters(const sensor_msgs::PointCloud2 &scene)
{

  std::vector<PointCloud> clusters;

  // Convert the dataset
  PointCloud cloud; PointCloud::Ptr cloudPtr;
  pcl::fromROSMsg (scene, cloud);
  cloudPtr.reset(new PointCloud(cloud));

  // Remove NaNs
  PointCloud cloud_filtered;
  pass_.setInputCloud (cloudPtr);
  pass_.filter (cloud_filtered);
  cloudPtr.reset(new PointCloud(cloud_filtered));

  // Downsample
  PointCloud cloud_downsampled;
  grid_.setInputCloud (cloudPtr);
  grid_.filter (cloud_downsampled);
  cloudPtr.reset(new PointCloud(cloud_downsampled));

  if ((int)cloud_filtered.points.size() < k_)
  {
    ROS_WARN("Filtering returned %zd points! Skipping.", cloud_filtered.points.size());
    return clusters;
  }

  // Estimate the point normals
  pcl::PointCloud<pcl::Normal> cloud_normals;pcl::PointCloud<pcl::Normal>::Ptr cloud_normalsPtr;
  // add this if normal estimation is inaccurate
  //n3d_.setSearchSurface (cloud_);
  n3d_.setInputCloud (cloudPtr);
  n3d_.compute (cloud_normals);
  cloud_normalsPtr.reset(new pcl::PointCloud<pcl::Normal>(cloud_normals));
  ROS_INFO ("[TableObjectDetector] %d normals estimated.", (int)cloud_normals.points.size ());

  // ---[ Perform segmentation
  pcl::PointIndices table_inliers; pcl::PointIndices::Ptr table_inliersPtr;
  pcl::ModelCoefficients table_coefficients; pcl::ModelCoefficients::Ptr table_coefficientsPtr;
  seg_.setInputCloud (cloudPtr);
  seg_.setInputNormals (cloud_normalsPtr);
  seg_.segment (table_inliers, table_coefficients);
  table_inliersPtr = boost::make_shared<pcl::PointIndices>(table_inliers);
  table_coefficientsPtr = boost::make_shared<pcl::ModelCoefficients>(table_coefficients);

  if (table_coefficients.values.size () > 3)
    ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].", (int)table_inliers.indices.size (),
        table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);

  if (table_inliers.indices.size () == 0)
    return clusters;

  // ---[ Extract the table
  PointCloud table_projected; PointCloud::Ptr table_projectedPtr;
  proj_.setInputCloud (cloudPtr);
  proj_.setIndices (table_inliersPtr);
  proj_.setModelCoefficients (table_coefficientsPtr);
  proj_.filter (table_projected);
  table_projectedPtr.reset (new PointCloud(table_projected));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of projected inliers: %d.", (int)table_projected.points.size ());

  sensor_msgs::PointCloud table_points;
  tf::Transform table_plane_trans = getPlaneTransform (*table_coefficientsPtr, 1.0);
  //takes the points projected on the table and transforms them into the PointCloud message
  //while also transforming them into the table's coordinate system
  getPlanePoints<Point> (table_projected, table_plane_trans, table_points);
  ROS_INFO("Table computed");

  table_ = computeTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans, table_points);
  publishTable(table_);
  // ---[ Estimate the convex hull
  PointCloud table_hull; PointCloud::Ptr table_hullPtr;
  hull_.setInputCloud (table_projectedPtr);
  hull_.reconstruct (table_hull);
  table_hullPtr.reset (new PointCloud(table_hull));

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices; pcl::PointIndices::Ptr cloud_object_indicesPtr;
  prism_.setInputCloud (cloudPtr);
  prism_.setInputPlanarHull (table_hullPtr);
  prism_.segment (cloud_object_indices);
  cloud_object_indicesPtr = boost::make_shared<pcl::PointIndices>(cloud_object_indices);
  ROS_INFO ("[TableObjectDetector::input_callback] Number of object point indices: %d.", (int)cloud_object_indices.indices.size ());

  PointCloud cloud_objects; PointCloud::Ptr cloud_objectsPtr;
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloudPtr);
  extract_object_indices.setIndices (cloud_object_indicesPtr);
  extract_object_indices.filter (cloud_objects);
  cloudPtr.reset(new PointCloud(cloud_objects));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of object point candidates: %d.", (int)cloud_objects.points.size ());

  if (cloud_objects.points.size () == 0)
    return clusters;

  // ---[ Downsample the points
  PointCloud cloud_objects_downsampled;PointCloud::Ptr cloud_objects_downsampledPtr;
  grid_objects_.setInputCloud (cloudPtr);
  grid_objects_.filter (cloud_objects_downsampled);
  cloudPtr.reset (new PointCloud(cloud_objects_downsampled));
  ROS_INFO ("[TableObjectDetector::input_callback] Number of object point candidates left after downsampling: %d.", (int)cloud_objects_downsampled.points.size ());

  // ---[ Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clustersIndices;
  cluster_.setInputCloud (cloudPtr);
  cluster_.extract (clustersIndices);
  ROS_INFO ("[TableObjectDetector::input_callback] Number of clusters found matching the given constraints: %d.", (int)clustersIndices.size ());

  BOOST_FOREACH(pcl::PointIndices indices, clustersIndices) {
    PointCloud clusterCloud;
    pcl::copyPointCloud(cloud_objects_downsampled, indices, clusterCloud);
    clusters.push_back(clusterCloud);
  }

  return clusters;

}

template <class PointCloudType>
Table TableClusterDetector::computeTable(std_msgs::Header cloud_header,
                                  const tf::Transform &table_plane_trans, 
                                  const PointCloudType &table_points)
{
  Table table;
 
  //get the extents of the table
  if (!table_points.points.empty()) 
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }  
  for (size_t i=1; i<table_points.points.size(); ++i) 
  {
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header = cloud_header;

  return table;
}



/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform TableClusterDetector::getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction=1.0)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  //make sure z points "up"
  ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
    z = -1.0 * z;
    ROS_INFO("flipped z");
  }
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x;  // x
  rotation[1] = y;  // y
  rotation[2] = z;  // z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}

template <typename PointT> 
bool TableClusterDetector::getPlanePoints (const pcl::PointCloud<PointT> &table, 
         const tf::Transform& table_plane_trans,
         sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header = table.header;
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, 
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s", 
        table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  try
  {
    listener.transformPointCloud("table_frame", table_points, table_points);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
        table_points.header.frame_id.c_str(), ex.what());
    return false;
  }
  table_points.header.stamp = table.header.stamp;
  table_points.header.frame_id = "table_frame";
  return true;
}

void TableClusterDetector::publishTable(const Table & table) {
  visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker(table.x_min, table.x_max,
      table.y_min, table.y_max);
  tableMarker.header = table.pose.header;
  tableMarker.pose = table.pose.pose;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub.publish(tableMarker);
}
