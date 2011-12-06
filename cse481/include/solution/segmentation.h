#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "typedefs.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


/* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input 
 *     The input point cloud
 *   max_iterations 
 *     The maximum number of RANSAC iterations to run
 *   distance_threshold 
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane, 
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
pcl::ModelCoefficients::Ptr
fitPlane (const PointCloudPtr & input, float distance_threshold, float max_iterations)
{
  // Intialize the SACSegmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);

  seg.setInputCloud (input);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);  

  return (coefficients);
}

void computePlaneTransform(const pcl::ModelCoefficients & coeffs, Eigen::Affine3f & tr, double up_direction) {
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  Eigen::Vector3f position(-a*d, -b*d, -c*d);
  Eigen::Vector3f z(a, b, c);
  if (z.dot(Eigen::Vector3f(0,0, up_direction)) < 0 ) {
    z = -1.0 * z;
    ROS_INFO("flipped z");
  }

  Eigen::Vector3f x(0,1,0);
  //if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f y = z.cross(x).normalized();
  x = y.cross(z).normalized();
  tr.matrix().block<3,1>(0,0) = x;
  tr.matrix().block<3,1>(0,1) = y;
  tr.matrix().block<3,1>(0,2) = z;
  tr.matrix().block<3,1>(0,3) = position;
  //tr.rotate(Eigen::AngleAxisf(3.14, z));
}

/* Use SACSegmentation and an ExtractIndices filter to find the dominant plane and subtract it
 * Inputs:
 *   input 
 *     The input point cloud
 *   max_iterations 
 *     The maximum number of RANSAC iterations to run
 *   distance_threshold 
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to a new point cloud which contains only the non-plane points
 */
PointCloudPtr
findAndSubtractPlane (const PointCloudPtr & input, float distance_threshold, 
    float max_iterations, Eigen::Affine3f *plane_transform, double up_direction)
{
  // Find the dominant plane
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (false);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (input);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);

  computePlaneTransform(*coefficients, *plane_transform, up_direction);

  // Extract the inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (true);
  PointCloudPtr output (new PointCloud);
  extract.filter (*output);

  return (output);
}



/* Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
 * Inputs:
 *   input
 *     The input point cloud
 *   cluster_tolerance
 *     The maximum distance between neighboring points in a cluster
 *   min/max_cluster_size
 *     The minimum and maximum allowable cluster sizes
 * Return (by reference): a vector of PointIndices containing the points indices in each cluster
 */
void
clusterObjects (const PointCloudPtr & input, 
                float cluster_tolerance, int min_cluster_size, int max_cluster_size,
                std::vector<pcl::PointIndices> & cluster_indices_out)
{  
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);

  ec.setInputCloud (input);
  ec.extract (cluster_indices_out);
}

#endif
