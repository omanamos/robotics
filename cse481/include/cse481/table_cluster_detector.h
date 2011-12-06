#ifndef TABLE_CLUSTER_DETECTOR_H
#define TABLE_CLUSTER_DETECTOR_H

#include <vector>
#include <ros/ros.h>
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/features/normal_3d.h"
#include "cse481/Table.h"
#include "cse481/typedefs.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

using namespace cse481;

class TableClusterDetector
{
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;

  public:
  // PCL objects
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::PassThrough<Point> pass_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> cluster_;

  ros::Publisher marker_pub;
  tf::TransformBroadcaster tf_pub_;
  int current_marker_id_;
  double downsample_leaf_, downsample_leaf_objects_;
  int k_;
  double min_z_bounds_, max_z_bounds_;
  double sac_distance_threshold_;
  double normal_distance_weight_;

  // Min/Max height from the table plane object points will be considered from/to
  double object_min_height_, object_max_height_;

  // Object cluster tolerance and minimum cluster size
  double object_cluster_tolerance_, object_cluster_min_size_;

  Table table_;

  TableClusterDetector ();

  std::vector<PointCloud> findTableClusters(const sensor_msgs::PointCloud2 &scene);

  template <class PointCloudType>
    Table computeTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans, 
        const PointCloudType &table_points);


  tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction);

  template <typename PointT> 
    bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
        const tf::Transform& table_plane_trans,
        sensor_msgs::PointCloud &table_points);


  void publishTable(const Table & table);
};

#endif
