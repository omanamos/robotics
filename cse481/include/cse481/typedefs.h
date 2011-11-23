#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace cse481 {
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef Eigen::Matrix4f AffineTransform;
}
#endif
