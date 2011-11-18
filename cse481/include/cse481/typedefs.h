#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Matrix4f AffineTransform;

#endif
