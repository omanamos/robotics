#ifndef UTILS_H
#define UTILS_H

#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
// TODO: maybe use a different transform method?
#include <pcl/registration/transforms.h>
#include "cse481/typedefs.h"
#include <iostream>

namespace cse481 {

  inline pcl::PointXYZRGB getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
    pcl::PointXYZRGB mean;
    float r=0;
    float g=0;
    float b=0;
    BOOST_FOREACH(const pcl::PointXYZRGB &pt, cloud) {
      r += pt.r;
      g += pt.g;
      b += pt.b;
    }

    mean.r = (int)(r / cloud.points.size());
    mean.g = (int)(g / cloud.points.size());
    mean.b = (int)(b / cloud.points.size());
    return mean;
  }

  inline void printRotation(const Eigen::Matrix3f &rotation) {
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  }
  inline void printTransform(const AffineTransform &t) {
    Eigen::Matrix3f rotation = t.block<3,3>(0, 0);
    Eigen::Vector3f translation = t.block<3,1>(0, 3);
    
    printf ("\n");
    printRotation(rotation);
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  inline void printMatrix(const Eigen::Affine3f &t) {
    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", t(0,0), t(0,1), t(0,2), t(0,3));
    printf ("M = | %6.3f %6.3f %6.3f %6.3f | \n", t(1,0), t(1,1), t(1,2), t(1,3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", t(2,0), t(2,1), t(2,2), t(2,3));
    printf ("    | %6.3f %6.3f %6.3f %6.3f | \n", t(3,0), t(3,1), t(3,2), t(3,3));
    printf ("\n");
  }

  template<typename tfType>
  inline void tfToEigen(const tfType &tx, AffineTransform &out) {
    tf::Quaternion rot = tx.getRotation();
    Eigen::Quaternionf rot_eig(rot.w(), rot.x(), rot.y(), rot.z());
    tf::Vector3 origin = tx.getOrigin();
    Eigen::Vector3f origin_eig(origin.x(), origin.y(), origin.x());
    Eigen::Vector3f scale(1,1,1);
    Eigen::Affine3f tr;
    tr.fromPositionOrientationScale(origin_eig, rot_eig, scale);
    out = tr.matrix();
  }

  inline void transformPointCloud(const PointCloud &cloud_in, PointCloud &cloud_out, tf::StampedTransform &tx) {
    AffineTransform t;
    tfToEigen(tx, t);
    pcl::transformPointCloud(cloud_in, cloud_out, t);
  }

  inline std::ostream& operator<<(std::ostream &stream, const pcl::PointXYZRGB &pt) {
    stream << "(" << (int)pt.r << ", " << (int)pt.g << ", " << (int)pt.b << ")";
    return stream;
  }

}
#endif
