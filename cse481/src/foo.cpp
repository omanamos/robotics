#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

int main()
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::io::loadPCDFile("objects/cluster_0.pcd", cloud);
  BOOST_FOREACH(pcl::PointXYZRGB &pt, cloud) {
    pt.x *= 2;
  }
  pcl::io::savePCDFile("objects/cluster_0_xsquared.pcd", cloud);
}
