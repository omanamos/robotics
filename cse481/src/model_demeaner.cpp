#include <string>
#include <Eigen/Core>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include "typedefs.h"
#include "load_clouds.h"
#include "solution/filters.h"
#include "solution/segmentation.h"
#include "solution/feature_estimation.h"
#include "solution/registration.h"


int main(int argc, char** argv)
{
  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%zu points)\n", argv[1], cloud->size ());


  Eigen::Affine3f newBasis;
  pcl::PCA<PointT> pcaAligner(*cloud);
  newBasis.linear() = pcaAligner.getEigenVectors();
  newBasis.translation() = pcaAligner.getMean().head(3);

  PointCloudPtr cloud_demeaned (new PointCloud);
  pcl::transformPointCloud(*cloud, *cloud_demeaned, newBasis.inverse());
  //pcl::demeanPointCloud(*cloud, centroid, *cloud_demeaned);


  // Save output
  std::string output_filename;
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", output_filename) > 0;
  if (save_cloud)
  {
    pcl::io::savePCDFile (output_filename, *cloud_demeaned);
    pcl::console::print_info ("Saved result as %s\n", output_filename.c_str ());
  }

}
