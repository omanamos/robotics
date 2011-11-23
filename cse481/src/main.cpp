#include <string>
#include <vector>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include "cse481/typedefs.h"
#include "cse481/object_detector.h"
  



sensor_msgs::PointCloud2 loadSceneFromFile() {
  // load the scene cloud
  PointCloud scene;
  pcl::io::loadPCDFile("test_data/scene.pcd", scene);
  // match the point cloud
  sensor_msgs::PointCloud2 scene_msg;
  pcl::toROSMsg(scene, scene_msg);
  return scene_msg;
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "kinectLoader");
  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("matches", 1000);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("labels", 10);
  ObjectDetector det;
  // add the templates and make sure they are centered
  

  //sensor_msgs::PointCloud2 scene_msg = loadSceneFromFile();
  }
