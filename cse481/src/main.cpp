#include <string>
#include <vector>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <ros/ros.h>
#include "cse481/typedefs.h"
#include "cse481/object_detector.h"

void printMatches(const std::vector<ObjectMatch> &matches) {
  BOOST_FOREACH(const ObjectMatch match, matches) {
    std::cout << "Object Name: " << match.getTemplate().getName() << std::endl;
    std::cout << "Fitness: " << match.getFitness() << std::endl;
    AffineTransform final_transformation = match.getTransformation();
    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = final_transformation.block<3,1>(0, 3);

    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
}

sensor_msgs::PointCloud2 loadSceneFromFile() {
  // load the scene cloud
  PointCloud scene;
  pcl::io::loadPCDFile("scene.pcd", scene);
  // match the point cloud
  sensor_msgs::PointCloud2 scene_msg;
  pcl::toROSMsg(scene, scene_msg);
  return scene_msg;
}

sensor_msgs::PointCloud2 loadSceneFromKinect(int argc, char** argv) {
  ros::init(argc, argv, "kinectLoader");
  ROS_WARN("Waiting for point cloud message on 'cloud' topic");
  return *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud"));
}

int main(int argc, char** argv)
{
  ObjectDetector det;
  // add the templates and make sure they are centered
  std::vector<std::string> files;
  pcl::getAllPcdFilesInDirectory("objects", files);
  BOOST_FOREACH(std::string file, files) {
    std::string name(pcl::getFilenameWithoutExtension(pcl::getFilenameWithoutPath(file)));
    PointCloud t1, t1_centered;
    pcl::io::loadPCDFile("objects/"+file, t1);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(t1, centroid);
    std::cout << "Name: " << name << std::endl;
    std::cout << "Mean: " << centroid(0) << " " << centroid(1) << " " << centroid(2) << std::endl;
    pcl::demeanPointCloud(t1, centroid, t1_centered);
    ObjectTemplate t(name, t1_centered);
    det.addTemplate(t);
  }

  std::cout << "Added " << files.size() << " templates." << std::endl;

  sensor_msgs::PointCloud2 scene_msg = loadSceneFromFile();
  //sensor_msgs::PointCloud2 scene_msg = loadSceneFromKinect(argc, argv);
  
  std::vector<ObjectMatch> recognized_objects, unrecognized_objects;
  det.detectObjectsInScene(scene_msg, &recognized_objects, &unrecognized_objects);
  std::cout << "Recognized objects: " << std::endl;
  printMatches(recognized_objects); 
  std::cout << "Unrecognized objects: " << std::endl;
  printMatches(unrecognized_objects);
  BOOST_FOREACH(ObjectMatch &m, unrecognized_objects) {
    PointCloud c = m.getTemplate().getModel();
    PointCloud c_demeaned;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(c, centroid);
    pcl::demeanPointCloud(c, centroid, c_demeaned);
    pcl::io::savePCDFile("objects/"+m.getTemplate().getName()+".pcd", c_demeaned);
  }
}
