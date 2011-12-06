#include "Rpc.h"
#include <protocol/TBinaryProtocol.h>
#include <server/TSimpleServer.h>
#include <transport/TServerSocket.h>
#include <transport/TBufferTransports.h>


#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include "cse481/marker_generator.h"
#include "solution/object_recognition.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

communication::Color getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  communication::Color mean;
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

class RpcHandler : virtual public communication::RpcIf {
 public:
  RpcHandler(int argc, char** argv) {
    // Load the recognition params
    loadParams(argc, argv);
    rec.reset(new ObjectRecognition(params));
    // Load the NAO model in
    rec->loadNaoModel("models/nao2/nao");
    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker>("server_markers", 10);
    marker_ids_ = 0;
    useKinect = true;
  }

  void ping() {
    // Your implementation goes here
    printf("ping\n");
  }

  void printObject(const DetectedObject &obj) {
    ROS_INFO_STREAM("Found object: " << obj.identifier);
    ROS_INFO_STREAM("Location: " << std::endl << obj.transform.translation());
    ROS_INFO("Fitness: %f", obj.fitness);
  }

  PointCloudPtr getKinectCloud() {
    sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud");
    PointCloudPtr pclCloud(new PointCloud);
    pcl::fromROSMsg(*cloud, *pclCloud);
    return pclCloud;
  }

  void getObjects(std::vector<communication::PointCloud> & objects) {
    printf("=========getObjects=========\n");
    // Grab a frame
    
    PointCloudPtr scene;
    if (useKinect) {
      scene = getKinectCloud();
    } else  {
      scene = loadPointCloud<PointT>("nao_kinect1", ".pcd");
      scene->header.frame_id = "/camera_rgb_optical_frame";
    }

    std::vector<DetectedObject> found_objects;
    rec->recognizeAndAlignPoints(scene, &found_objects);
    std::vector<DetectedObject> unknownModels;

    
   
    std_msgs::ColorRGBA green;
    green.a = 1.0;
    green.g = 1.0;

    std_msgs::ColorRGBA red;
    red.r = 1.0;
    red.a = 1.0;


    
    
    BOOST_FOREACH(DetectedObject &det, found_objects) {
      printObject(det);
      bool isNew = (objects_map.find(det.identifier) == objects_map.end());
      communication::Point avg;
      avg.x = det.transform.translation()(0);
      avg.y = det.transform.translation()(1);
      avg.z = det.transform.translation()(2);
      communication::Color avgColor = getAverageColor(det.points);
      if (isNew) {
        // Create a new PointCloud
        communication::PointCloud res;
        res.average = avg;
        res.color = avgColor;
        res.identifier = det.identifier;
        // Add it to the map and to the recognizer
        objects_map[res.identifier] = res;
        objects.push_back(res);
        unknownModels.push_back(det);

        PointCloud transformed_object;
        pcl::transformPointCloud(det.points, transformed_object, rec->getLatestPlaneTransform() * det.transform);
        visualization_msgs::Marker obj_marker = MarkerGenerator::getCloudMarker(transformed_object, red);
        obj_marker.header.frame_id = "/camera_rgb_optical_frame";
        obj_marker.header.stamp = ros::Time::now();
        obj_marker.id = marker_ids_++;
        obj_marker.ns = "unrecognized_objects";
        marker_pub.publish(obj_marker);
      } else {
        // Look up the point cloud
        communication::PointCloud& res = objects_map[det.identifier];
        // modify it
        res.average = avg;
        res.color = avgColor;
        
        objects.push_back(res);

        PointCloud transformed_object;
        pcl::transformPointCloud(det.points, transformed_object, rec->getLatestPlaneTransform() * det.transform);
        visualization_msgs::Marker obj_marker = MarkerGenerator::getCloudMarker(transformed_object, green);
        obj_marker.header.frame_id = "/camera_rgb_optical_frame";
        obj_marker.header.stamp = ros::Time::now();
        obj_marker.id = marker_ids_++;
        obj_marker.ns = "recognized_objects";
        marker_pub.publish(obj_marker);
      }
    }
    ROS_INFO("Adding %zd models to the recognizer", unknownModels.size());
    rec->addModels(unknownModels);
  }


  void locateNao(communication::Point& loc) {
    printf("=========locateNao=========\n");
  
    PointCloudPtr scene;
    if (useKinect) {
      scene = getKinectCloud();
    } else  {
      scene = loadPointCloud<PointT>("nao_kinect1", ".pcd");
      scene->header.frame_id = "/camera_rgb_optical_frame";
    }
    Eigen::Affine3f nao_tr;
    PointCloudPtr naoModel = rec->findNao(scene, &nao_tr);

    PointCloud tr_nao;
    pcl::transformPointCloud(*naoModel, tr_nao, rec->getLatestPlaneTransform() * nao_tr);
    std_msgs::ColorRGBA green;
    green.a = 1.0;
    green.g = 1.0;
    visualization_msgs::Marker nao_marker = MarkerGenerator::getCloudMarker(tr_nao, green);
    nao_marker.header.frame_id = "/camera_rgb_optical_frame";
    nao_marker.header.stamp = ros::Time::now();
    nao_marker.id = marker_ids_++;
    nao_marker.ns = "nao";
    marker_pub.publish(nao_marker);

    Eigen::Vector3f position = nao_tr.matrix().block<3,1>(0,3);
    Eigen::Vector3f xvector = nao_tr.matrix().block<3,1>(0,0);
    loc.x = position(0);
    loc.y = position(1);
    // Z is angle between nao-forward and floor +x
    loc.z = atan2(xvector(0), xvector(1));
    printf("NAO Position: %f %f, Theta: %f\n", loc.x, loc.y, loc.z);
  }

  bool update(const std::string& oldIdentifier, const std::string& newIdentifier) {
    printf("=========update=========\n");
    std::map<std::string, communication::PointCloud>::iterator it;
    it = objects_map.find(oldIdentifier);
    rec->updateModelIdentifier(oldIdentifier, newIdentifier);
    if (it != objects_map.end()) {
      communication::PointCloud pc = it->second;
      pc.identifier = newIdentifier;
      objects_map.erase(it);
      objects_map[newIdentifier] = pc;
      return true;
    }
    return false;
  }

 private:

  void loadParams(int argc, char** argv) {
    std::string filter_parameters_file;
    pcl::console::parse_argument (argc, argv, "--filter", filter_parameters_file);    
    std::ifstream input_stream;
    input_stream.open (filter_parameters_file.c_str ());
    if (input_stream.is_open())
    {
      input_stream >> params.min_depth;
      input_stream >> params.max_depth;
      input_stream >> params.downsample_leaf_size;
      input_stream >> params.outlier_rejection_radius;
      input_stream >> params.outlier_rejection_min_neighbors;
      input_stream.close ();
    }
    else
    {
      pcl::console::print_info ("Failed to open the filter parameters file (%s)\n", filter_parameters_file.c_str ());
    }  

    // Parse segmentation parameters
    std::string segmentation_parameters_file;
    pcl::console::parse_argument (argc, argv, "--segment", segmentation_parameters_file);    
    input_stream.open (segmentation_parameters_file.c_str ());
    if (input_stream.is_open())
    {
      input_stream >> params.plane_inlier_distance_threshold;
      input_stream >> params.max_ransac_iterations;
      input_stream >> params.cluster_tolerance;
      input_stream >> params.min_cluster_size;
      input_stream >> params.max_cluster_size;
      input_stream >> params.plane_up_direction;
      input_stream.close ();
    }
    else
    {
      pcl::console::print_info ("Failed to open the segmentation parameters file (%s)\n", 
          segmentation_parameters_file.c_str ());
    }

    // Parse feature estimation parameters
    std::string feature_estimation_parameters_file;
    pcl::console::parse_argument (argc, argv, "--feature", feature_estimation_parameters_file);    
    input_stream.open (feature_estimation_parameters_file.c_str ());
    if (input_stream.is_open())
    {
      input_stream >> params.surface_normal_radius;
      input_stream >> params.keypoints_min_scale;
      input_stream >> params.keypoints_nr_octaves;
      input_stream >> params.keypoints_nr_scales_per_octave;
      input_stream >> params.keypoints_min_contrast;
      input_stream >> params.local_descriptor_radius;
      input_stream.close ();
    }
    else
    {
      pcl::console::print_info ("Failed to open the feature estimation parameters file (%s)\n", 
          feature_estimation_parameters_file.c_str ());
    }

    // Parse the registration parameters
    std::string registration_parameters_file;
    pcl::console::parse_argument (argc, argv, "--registration", registration_parameters_file);    
    input_stream.open (registration_parameters_file.c_str ());
    if (input_stream.is_open())
    {
      input_stream >> params.initial_alignment_min_sample_distance;
      input_stream >> params.initial_alignment_max_correspondence_distance;
      input_stream >> params.initial_alignment_nr_iterations;
      input_stream >> params.icp_max_correspondence_distance;
      input_stream >> params.icp_outlier_rejection_threshold;
      input_stream >> params.icp_transformation_epsilon;
      input_stream >> params.icp_max_iterations;
      input_stream.close ();
    }
    else
    {
      pcl::console::print_info ("Failed to open the registration parameters file (%s)\n", 
          registration_parameters_file.c_str ());
    }

  }

  std::map<std::string, communication::PointCloud> objects_map;
  ObjectRecognitionParameters params;
  shared_ptr<ObjectRecognition> rec;
  ros::Publisher marker_pub;
  int marker_ids_;
  bool useKinect;
};

using namespace communication;

int main(int argc, char **argv) {
  int port = 9090;
  ros::init(argc, argv, "server",ros::init_options::NoSigintHandler);
  shared_ptr<RpcHandler> handler(new RpcHandler(argc, argv));
  shared_ptr<TProcessor> processor(new RpcProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}
