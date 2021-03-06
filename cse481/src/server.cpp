#include "Rpc.h"
#include <protocol/TBinaryProtocol.h>
#include <server/TSimpleServer.h>
#include <transport/TServerSocket.h>
#include <transport/TBufferTransports.h>

#include <string>
#include <vector>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/file_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include "cse481/typedefs.h"
#include "cse481/object_detector.h"
#include "cse481/Table.h"
#include "cse481/marker_generator.h"
#include "cse481/utils.h"

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace communication;

void printMatches(const std::vector<ObjectMatch> &matches) {
  BOOST_FOREACH(const ObjectMatch match, matches) {
    std::cout << "Object Name: " << match.getTemplate().getName() << std::endl;
    std::cout << "Average Color: ";
    cse481::operator<<(std::cout, getAverageColor(match.getTemplate().getModel())) << std::endl;
    std::cout << "Fitness: " << match.getFitness() << std::endl;
    AffineTransform final_transformation = match.getTransformation();
    // Print the rotation matrix and translation vector
    printTransform(final_transformation); 
  }
}

sensor_msgs::PointCloud2 loadSceneFromFile() {
  // load the scene cloud
  cse481::PointCloud scene;
  pcl::io::loadPCDFile("nao_kinect2.pcd", scene);
  pcl::PointXYZRGB col = getAverageColor(scene);
  std::cout << "scene avg color: ";
  cse481::operator<<(std::cout, col) << std::endl;
  // match the point cloud
  sensor_msgs::PointCloud2 scene_msg;
  pcl::toROSMsg(scene, scene_msg);
  scene_msg.header.frame_id = "/camera_rgb_optical_frame";
  return scene_msg;
}

sensor_msgs::PointCloud2 loadSceneFromKinect() {
  ROS_WARN("Waiting for point cloud message on 'cloud' topic");
  return *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud"));
}

class RpcHandler : virtual public RpcIf {
  public:
    RpcHandler() : frame_id("/camera_rgb_optical_frame") {
      // Your initialization goes here
      cloud_pub = n.advertise<sensor_msgs::PointCloud2>("matches", 1000);
      marker_pub = n.advertise<visualization_msgs::Marker>("labels", 10);
      loadTemplates();
      current_marker_id_ = 1000;
    }

    void ping() {
      // Your implementation goes here
      printf("ping\n");
    }

    visualization_msgs::Marker getTextMarker(const std::string &text, 
        const std::string &frame_id, const AffineTransform &pose, 
        const std_msgs::ColorRGBA &color) {
      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.header.stamp = ros::Time::now();
      m.ns = "text";
      m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::Marker::ADD;

      Eigen::Vector3f translation = pose.block<3,1>(0, 3);
      m.pose.position.x = translation(0); 
      m.pose.position.y = translation(1); 
      m.pose.position.z = translation(2);
      m.pose.orientation.w = 1.0;

      m.color = color;
      m.scale.z = 0.1;
      m.text = text;
      m.id = current_marker_id_++;
      m.lifetime = ros::Duration();
      return m;
    }

    void locateNao(communication::Point&  _return) {
      sensor_msgs::PointCloud2 scene_msg = loadSceneFromFile();
      frame_id = scene_msg.header.frame_id;
      std::vector<ObjectMatch> recognized_objects, unrecognized_objects;
      det.detectObjectsInScene(scene_msg, &recognized_objects, &unrecognized_objects);

      _return.x = 0.0;
      _return.y = 0.0;
      _return.z = 0.0;
    }


    void getObjects(std::vector<communication::PointCloud> & _return) {
      // Get a point cloud from the kinect
      //sensor_msgs::PointCloud2 scene_msg = loadSceneFromKinect();
      tf::TransformListener l;
      sensor_msgs::PointCloud2 scene_msg = loadSceneFromFile();
      frame_id = scene_msg.header.frame_id;
      std::vector<ObjectMatch> recognized_objects, unrecognized_objects;
      det.detectObjectsInScene(scene_msg, &recognized_objects, &unrecognized_objects);
      std::cout << "Recognized objects: " << std::endl;
      printMatches(recognized_objects);
      std_msgs::ColorRGBA green;
      green.g = 1.0f;
      green.a = 1.0f;
      publishMatches(recognized_objects, green);
      BOOST_FOREACH(ObjectMatch & m, recognized_objects) {
        // Add them to the return vector
        communication::PointCloud c;
        c.identifier = m.getTemplate().getName();
        cse481::PointCloud cloud = m.getTemplate().getModel();
        Eigen::Vector3f centroid(m.getTransformation().block<3,1>(0,3));

        communication::Point avg;
        
        avg.x = centroid(0);
        avg.y = centroid(1);
        avg.z = centroid(2);
        /*
        avg.x = 1.0;
        avg.x = 0.5;
        avg.z = 0.0;
        */
        c.average = avg;
        _return.push_back(c);
      }

      std::cout << "Unrecognized objects: " << std::endl;
      printMatches(unrecognized_objects);
      std_msgs::ColorRGBA red;
      red.r = 1.0f;
      red.a = 1.0f;
      publishMatches(unrecognized_objects, red);

      // Create new templates from the unrecognized objects
      BOOST_FOREACH(ObjectMatch &m, unrecognized_objects) {
        cse481::PointCloud c = m.getTemplate().getModel();
        cse481::PointCloud c_tx,c_demeaned;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(c, centroid);
        pcl::demeanPointCloud(c, centroid, c_demeaned);
        pcl::io::savePCDFile("objects/"+m.getTemplate().getName()+".pcd", c_demeaned);
      }

    }

  protected:
    ros::NodeHandle n;
    ros::Publisher cloud_pub;
    ros::Publisher marker_pub;
    ObjectDetector det;
    std::string frame_id;
    int current_marker_id_;
  private:
    void loadTemplates() {
      std::vector<std::string> files;
      pcl::getAllPcdFilesInDirectory("objects", files);
      BOOST_FOREACH(std::string file, files) {
        std::string name(pcl::getFilenameWithoutExtension(pcl::getFilenameWithoutPath(file)));
        cse481::PointCloud t1, t1_centered;
        pcl::io::loadPCDFile("objects/"+file, t1);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(t1, centroid);
        std::cout << "Name: " << name << std::endl;
        std::cout << "Mean: " << centroid(0) << " " << centroid(1) << " " << centroid(2) << std::endl;
        pcl::PointXYZRGB col = getAverageColor(t1);
        std::cout << "template avg color: ";
        cse481::operator<<(std::cout, col) << std::endl;
        pcl::demeanPointCloud(t1, centroid, t1_centered);
        ObjectTemplate t(name, t1_centered);
        det.addTemplate(t);
      }

      std::cout << "Added " << files.size() << " templates." << std::endl;
    }


    void publishMatches(const std::vector<ObjectMatch> &matches, 
        const std_msgs::ColorRGBA &color) {
      BOOST_FOREACH(const ObjectMatch &m, matches) {
        cse481::PointCloud pc;
        pcl::transformPointCloud(m.getTemplate().getModel(), pc, m.getTransformation());
        visualization_msgs::Marker cloud_marker = 
          MarkerGenerator::getCloudMarker(pc, color);
        //cloud_marker.header.frame_id = frame_id;
        cloud_marker.header.frame_id = "/table";
        cloud_marker.ns = "matches";
        cloud_marker.header.stamp = ros::Time::now();
        cloud_marker.id = current_marker_id_++;
        cloud_marker.lifetime = ros::Duration();
        marker_pub.publish(cloud_marker);
        visualization_msgs::Marker mk = getTextMarker(m.getTemplate().getName(), 
            "/table", m.getTransformation(), color);
        marker_pub.publish(mk);
      }
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinectLoader", ros::init_options::NoSigintHandler);
  int port = 9090;
  shared_ptr<RpcHandler> handler(new RpcHandler());
  shared_ptr<TProcessor> processor(new RpcProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}

