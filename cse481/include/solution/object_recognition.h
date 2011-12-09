#ifndef OBJECT_RECOGNITION_H_
#define OBJECT_RECOGNITION_H_

#include "typedefs.h"
#include "load_clouds.h"
#include "solution/filters.h"
#include "solution/segmentation.h"
#include "solution/feature_estimation.h"
#include "solution/registration.h"

#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

struct ObjectRecognitionParameters
{
  // Filter parameters
  float min_depth;
  float max_depth;
  float downsample_leaf_size;
  float outlier_rejection_radius;
  int outlier_rejection_min_neighbors;

  // Segmentation parameters
  float plane_inlier_distance_threshold;
  int max_ransac_iterations;
  float cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;
  double plane_up_direction;
  
  // Feature estimation parameters
  float surface_normal_radius;
  float keypoints_min_scale;
  float keypoints_nr_octaves;
  float keypoints_nr_scales_per_octave;
  float keypoints_min_contrast;
  float local_descriptor_radius;

  // Registration parameters
  float initial_alignment_min_sample_distance;
  float initial_alignment_max_correspondence_distance;
  int initial_alignment_nr_iterations;
  float icp_max_correspondence_distance;
  float icp_outlier_rejection_threshold;
  float icp_transformation_epsilon;
  int icp_max_iterations;
};

struct ObjectModel
{
  PointCloudPtr points;
  PointCloudPtr keypoints;
  LocalDescriptorsPtr local_descriptors;
  GlobalDescriptorsPtr global_descriptor;
  std::string identifier;
};

struct nn_match
{
  int objectId;
  float score;
};

struct DetectedObject
{
  std::string identifier;
  PointCloud points;
  Eigen::Affine3f transform;
  float fitness;
};


void getAverageColor(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Vector3f& color) {
  BOOST_FOREACH(const pcl::PointXYZRGB &pt, cloud) {
    color(0) += pt.r;
    color(1) += pt.g;
    color(2) += pt.b;
  }
  color /= cloud.size();
}

float colorDist(const Eigen::Vector3f& c1, const Eigen::Vector3f& c2) {
  return (c1 - c2).norm();
}

class ObjectRecognition
{
  public:
    ObjectRecognition (const ObjectRecognitionParameters & params) : params_ (params), unknown_threshold(1000)
    {
      ros::NodeHandle nh;
      kdtree_ = pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr (new pcl::KdTreeFLANN<GlobalDescriptorT>);
      descriptors_ = GlobalDescriptorsPtr (new GlobalDescriptors);
      table_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("table_pose", 10);
      identifiers_ = 0;
    }

    /*
    void addModels(std::string model_folder) {
      std::vector<std::string> model_files;

      std::vector<DetectedObject> loaded_objects;
      BOOST_FOREACH(const std::string& model_file, model_files) {
        // If filename ends pcd extension, load the points and process them
        PointCloudPtr raw_input (new PointCloud);
        pcl::io::loadPCDFile (filenames[i], *raw_input);

        DetectedObject object;
        object.identifier = model_file.substr(0, model_file.size() - 4);
        object.points = *raw_input;
        loaded_objects.push_back(object);
      }
      addModels(loaded_objects);
    }
    */

    /*
    void 
    populateDatabase (const std::vector<std::string> & filenames)
    {
      size_t n = filenames.size ();
      models_.resize (n);
      descriptors_ = GlobalDescriptorsPtr (new GlobalDescriptors);
      for (size_t i = 0; i < n; ++i)
      {
        const std::string & filename = filenames[i];
        if (filename.compare (filename.size ()-4, 4, ".pcd") == 0)
        {
          // If filename ends pcd extension, load the points and process them
          PointCloudPtr raw_input (new PointCloud);
          pcl::io::loadPCDFile (filenames[i], *raw_input);
          
          constructObjectModel (raw_input, models_[i]);
        }
        else
        {
          // If the filename has no extension, load the pre-computed models
          models_[i].points = loadPointCloud<PointT> (filename, "_points.pcd");
          models_[i].keypoints = loadPointCloud<PointT> (filename, "_keypoints.pcd");
          models_[i].local_descriptors = loadPointCloud<LocalDescriptorT> (filename, "_localdesc.pcd");
          models_[i].global_descriptor = loadPointCloud<GlobalDescriptorT> (filename, "_globaldesc.pcd");       
        }
        *descriptors_ += *(models_[i].global_descriptor);
      }
      kdtree_ = pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr (new pcl::KdTreeFLANN<GlobalDescriptorT>);
      kdtree_->setInputCloud (descriptors_);
    }
    */

    void addModels(std::vector<DetectedObject> &models) {
      BOOST_FOREACH(DetectedObject &obj, models) {
        ObjectModel model;
        model.identifier = obj.identifier;
        PointCloudPtr input_cloud(new PointCloud(obj.points));
        bool filterAndSegment = false;
        constructObjectModel(input_cloud, model, filterAndSegment);
        models_.push_back(model);
        *descriptors_ += *(model.global_descriptor);
      }
      kdtree_->setInputCloud (descriptors_);
    }

    void updateModelIdentifier(const std::string &oldIdentifier, const std::string& newIdentifier) {
      BOOST_FOREACH(ObjectModel &mod, models_) {
        if (mod.identifier == oldIdentifier) {
          mod.identifier = newIdentifier;
        }
      }
    }

    void loadNaoModel(std::string nao_filename) {
      naoModel.points = loadPointCloud<PointT> (nao_filename, "_points.pcd");
      naoModel.keypoints = loadPointCloud<PointT> (nao_filename, "_keypoints.pcd");
      naoModel.local_descriptors = loadPointCloud<LocalDescriptorT> (nao_filename, "_localdesc.pcd");
      naoModel.global_descriptor = loadPointCloud<GlobalDescriptorT> (nao_filename, "_globaldesc.pcd");
      naoModel.identifier = "nao";
    }

    PointCloudPtr matchClustersToNao(std::vector<ObjectModel>& query_clusters, bool align, Eigen::Affine3f *nao_transform) {
      pcl::KdTreeFLANN<GlobalDescriptorT> scene_objects_tree;
      
      GlobalDescriptorsPtr descriptors_list(new GlobalDescriptors);

      size_t numClusters = query_clusters.size();
      ROS_INFO("Found %zd clusters", numClusters);
      if (numClusters > 0) {
        Eigen::Vector3f naoColor(0,0,0);
        getAverageColor(*(naoModel.points), naoColor);
        ROS_INFO_STREAM("Nao color: "<<naoColor);
        int closest_color_idx = -1;
        float color_dist = 100000;
        for (size_t i = 0; i < numClusters; i++) {
          ObjectModel query_object = query_clusters[i];
          (*descriptors_list) += *(query_object.global_descriptor);
          Eigen::Vector3f avgColor(0,0,0);
          getAverageColor(*(query_object.points), avgColor);
          float thisColorDist = colorDist(avgColor, naoColor);
          ROS_INFO("Colordist for %zd (color (%f %f %f)), %f", i, avgColor(0), avgColor(1), avgColor(2), thisColorDist);
          if (thisColorDist < color_dist) {
            color_dist = thisColorDist;
            closest_color_idx = i;
          }
        }

        scene_objects_tree.setInputCloud(descriptors_list);

        // Search for the NAO descriptor in the scene
        std::vector<int> nn_index (1);
        std::vector<float> nn_sqr_distance (1);
        scene_objects_tree.nearestKSearch (naoModel.global_descriptor->points[0], descriptors_list->size(), 
          nn_index, nn_sqr_distance);
        ROS_INFO("Locating NAO: Object distances:");
        for (int i=0; i < (int)descriptors_list->size(); i++) {
          ROS_INFO("Object %d: %f", nn_index[i], nn_sqr_distance[i]);
        }
        int best_match = nn_index[0];
        ROS_INFO("Finding the NAO, closest squared distance: %f", nn_sqr_distance[0]);

        ROS_INFO("From color, closest is %d. From GDesc, closest is %d", closest_color_idx, best_match);
        best_match = closest_color_idx;
        if (align) {
          ROS_INFO("Aligning NAO to object %d", best_match);
          PointCloudPtr output = alignModelPoints (naoModel, query_clusters[best_match], params_, nao_transform);
        }
        query_clusters.erase(query_clusters.begin()+best_match);
      }
      return (naoModel.points);
    }

    bool matchClustersToModel(ObjectModel& model, std::vector<ObjectModel>& query_clusters, int* best_matched_model, bool align, Eigen::Affine3f *transform, float* fitness_val) {
      pcl::KdTreeFLANN<GlobalDescriptorT> scene_objects_tree;
      
      GlobalDescriptorsPtr descriptors_list(new GlobalDescriptors);

      size_t numClusters = query_clusters.size();
      ROS_INFO("Found %zd clusters", numClusters);
      if (numClusters > 0) {
        Eigen::Vector3f modelColor(0,0,0);
        getAverageColor(*(model.points), modelColor);
        ROS_INFO_STREAM("Model color: "<<modelColor);
        int closest_color_idx = -1;
        float color_dist = 100000;
        for (size_t i = 0; i < numClusters; i++) {
          ObjectModel query_object = query_clusters[i];
          (*descriptors_list) += *(query_object.global_descriptor);
          Eigen::Vector3f avgColor(0,0,0);
          getAverageColor(*(query_object.points), avgColor);
          float thisColorDist = colorDist(avgColor, modelColor);
          ROS_INFO("Colordist for %zd (color (%f %f %f)), %f", i, avgColor(0), avgColor(1), avgColor(2), thisColorDist);
          if (thisColorDist < color_dist) {
            color_dist = thisColorDist;
            closest_color_idx = i;
          }
        }

        scene_objects_tree.setInputCloud(descriptors_list);

        // Search for the NAO descriptor in the scene
        std::vector<int> nn_index (1);
        std::vector<float> nn_sqr_distance (1);
        scene_objects_tree.nearestKSearch (model.global_descriptor->points[0], descriptors_list->size(), 
          nn_index, nn_sqr_distance);
        ROS_INFO("Locating NAO: Object distances:");
        for (int i=0; i < (int)descriptors_list->size(); i++) {
          ROS_INFO("Object %d: %f", nn_index[i], nn_sqr_distance[i]);
        }
        int best_match = nn_index[0];
        ROS_INFO("Matching object %s, closest squared distance: %f", model.identifier.c_str(), nn_sqr_distance[0]);

        ROS_INFO("From color, closest is %d. From GDesc, closest is %d", closest_color_idx, best_match);
        ROS_INFO("Closest colordist %f, closest nn %f", color_dist, nn_sqr_distance[0]);
        bool color_match = color_dist < 1000;
        bool nn_match = nn_sqr_distance[0] < 2100;
        if (color_match) {
          ROS_INFO("Object matched based on color");
          best_match = closest_color_idx;
          *fitness_val = color_dist;
        }
        else if (nn_match) {
          ROS_INFO("Object matched based on shape");
          best_match = nn_index[0];
          *fitness_val = nn_sqr_distance[0];
        }
        else {
          ROS_INFO("Object didn't match color or shape");
          return false;
        }
        if (align) {
          ROS_INFO("Aligning object %s to object %d",model.identifier.c_str(), best_match);
          PointCloudPtr output = alignModelPoints (naoModel, query_clusters[best_match], params_, transform);
        }
        *best_matched_model = best_match;
        return true;
      } else {
        ROS_INFO("no clusters");
        return false;
      }
    }


    PointCloudPtr findNao(const PointCloudPtr & query_cloud, Eigen::Affine3f *nao_transform) {
      if (naoModel.identifier != "nao") {
        ROS_ERROR("Nao model is not yet set!");
        return PointCloudPtr(new PointCloud); 
      }

      camera_frame_ = query_cloud->header.frame_id;
      // Need to clear the frame_id so that it matches saved models
      query_cloud->header.frame_id = "";

      std::vector<ObjectModel> query_clusters;
      constructObjectModelsFromScene (query_cloud, query_clusters);
      return matchClustersToNao(query_clusters, true, nao_transform);
    }
    void
    recognizeAndAlignPoints (const PointCloudPtr & query_cloud, std::vector<DetectedObject> *found_objects)
    {
      std::vector<ObjectModel> query_clusters;
      camera_frame_ = query_cloud->header.frame_id;
      // Need to clear the frame_id so that it matches saved models
      query_cloud->header.frame_id = "";
      constructObjectModelsFromScene (query_cloud, query_clusters);
      size_t numClusters = query_clusters.size();
      ROS_INFO("Found %zd clusters", numClusters);
      if (numClusters > 0) {
        matchClustersToNao(query_clusters, false, NULL);
        numClusters = query_clusters.size();
        ROS_INFO("After removing NAO, found %zd clusters", numClusters);

        for (size_t i=0; i < models_.size(); i++) {
          int match_idx;
          DetectedObject det;
          bool foundMatch = matchClustersToModel(models_[i], query_clusters, &match_idx, false, NULL, &(det.fitness));
          if (foundMatch) {
            ROS_INFO("Cluster %zd matched existing object %s", i, models_[i].identifier.c_str());
            det.identifier = models_[i].identifier;
            det.points = *(models_[i].points);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*(query_clusters[match_idx].points), centroid);
            query_clusters.erase(query_clusters.begin()+match_idx);
            det.transform = Eigen::Affine3f::Identity();
            // Grab the first 3 elements of the centroid (last elem is garbage)
            det.transform.translation() = centroid.head<3>();

            found_objects->push_back(det);
          }
        }

        ROS_INFO("After removing clusters which match previous objects, there are %zd clusters left", query_clusters.size());

        for (size_t i = 0; i < query_clusters.size(); i++) {
          DetectedObject det;
          det.fitness = -1.0;
          ObjectModel query_object = query_clusters[i];
          PointCloudPtr pts = query_object.points;
          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*pts, centroid);
          // This is the zero-mean set of points defining the object
          pcl::demeanPointCloud(*pts, centroid, det.points);
          det.identifier = std::string("_unknown")+boost::lexical_cast<std::string>(identifiers_++);
          // This is the position of those points
          det.transform = Eigen::Affine3f::Identity();
          // Grab the first 3 elements of the centroid (last elem is garbage)
          det.transform.translation() = centroid.head<3>();
          found_objects->push_back(det);
        }
      } else {
        ROS_INFO("No clusters found in scene.");
      }
    }

    /* Construct an object model by filtering, segmenting, and estimating feature descriptors */
    void
    constructObjectModel (const PointCloudPtr & points, ObjectModel & output, bool filterAndSegment)
    {
      if (filterAndSegment) {
        std::vector<pcl::PointIndices> cluster_indices;
        PointCloudPtr cloud = applyFiltersAndSegment (points, params_, cluster_indices, false);
        PointCloud largest_cluster;
        pcl::copyPointCloud (*cloud, cluster_indices[0], largest_cluster);
        
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(largest_cluster, centroid);
        PointCloudPtr demeaned_pointcloud(new PointCloud);
        pcl::demeanPointCloud(largest_cluster, centroid, *demeaned_pointcloud);
        
        output.points = demeaned_pointcloud;
      } else {
        output.points.reset(new PointCloud);
        pcl::copyPointCloud(*points, *(output.points));
      }
      SurfaceNormalsPtr normals;
      estimateFeatures (output.points, params_, normals, output.keypoints, 
                        output.local_descriptors, output.global_descriptor);
    }

    /* Construct object models by filtering, segmenting, and estimating feature descriptors 
     * for all detected clusters */
    void
    constructObjectModelsFromScene (const PointCloudPtr & points, std::vector<ObjectModel> & outputs)
    {
      std::vector<pcl::PointIndices> clusters;
      PointCloudPtr cluster_points = applyFiltersAndSegment (points, params_, clusters, true);
      BOOST_FOREACH(const pcl::PointIndices &indices, clusters) {
        ObjectModel output;
        output.points.reset( new PointCloud);
        pcl::copyPointCloud(*cluster_points, indices, *(output.points));

        SurfaceNormalsPtr normals;
        estimateFeatures (output.points, params_, normals, output.keypoints, 
                          output.local_descriptors, output.global_descriptor);
        outputs.push_back(output);
      }
      
    }

    Eigen::Affine3f& getLatestPlaneTransform() {
      return latest_plane_transform_;
    }

  protected: 
    /* Apply a series of filters (threshold depth, downsample, and remove outliers) */
    PointCloudPtr
    applyFiltersAndSegment (const PointCloudPtr & input, const ObjectRecognitionParameters & params, 
        std::vector<pcl::PointIndices> & clusters, bool transformToTableFrame)
    {
      PointCloudPtr cloud;
      cloud = thresholdDepth (input, params.min_depth, params.max_depth);
      ROS_INFO("After thresholding, %zd points left", cloud->size());
      cloud = downsample (cloud, params.downsample_leaf_size);
      ROS_INFO("After downsampling, %zd points left", cloud->size());
      cloud = removeOutliers (cloud, params.outlier_rejection_radius, params.outlier_rejection_min_neighbors);
      ROS_INFO("After outlier removal, %zd points left", cloud->size());
      Eigen::Affine3f plane_transform;
      cloud = findAndSubtractPlane (cloud, params.plane_inlier_distance_threshold, params.max_ransac_iterations, 
          &plane_transform, params.plane_up_direction);
      std::cout << "Plane transform: " << std::endl;
      std::cout << plane_transform.matrix() << std::endl;

      latest_plane_transform_ = plane_transform;
      Eigen::Affine3d plane_tr_double(plane_transform);
      geometry_msgs::PoseStamped table_pose;
      tf::poseEigenToMsg(plane_tr_double, table_pose.pose);
      table_pose.header.frame_id = camera_frame_;
      table_pose.header.stamp = ros::Time::now();
      table_pose_pub.publish(table_pose);

      ROS_INFO("After plane removal, %zd points left", cloud->size());
      if (transformToTableFrame) {
        PointCloudPtr cloud_transformed(new PointCloud);
        pcl::transformPointCloud(*cloud, *cloud_transformed, plane_transform.inverse());
        cloud = cloud_transformed;
      } 

      clusterObjects (cloud, params.cluster_tolerance, params.min_cluster_size, 
                      params.max_cluster_size, clusters);

      /*
            */
      return (cloud);
    }

    /* Estimate surface normals, keypoints, and local/global feature descriptors */
    void
    estimateFeatures (const PointCloudPtr & points, const ObjectRecognitionParameters & params,
                      SurfaceNormalsPtr & normals_out, PointCloudPtr & keypoints_out, 
                      LocalDescriptorsPtr & local_descriptors_out, GlobalDescriptorsPtr & global_descriptor_out) 
    {
      normals_out = estimateSurfaceNormals (points, params.surface_normal_radius);
      
      keypoints_out = detectKeypoints (points, normals_out, params.keypoints_min_scale, params.keypoints_nr_octaves,
                                       params.keypoints_nr_scales_per_octave, params.keypoints_min_contrast);
      
      local_descriptors_out = computeLocalDescriptors (points, normals_out, keypoints_out, 
                                                       params.local_descriptor_radius);
      
      global_descriptor_out = computeGlobalDescriptor (points, normals_out);
    }

    /* Align the points in the source model to the points in the target model */
    PointCloudPtr
    alignModelPoints (const ObjectModel & source, const ObjectModel & target, 
                      const ObjectRecognitionParameters & params, Eigen::Affine3f *transform)
    {
      Eigen::Matrix4f tform; 
      tform = computeInitialAlignment (source.keypoints, source.local_descriptors,
                                       target.keypoints, target.local_descriptors,
                                       params.initial_alignment_min_sample_distance,
                                       params.initial_alignment_max_correspondence_distance, 
                                       params.initial_alignment_nr_iterations);

      tform = refineAlignment (source.points, target.points, tform, 
                               params.icp_max_correspondence_distance, params.icp_outlier_rejection_threshold, 
                               params.icp_transformation_epsilon, params.icp_max_iterations);

      *transform = tform;
      PointCloudPtr output (new PointCloud);
      pcl::transformPointCloud (*(source.points), *output, tform);

      return (output);
    }  

    ObjectRecognitionParameters params_;
    ObjectModel naoModel;
    std::vector<ObjectModel> models_;
    GlobalDescriptorsPtr descriptors_;
    pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr kdtree_;
    float unknown_threshold;
    std::string camera_frame_;
    ros::Publisher table_pose_pub;
    Eigen::Affine3f latest_plane_transform_;
    int identifiers_;
};

#endif
