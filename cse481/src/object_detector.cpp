#include <string>
#include <vector>
#include <boost/foreach.hpp>

#include "cse481/typedefs.h"
#include "cse481/table_cluster_detector.h"
#include "cse481/template_aligner.h"
#include "cse481/object_detector.h"

void ObjectDetector::addTemplate(const ObjectTemplate &t) {
  templates_.push_back(t);
  // create feature cloud
  FeatureCloud template_cloud; 
  template_cloud.setInputCloud(t.getModel().makeShared());
  template_align_.addTemplateCloud(template_cloud);
}

void addUnknownObject(int & unknownCount, const PointCloud & cluster, 
    std::vector<ObjectMatch> *unrecognized_objects) {
  std::stringstream ss;
  ss << "unknown" << unknownCount++;
  ObjectTemplate newTemplate(ss.str(), cluster);
  ObjectMatch newMatch(newTemplate, Eigen::Matrix4f::Identity(), -1.0);
  unrecognized_objects->push_back(newMatch);
}

void ObjectDetector::detectObjectsInScene(const sensor_msgs::PointCloud2 &scene, 
        std::vector<ObjectMatch> *recognized_objects,
        std::vector<ObjectMatch> *unrecognized_objects)
{
  TableClusterDetector tcd;
  // locate table in scene
  // find objects on table
  std::vector<PointCloud> clusters = tcd.findTableClusters(scene);
  
  // for each object
  int numUnknown=0;
  BOOST_FOREACH(PointCloud cluster, clusters) {
    // Handle the case where we have no templates
    if (template_align_.getTemplateCount() > 0) {
      FeatureCloud target_cloud; 
      target_cloud.setInputCloud(cluster.makeShared());
      template_align_.setTargetCloud(target_cloud);
      // find best-aligned template among templates
      // Find the best template alignment
      TemplateAligner::Result best_alignment;
      int best_index = template_align_.findBestAlignment(best_alignment);
      // TODO: check the color as well
      // if fitness is above a threshold, it is a match
      if (best_alignment.fitness_score < fitness_match_threshold_) {
        ObjectMatch match(templates_[best_index], 
            best_alignment.final_transformation, best_alignment.fitness_score);
        recognized_objects->push_back(match);
      } else {// else it is a new object
        addUnknownObject(numUnknown, cluster, unrecognized_objects);
      }
    } else {// It must be a new object 
      addUnknownObject(numUnknown, cluster, unrecognized_objects);
    }
  }
}


