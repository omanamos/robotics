#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <string>
#include <vector>
#include <Eigen/Core>
#include "cse481/typedefs.h"
#include "cse481/template_aligner.h"

// Holds an object template and name
class ObjectTemplate {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectTemplate(const std::string &name, const PointCloud &model) : 
      name_(name)
    {
      model_.reset(new PointCloud(model));
    }
    const PointCloud & getModel() const {
      return *model_;
    }
    const std::string & getName() const {
      return name_;
    }
  protected:
    std::string name_;
    PointCloud::Ptr model_;
};

// Represents a template that when transformed, matches an object
class ObjectMatch {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ObjectMatch(const ObjectTemplate &matched_template, 
        const AffineTransform &final_transformation,
        const float fitness) : 
      matched_template_(matched_template),
      final_transformation_(final_transformation),
      fitness_(fitness)
  { }

    const ObjectTemplate & getTemplate() const {
      return matched_template_;
    }

    const AffineTransform & getTransformation() const {
      return final_transformation_;
    }

    const float & getFitness() const {
      return fitness_;
    }

  protected:
    ObjectTemplate matched_template_;
    AffineTransform final_transformation_;
    float fitness_;
};

// Main class for detecting new and old objects on a table in a scene
class ObjectDetector {
  public:
    ObjectDetector() :fitness_match_threshold_(0.01){ }

    virtual ~ObjectDetector() { }

    void addTemplate(const ObjectTemplate &t);

    void detectObjectsInScene(const sensor_msgs::PointCloud2 &scene, 
        std::vector<ObjectMatch> *recognized_objects,
        std::vector<ObjectMatch> *unrecognized_objects);

  protected:
    std::vector<ObjectTemplate> templates_;
    TemplateAligner template_align_;
    float fitness_match_threshold_;
};

#endif
