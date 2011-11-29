/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Marius Muja and Matei Ciocarlie

#include <cstdlib>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include "cse481/marker_generator.h"
#include "cse481/Table.h"
#include "cse481/typedefs.h"
/*! The point cloud is a set of points belonging to the plane, in the plane coordinate system
  (with the origin in the plane and the z axis normal to the plane).

  It is the responsibility of the caller to set the appropriate pose for the marker so that
  it shows up in the right reference frame.
 */
visualization_msgs::Marker MarkerGenerator::getTableMarker(float xmin, float xmax, float ymin, float ymax)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration();

  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;

  marker.points.resize(5);
  marker.points[0].x = xmin;
  marker.points[0].y = ymin;
  marker.points[0].z = 0;
  
  marker.points[1].x = xmin;
  marker.points[1].y = ymax;
  marker.points[1].z = 0;
  
  marker.points[2].x = xmax;
  marker.points[2].y = ymax;
  marker.points[2].z = 0;
  
  marker.points[3].x = xmax;
  marker.points[3].y = ymin;
  marker.points[3].z = 0;
  
  marker.points[4].x = xmin;
  marker.points[4].y = ymin;
  marker.points[4].z = 0;

  marker.points.resize(6);
  marker.points[5].x = xmin;
  marker.points[5].y = ymin;
  marker.points[5].z = 0.02;
   
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker MarkerGenerator::getCloudMarker(
    const cse481::PointCloud& cloud, 
    const std_msgs::ColorRGBA& color)
{
  static bool first_time = true;
  if (first_time) {
    srand ( time(NULL) );
    first_time = false;
  }

  //create the marker
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.002;
  marker.scale.y = 0.002;
  marker.scale.z = 1.0;

  if (color.a == 0.0) {
    marker.color.r = ((double)rand())/RAND_MAX;
    marker.color.g = ((double)rand())/RAND_MAX;
    marker.color.b = ((double)rand())/RAND_MAX;
    marker.color.a = 1.0;
  } else {
    marker.color = color;
  }
  for(size_t i=0; i<cloud.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    marker.points.push_back(p);
  }

  //the caller must decide the header; we are done here
  return marker;
}
