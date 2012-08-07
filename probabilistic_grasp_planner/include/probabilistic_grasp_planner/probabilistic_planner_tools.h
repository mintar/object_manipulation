/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

// Author(s): Peter Brook

#ifndef _ROBUST_PLANNER_TOOLS_H_
#define _ROBUST_PLANNER_TOOLS_H_

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

#include <map>
#include <cmath>

#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

#include "probabilistic_grasp_planner/forward_decls.h"

namespace probabilistic_grasp_planner {

class GraspWithMetadata
{
private:

public:
  int grasp_id_;
  int model_id_;
  double energy_function_score_;
  object_manipulation_msgs::Grasp grasp_;
  tf::Stamped<tf::Pose> object_pose_;
  tf::Stamped<tf::Pose> tool_point_pose_;
  double success_probability;
  std::vector<std::pair<int,std::pair<double,double> > >debug_probabilities;

  GraspWithMetadata() : success_probability(0.0)
  {

  }

  void getDistance(const GraspWithMetadata &other, double &cartesian_dist, double &rotation_dist, bool debug=false) const
  {
    tf::Vector3 gstar_position = tool_point_pose_.getOrigin();
    tf::Quaternion gstar_orientation = tool_point_pose_.getRotation();

    tf::Vector3 grasp_position = other.tool_point_pose_.getOrigin();
    tf::Quaternion grasp_orientation = other.tool_point_pose_.getRotation();

    if (debug) ROS_INFO("grasp: %f %f %f; other: %f %f %f", 
			gstar_position.x(),gstar_position.y(),gstar_position.z(),
			grasp_position.x(),grasp_position.y(),grasp_position.z());

    tf::Vector3 delta_position = gstar_position - grasp_position;
    cartesian_dist = delta_position.length();

    rotation_dist = gstar_orientation.angleShortestPath(grasp_orientation);
  }
};

struct ObjectRepresentation
{
  object_manipulation_msgs::GraspableObject object;
  boost::shared_ptr<GraspSuccessProbabilityComputer> grasp_success_computer;
  boost::shared_ptr<GraspSuccessProbabilityComputer> precise_grasp_success_computer;
  boost::shared_ptr<GraspRetriever> grasp_retriever;
  double probability;
};


inline bool operator< (const GraspWithMetadata &g1, const GraspWithMetadata &g2)
{
  return g1.grasp_.success_probability < g2.grasp_.success_probability;
}

template <class svcType>
inline ros::ServiceClient register_service(ros::NodeHandle &nh, const std::string &service_name)
{
  while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for %s service to come up", service_name.c_str());
  }
  if (!nh.ok()) exit(0);
  return nh.serviceClient<svcType>(service_name, true);
}

//! Wrapper class for service clients to perform initialization on first use
/*! When the client is first used, it will check for the existence of the service
  and wait until the service becomes available.
 */
template <class ServiceDataType>
class ServiceWrapper
{
 private:
  //! Has the service client been initialized or not
  bool initialized_;
  //! The name of the service
  std::string service_name_;
  //! The node handle to be used when initializing services
  ros::NodeHandle nh_;
  //! The actual client handle
  ros::ServiceClient client_;
 public:
 ServiceWrapper(std::string service_name) : initialized_(false), 
    service_name_(service_name),
    nh_("")
    {}
  
  //! Returns reference to client. On first use, initializes (and waits for) client. 
  ros::ServiceClient& client() 
  {
    if (!initialized_)
    {
      while ( !ros::service::waitForService(service_name_, ros::Duration(2.0)) && nh_.ok() )
      {
	ROS_INFO_STREAM("Waiting for service: " << service_name_);
      }
      if (!nh_.ok()) exit(0);
      client_ = nh_.serviceClient<ServiceDataType>(service_name_);	
      initialized_ = true;
    }
    return client_;
  }
};


template <typename KeyType, typename ValueType>
static bool retrieve_from_map(std::map<KeyType, ValueType> &map, KeyType &key, ValueType &value)
{
  typename std::map<KeyType, ValueType>::iterator it = map.find(key);

  if (it != map.end())
  {
    value = it->second;
    return true;
  }
  return false;
}

} //namespace

#endif
