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

#ifndef GRASP_GENERATOR_H
#define GRASP_GENERATOR_H

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/Grasp.h>
#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database/objects_database.h>
#include "household_objects_database/database_grasp.h"
#include "household_objects_database/database_perturbation.h"
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"

namespace bayesian_grasp_planner {

class GraspGenerator
{
protected:
  std::vector<GraspWM> grasps_;
 
public:
  virtual void generateGrasps() = 0;

  const std::vector<GraspWM> & getGrasps() {return grasps_;}

  void getGrasps(std::vector<GraspWM> &grasps)
  {
    grasps.insert( grasps.end(), grasps_.begin(), grasps_.end());
  }

};

class GraspGeneratorServiceCaller : public GraspGenerator
{
private:
  ros::ServiceClient service_;
  const object_manipulation_msgs::GraspableObject object_;
  const std::string arm_name_;
  const std::string service_name_;

  void appendMetadataToGrasps(std::vector<object_manipulation_msgs::Grasp> &grasp_msgs,
			      std::vector<GraspWM> &grasps);

public:
  GraspGeneratorServiceCaller(ros::NodeHandle &nh, const std::string service_name, 
			      const object_manipulation_msgs::GraspableObject &graspable_object,
			      const std::string arm_name);

  void generateGrasps();
};

class GraspGeneratorDatabaseRetriever : public GraspGenerator
{
private:
  boost::shared_ptr<household_objects_database::ObjectsDatabase> database_;
  household_objects_database_msgs::DatabaseModelPose model_;
  const std::string arm_name_;

  void appendMetadataToGrasps(const std::vector<household_objects_database::DatabaseGraspPtr> &db_grasps,
		       std::vector<GraspWM> &grasps);

  bool cluster_reps_;

public:
  GraspGeneratorDatabaseRetriever(boost::shared_ptr<household_objects_database::ObjectsDatabase> database,
                                  household_objects_database_msgs::DatabaseModelPose model, const std::string arm_name,
                                  bool cluster_reps);

  void generateGrasps();
};


}


#endif
