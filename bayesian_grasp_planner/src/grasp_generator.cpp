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

#include "bayesian_grasp_planner/grasp_generator.h"
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulator/tools/hand_description.h>

namespace bayesian_grasp_planner {

GraspGeneratorServiceCaller::GraspGeneratorServiceCaller(ros::NodeHandle &nh, const std::string service_name,
						 const object_manipulation_msgs::GraspableObject &graspable_object, 
							 const std::string arm_name) : 
  object_(graspable_object),arm_name_(arm_name),service_name_(service_name)
{
  service_ = register_service<object_manipulation_msgs::GraspPlanning>(nh, service_name);
}

void GraspGeneratorServiceCaller::generateGrasps()
{
  object_manipulation_msgs::GraspPlanning planner_call;

  ROS_INFO("calling grasp service %s", service_name_.c_str());

  //leaves several grasp planning arguments empty (OK for cluster planner, maybe not for others)
  planner_call.request.target = object_;
  planner_call.request.arm_name = arm_name_;
  service_.call(planner_call);
  if (planner_call.response.error_code.value != object_manipulation_msgs::GraspPlanningErrorCode::SUCCESS)
  {
   ROS_ERROR("Call to service-based planner failed!");
  }
  ROS_INFO("Got %zd grasps from the service-based planner", planner_call.response.grasps.size());
  appendMetadataToGrasps(planner_call.response.grasps, grasps_);
}


void GraspGeneratorServiceCaller::appendMetadataToGrasps(std::vector<object_manipulation_msgs::Grasp> &grasp_msgs,
							 std::vector<GraspWM> &grasps)
{
  grasps.reserve(grasps.size()+grasp_msgs.size());


  BOOST_FOREACH(object_manipulation_msgs::Grasp &grasp, grasp_msgs)
  {
    GraspWM grasp_with_metadata;
    grasp_with_metadata.grasp_ = grasp;
    grasp_with_metadata.model_id_ = -1;
    grasp_with_metadata.grasp_id_ = -1;

    grasp_with_metadata.object_pose_.setIdentity();
    grasp_with_metadata.object_pose_.frame_id_ = object_.cluster.header.frame_id;
    grasp_with_metadata.object_pose_.stamp_ = object_.cluster.header.stamp;
    grasp_with_metadata.energy_function_score = grasp.success_probability;

    tf::Pose grasp_in_base_frame;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_in_base_frame);
    tf::poseTFToMsg(grasp_in_base_frame,grasp_with_metadata.grasp_.grasp_pose);
    double wrist_to_tool_point_offset_ = 0.13;
    tf::Pose grasp_to_tool_point(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 13cm from wrist
    grasp_with_metadata.tool_point_pose_ = tf::Stamped<tf::Pose>(
        grasp_in_base_frame * grasp_to_tool_point, object_.cluster.header.stamp,
        object_.cluster.header.frame_id);

    grasps.push_back(grasp_with_metadata);
  }
}

GraspGeneratorDatabaseRetriever::GraspGeneratorDatabaseRetriever
  (boost::shared_ptr<household_objects_database::ObjectsDatabase> database,
   household_objects_database_msgs::DatabaseModelPose model, const std::string arm_name,
   bool cluster_reps) : 
    database_(database),
    model_(model),
    arm_name_(arm_name),
    cluster_reps_(cluster_reps)
{
  if (cluster_reps_) ROS_INFO("creating cluster reps GraspGeneratorDatabaseRetriever for model_id %d", model.model_id);
  else ROS_INFO("creating GraspGeneratorDatabaseRetriever for model_id %d", model.model_id);
}

void GraspGeneratorDatabaseRetriever::generateGrasps()
{
  std::vector<household_objects_database::DatabaseGraspPtr> db_grasps;
  ROS_INFO("generating grasps by pulling them from database for model %d", model_.model_id);

  //retrieve the raw grasps from the database
  bool result;
  if (cluster_reps_)
  {
    result = database_->getClusterRepGrasps(model_.model_id, 
                                        object_manipulator::handDescription().handDatabaseName(arm_name_), db_grasps);
  }
  else
  {
    result = database_->getGrasps(model_.model_id, 
                                  object_manipulator::handDescription().handDatabaseName(arm_name_), db_grasps);
  }
  if (!result) ROS_ERROR("Robust Database Grasp Planning Node: grasp retrieval error");
  ROS_WARN("Size from DB: %zd",db_grasps.size());
  appendMetadataToGrasps(db_grasps, grasps_);
  ROS_WARN("Size after appending: %zd",grasps_.size());
}


void GraspGeneratorDatabaseRetriever::appendMetadataToGrasps(
                                            const std::vector<household_objects_database::DatabaseGraspPtr> &db_grasps,
					    std::vector<GraspWM> &grasps)
{
  grasps.reserve(grasps.size()+db_grasps.size());

  BOOST_FOREACH(household_objects_database::DatabaseGraspPtr db_grasp, db_grasps) {
    GraspWM grasp;

    ROS_ASSERT( db_grasp->final_grasp_posture_.get().joint_angles_.size() ==
              db_grasp->pre_grasp_posture_.get().joint_angles_.size() );
    ROS_ASSERT(db_grasp->scaled_model_id_.get() ==  model_.model_id);

    std::vector<std::string> joint_names = object_manipulator::handDescription().handJointNames(arm_name_);
    if (object_manipulator::handDescription().handDatabaseName(arm_name_) != "WILLOW_GRIPPER_2010")
    {
      //check that the number of joints in the ROS description of this hand
      //matches the number of values we have in the database
      if (joint_names.size() != db_grasp->final_grasp_posture_.get().joint_angles_.size())
      {
        ROS_ERROR("Database grasp specification does not match ROS description of hand. "
                  "Hand is expected to have %zd joints, but database grasp specifies %zd values",
                  joint_names.size(), db_grasp->final_grasp_posture_.get().joint_angles_.size());
        continue;
      }
      //for now we silently assume that the order of the joints in the ROS description of
      //the hand is the same as in the database description
      grasp.grasp_.pre_grasp_posture.name = joint_names;
      grasp.grasp_.grasp_posture.name = joint_names;
      grasp.grasp_.pre_grasp_posture.position = db_grasp->pre_grasp_posture_.get().joint_angles_;
      grasp.grasp_.grasp_posture.position = db_grasp->final_grasp_posture_.get().joint_angles_;
    }
    else
    {
      //unfortunately we have to hack this, as the grasp is really defined by a single
      //DOF, but the urdf for the PR2 gripper is not well set up to do that
      if ( joint_names.size() != 4 || db_grasp->final_grasp_posture_.get().joint_angles_.size() != 1)
      {
        ROS_ERROR("PR2 gripper specs and database grasp specs do not match expected values");
        continue;
      }
      grasp.grasp_.pre_grasp_posture.name = joint_names;
      grasp.grasp_.grasp_posture.name = joint_names;
      //replicate the single value from the database 4 times
      grasp.grasp_.pre_grasp_posture.position.resize( joint_names.size(),
                                                   db_grasp->pre_grasp_posture_.get().joint_angles_.at(0));
      grasp.grasp_.grasp_posture.position.resize( joint_names.size(),
                                               db_grasp->final_grasp_posture_.get().joint_angles_.at(0));
    }

    //for now the effort and approach distances are not in the database 
    //so we hard-code them in here; this will change at some point
    grasp.grasp_.pre_grasp_posture.effort.resize(joint_names.size(), 100);
    grasp.grasp_.grasp_posture.effort.resize(joint_names.size(), 50);
    grasp.grasp_.desired_approach_distance = 0.10;
    grasp.grasp_.min_approach_distance = 0.05;

    grasp.grasp_.success_probability = 0.0;

    //the pose of the grasp
    grasp.energy_function_score = db_grasp->scaled_quality_.get();
    grasp.model_id_ = db_grasp->scaled_model_id_.get();
    grasp.grasp_id_ = db_grasp->id_.get();
    tf::Stamped<tf::Pose> base_to_object;
    tf::poseStampedMsgToTF(model_.pose,base_to_object);
    tf::Pose object_to_grasp;
    tf::poseMsgToTF(db_grasp->final_grasp_pose_.get().pose_, object_to_grasp);
    grasp.object_pose_ = base_to_object;
    tf::Pose base_to_grasp(base_to_object * object_to_grasp);
    tf::poseTFToMsg(base_to_grasp,grasp.grasp_.grasp_pose);

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 5cm from wrist
    double wrist_to_tool_point_offset_ = 0.13;
    tf::Pose offset_tf(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));
    grasp.tool_point_pose_ = tf::Stamped<tf::Pose>(base_to_grasp * offset_tf,
        grasp.object_pose_.stamp_, grasp.object_pose_.frame_id_);

    //insert the new grasp in the list
    grasps.push_back(grasp);
  }
}


} //namespace
