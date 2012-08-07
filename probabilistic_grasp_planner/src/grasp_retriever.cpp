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

#include <household_objects_database/objects_database.h>
using household_objects_database::ObjectsDatabasePtr;
#include <household_objects_database/database_grasp.h>
using household_objects_database::DatabaseGraspPtr;
#include <household_objects_database/database_perturbation.h>
using household_objects_database::DatabasePerturbationPtr;

#include <object_manipulator/tools/hand_description.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulation_msgs/GraspPlanningErrorCode.h>
#include <tf/transform_broadcaster.h>

//#define PROF_ENABLED
//#include <profiling/profiling.h>

#include "probabilistic_grasp_planner/grasp_retriever.h"

//PROF_DECLARE(CLUSTER_REP_CREATE_PROF)
//PROF_DECLARE(CLUSTER_PLANNER_CREATE_PROF)

namespace probabilistic_grasp_planner {

//! Prune grasps that require gripper to be open all the way, or that are marked in db as colliding with table
/*! Use negative value for table_clearance_threshold if no clearing should be done
  based on table clearance.
*/
void DatabaseGraspRetriever::pruneGraspList(std::vector<DatabaseGraspPtr> &grasps,
                                               double gripper_threshold,
                                               double table_clearance_threshold)
{
  std::vector<DatabaseGraspPtr>::iterator prune_it = grasps.begin();
  int pruned = 0;
  while ( prune_it != grasps.end() )
  {
    //by mistake, table clearance in database is currently in mm
    if ((*prune_it)->final_grasp_posture_.get().joint_angles_[0] > gripper_threshold ||
        (table_clearance_threshold >= 0.0 && (*prune_it)->table_clearance_.get() < table_clearance_threshold*1.0e3) )
    {
      prune_it = grasps.erase(prune_it);
      pruned++;
    }
    else
    {
      prune_it++;
    }
  }
  ROS_DEBUG("Database grasp planner: pruned %d grasps for table collision or gripper angle above threshold", pruned);
}

void DatabaseGraspRetriever::appendMetadataToGrasps(const std::vector<DatabaseGraspPtr> &db_grasps,
                                                      std::vector<GraspWithMetadata> &grasps)
{
  grasps.reserve(grasps.size()+db_grasps.size());

  BOOST_FOREACH(DatabaseGraspPtr db_grasp, db_grasps) {
    GraspWithMetadata grasp;

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
    //grasp.grasp_.grasp_pose = db_grasp->final_grasp_pose_.get().pose_;

    grasp.energy_function_score_ = db_grasp->quality_.get();
    grasp.model_id_ = db_grasp->scaled_model_id_.get();
    grasp.grasp_id_ = db_grasp->id_.get();

    tf::Stamped<tf::Pose> base_to_object;

    tf::poseStampedMsgToTF(model_.pose,base_to_object);

    tf::Pose object_to_grasp;
    tf::poseMsgToTF(db_grasp->final_grasp_pose_.get().pose_, object_to_grasp);

    //tf::Stamped<tf::Pose> base_to_best_object;
    //tf::poseStampedMsgToTF(best_model_pose_.pose, base_to_best_object);
    grasp.object_pose_ = base_to_object;

    //tf::Pose best_object_to_grasp(base_to_best_object.inverse()*base_to_object*object_to_grasp);

    tf::Pose base_to_grasp(base_to_object * object_to_grasp);

    tf::poseTFToMsg(base_to_grasp,grasp.grasp_.grasp_pose);


    double wrist_to_tool_point_offset_ = 0.13;

    tf::Pose offset_tf(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 5cm from wrist
    grasp.tool_point_pose_ = tf::Stamped<tf::Pose>(base_to_grasp * offset_tf,
        grasp.object_pose_.stamp_, grasp.object_pose_.frame_id_);

    //insert the new grasp in the list
    grasps.push_back(grasp);
  }

}
DatabaseGraspRetriever::DatabaseGraspRetriever(ObjectsDatabasePtr database,
                                               const household_objects_database_msgs::DatabaseModelPose &model,
                                               const std::string &arm_name,
                                               bool prune_compliant_copies,
											   bool use_cluster_rep_grasps) :
  GraspRetriever(arm_name),
  database_(database), model_(model), prune_compliant_copies_(prune_compliant_copies), use_cluster_rep_grasps_(use_cluster_rep_grasps)
{

}

void DatabaseGraspRetriever::fetchFromDB()
{
  /*
  if (retrieve_from_map(grasps_cache_, model_id, db_grasps))
  {
    ROS_DEBUG("Robust Database Grasp Planning Node: retrieved %zd grasps from cache", db_grasps.size());
  }
  else
  {

    grasps_cache_.insert(std::make_pair(model_id, db_grasps));
  }
  */
  std::vector<DatabaseGraspPtr> db_grasps;
  int model_id=model_.model_id;
  //retrieve the raw grasps from the database
  if (use_cluster_rep_grasps_)
  {
	if (!database_->getClusterRepGrasps(model_id, object_manipulator::handDescription().handDatabaseName(arm_name_), db_grasps))
	{
	  ROS_ERROR("Robust Database Grasp Planning Node: grasp retrieval error");
	}
  }
  else
  {
	if (!database_->getGrasps(model_id, object_manipulator::handDescription().handDatabaseName(arm_name_), db_grasps))
	{
	  ROS_ERROR("Robust Database Grasp Planning Node: grasp retrieval error");
	}
  }

  ROS_DEBUG("Robust Database Grasp Planning Node: retrieved %zd grasps from database", db_grasps.size());
  //pruneGraspList(db_grasps, 0.5,0.0);

  if (prune_compliant_copies_)
  {
    std::vector<DatabaseGraspPtr>::iterator it = db_grasps.begin();
    while (it!=db_grasps.end())
    {
      if ((*it)->compliant_copy_.get()) it = db_grasps.erase(it);
      else it++;
    }
  }

  appendMetadataToGrasps(db_grasps, grasps_);
}

void DatabaseGraspRetriever::getGrasps(std::vector<GraspWithMetadata> &grasps)
{
  grasps.insert(grasps.end(), grasps_.begin(), grasps_.end());
}

ClusterRepGraspRetriever::ClusterRepGraspRetriever(ObjectsDatabasePtr database,
                                                   const household_objects_database_msgs::DatabaseModelPose &model,
                                                   const std::string &arm_name) :
  DatabaseGraspRetriever(database, model, arm_name, true, false)
{
  //PROF_TIMER_FUNC(CLUSTER_REP_CREATE_PROF);

}

void ClusterRepGraspRetriever::fetchFromDB()
{

  /*
  if (retrieve_from_map(grasps_cache_, model_id, db_grasps))
  {
    ROS_DEBUG("Robust Database Grasp Planning Node: retrieved %zd grasps from cache", db_grasps.size());
  }
  else
  {

    grasps_cache_.insert(std::make_pair(model_id, db_grasps));
  }
  */
  std::vector<DatabaseGraspPtr> db_grasps;
  int model_id=model_.model_id;
  //retrieve the raw grasps from the database
  if (!database_->getClusterRepGrasps(model_id,
                                      object_manipulator::handDescription().handDatabaseName(arm_name_), db_grasps))
  {
    ROS_ERROR("Robust Database Grasp Planning Node: grasp retrieval error");
  }

  ROS_WARN("Size from DB: %zd",db_grasps.size());

  ROS_DEBUG("Robust Database Grasp Planning Node: retrieved %zd grasps from database", db_grasps.size());
  //pruneGraspList(db_grasps, 0.5,0.0);

  ROS_WARN("Size after pruning: %zd",db_grasps.size());
  ROS_WARN("Size before appending: %zd",grasps_.size());
  appendMetadataToGrasps(db_grasps, grasps_);

  ROS_WARN("Size after appending: %zd",grasps_.size());
}

void ClusterRepGraspRetriever::getGrasps(std::vector<GraspWithMetadata> &grasps)
{
  grasps.insert(grasps.end(), grasps_.begin(), grasps_.end());
}

/*!
 * Only return 5 grasps from the list for easier debugging
 */
void DebugClusterRepGraspRetriever::getGrasps(std::vector<GraspWithMetadata> &grasps)
{
  std::vector<GraspWithMetadata>::iterator begin_plus_five(grasps_.begin());
  std::advance(begin_plus_five, 5);
  grasps.insert(grasps.end(), grasps_.begin(), begin_plus_five);
}

PerturbationGraspRetriever::PerturbationGraspRetriever(ObjectsDatabasePtr database,
                                                       const household_objects_database_msgs::DatabaseModelPose &model,
                                                       const std::string &arm_name) :
  DatabaseGraspRetriever(database, model, arm_name, false, true), gstar_(NULL)
{
  ROS_DEBUG("Created new perturbation grasp retriever for model_id %d",model.model_id);
  fetchFromDB();
}

void PerturbationGraspRetriever::fetchFromDB()
{
  std::vector<DatabasePerturbationPtr> perturbations;
  database_->getAllPerturbationsForModel(model_.model_id, perturbations);

  ROS_DEBUG("Retrieved %zd perturbations of grasps for model %d from the DB", perturbations.size(), model_.model_id);

  perturbations_.insert(perturbations_.begin(), perturbations.begin(), perturbations.end());
}

void PerturbationGraspRetriever::getGrasps(std::vector<GraspWithMetadata> &grasps)
{
  if (gstar_ == NULL)
  {
    ROS_ERROR("Requested perturbations for a grasp, but grasp is not set.");
    return;
  }

  //!TODO: make this not hardcoded
  grasps.reserve(grasps.size()+250);
  BOOST_FOREACH(DatabasePerturbationPtr &perturbation, perturbations_)
  {
    int grasp_id = perturbation->grasp_id_.get();
    if (grasp_id == gstar_->grasp_id_)
    {
      //! TODO: check all this
      GraspWithMetadata grasp_with_metadata;
      grasp_with_metadata.model_id_ = model_.model_id;
      grasp_with_metadata.grasp_id_ = grasp_id;
      grasp_with_metadata.energy_function_score_ = perturbation->score_.get();
      grasp_with_metadata.object_pose_ = gstar_->object_pose_;

      object_manipulation_msgs::Grasp grasp;
      //grasp.grasp_pose

      //! TODO: add correct pose from deltas
      //grasp_with_metadata.
      std::vector<double> deltas = perturbation->deltas_.get();

      grasps.push_back(grasp_with_metadata);
    }
  }
}

ClusterPlannerGraspRetriever::ClusterPlannerGraspRetriever(ros::NodeHandle &nh, const std::string &cluster_planner_name,
                                                     const object_manipulation_msgs::GraspableObject &graspable_object,
                                                           const std::string &arm_name) :
  GraspRetriever(arm_name)
{
  //PROF_TIMER_FUNC(CLUSTER_PLANNER_CREATE_PROF);
  cluster_planner_srv_ = register_service<object_manipulation_msgs::GraspPlanning>(nh, cluster_planner_name);
  cloud_ = graspable_object.cluster;
  fetchFromPlanner(graspable_object);
}

void ClusterPlannerGraspRetriever::fetchFromPlanner(const object_manipulation_msgs::GraspableObject &graspable_object)
{
  object_manipulation_msgs::GraspPlanning planner_call;
  //! Note that this is leaving several of the grasp planning arguments empty. If the cluster planner changes this will
  //! need to change as well.
  planner_call.request.target = graspable_object;
  planner_call.request.arm_name = arm_name_;
  cluster_planner_srv_.call(planner_call);
  if (planner_call.response.error_code.value != object_manipulation_msgs::GraspPlanningErrorCode::SUCCESS)
  {
   ROS_ERROR("Call to cluster planner failed!");
  }
  grasps_from_cluster_planner_ = planner_call.response.grasps;
  ROS_INFO("Got %zd grasps from the cluster planner",grasps_from_cluster_planner_.size());
}

void ClusterPlannerGraspRetriever::getGrasps(std::vector<GraspWithMetadata> &grasps)
{

  //! Add the grasps from the cluster planner
  appendGraspsFromClusterPlanner(grasps);
}

void ClusterPlannerGraspRetriever::appendGraspsFromClusterPlanner(std::vector<GraspWithMetadata> &grasps)
{
  grasps.reserve(grasps.size()+grasps_from_cluster_planner_.size());


  BOOST_FOREACH(object_manipulation_msgs::Grasp &grasp, grasps_from_cluster_planner_)
  {
    GraspWithMetadata grasp_with_metadata;
    grasp_with_metadata.grasp_ = grasp;
    //grasp_with_metadata.grasp_.success_probability = 0.0;
    grasp_with_metadata.model_id_ = -1;
    grasp_with_metadata.grasp_id_ = -1;

    grasp_with_metadata.object_pose_.setIdentity();
    grasp_with_metadata.object_pose_.frame_id_ = cloud_.header.frame_id;
    grasp_with_metadata.object_pose_.stamp_ = cloud_.header.stamp;

    tf::Pose grasp_in_base_frame;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_in_base_frame);

    //tf::Pose best_object_to_grasp(base_to_best_object.inverse() * grasp_in_base_frame);

    tf::poseTFToMsg(grasp_in_base_frame,grasp_with_metadata.grasp_.grasp_pose);

    double wrist_to_tool_point_offset_ = 0.13;

    tf::Pose grasp_to_tool_point(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 13cm from wrist
    grasp_with_metadata.tool_point_pose_ = tf::Stamped<tf::Pose>(
        grasp_in_base_frame * grasp_to_tool_point, cloud_.header.stamp,
        cloud_.header.frame_id);


    grasps.push_back(grasp_with_metadata);
  }
}


}
