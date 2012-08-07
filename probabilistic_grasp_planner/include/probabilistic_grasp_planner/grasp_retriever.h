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

#ifndef _GRASP_RETRIEVER_H_
#define _GRASP_RETRIEVER_H_

#include <ros/ros.h>

#include <vector>
#include <map>
#include <tf/transform_broadcaster.h>
#include <object_manipulation_msgs/GraspableObject.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database/database_grasp.h>
#include <household_objects_database/database_perturbation.h>
using household_objects_database::DatabasePerturbationPtr;

#include "probabilistic_grasp_planner/forward_decls.h"

#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"

namespace probabilistic_grasp_planner {

class GraspRetriever
{
protected:
  std::string arm_name_;
  std::vector<GraspWithMetadata> grasps_;
public:
  //virtual void getGrasps(std::vector<GraspWithMetadata> &grasps) = 0;
  virtual void getGrasps(std::vector<GraspWithMetadata> &grasps) = 0;

  GraspRetriever(const std::string &arm_name) :
                   arm_name_(arm_name)
  {

  }
};
typedef boost::shared_ptr<GraspRetriever> GraspRetrieverPtr;

class DatabaseGraspRetriever : public GraspRetriever
{
private:
  //! A cache storing model_id->grasps for that model. This is private since subclasses should not share caches
  std::map<int, std::vector<household_objects_database::DatabaseGraspPtr> > grasps_cache_;

protected:
  ObjectsDatabasePtr database_;

  //! The model whose grasps this retriever will retrieve
  const household_objects_database_msgs::DatabaseModelPose &model_;

  //! Whether compliant copies should be returned or not
  bool prune_compliant_copies_;

  //! Whether only cluster representative grasps should be returned
  bool use_cluster_rep_grasps_;

  /*!
   * Given a model which the grasps are specified relative to, and a list of grasps from the database, populates a list
   * of GraspWithMetadata objects which contains the basic Grasp object for final return by the planner and
   * additional information used by the planner such as energy metrics and object coordinate frames.
   */
  void appendMetadataToGrasps(const std::vector<household_objects_database::DatabaseGraspPtr> &db_grasps,
                              std::vector<GraspWithMetadata> &grasps);
  /*!
   * Removes grasps from the list which do not match certain criteria (such as not coming from beneath the object).
   */
  void pruneGraspList(std::vector<household_objects_database::DatabaseGraspPtr> &grasps, double gripper_threshold,
                        double table_clearance_threshold);

public:
  /*!
   * Instantiates a DatabaseGraspRetriever which gets all grasps for a model (as opposed to just cluster rep grasps)
   */
  DatabaseGraspRetriever(ObjectsDatabasePtr database,
                         const household_objects_database_msgs::DatabaseModelPose &model,
                         const std::string &arm_name,
                         bool prune_compliant_copies,
						 bool use_cluster_rep_grasps);

  /*!
   * Given a list of models, retrieves all of the cluster representative grasps for those models and stores them with
   * their metadata in the grasps vector.
   */
  virtual void getGrasps(std::vector<GraspWithMetadata> &grasps);

  virtual void fetchFromDB();

};
typedef boost::shared_ptr<DatabaseGraspRetriever> DatabaseGraspRetrieverPtr;

class ClusterRepGraspRetriever : public DatabaseGraspRetriever
{
private:
  //! A cache storing model_id->{cluster rep grasps for that model}
    std::map<int, std::vector<household_objects_database::DatabaseGraspPtr> > grasps_cache_;
protected:

public:
  ClusterRepGraspRetriever(ObjectsDatabasePtr database, const household_objects_database_msgs::DatabaseModelPose &model,
                           const std::string &arm_name);
  /*!
   * Given a list of models, retrieves all of the cluster representative grasps for those models and stores them with
   * their metadata in the grasps vector.
   */
  virtual void getGrasps(std::vector<GraspWithMetadata> &grasps);

  virtual void fetchFromDB();
};
typedef boost::shared_ptr<ClusterRepGraspRetriever> ClusterRepGraspRetrieverPtr;

class DebugClusterRepGraspRetriever : public ClusterRepGraspRetriever
{
public:
  DebugClusterRepGraspRetriever(ObjectsDatabasePtr database,
                                const household_objects_database_msgs::DatabaseModelPose &model,
                                const std::string &arm_name) :
                                  ClusterRepGraspRetriever(database, model, arm_name)
  {

  }

  /*!
   * Given a list of models, retrieves all of the cluster representative grasps for those models and stores them with
   * their metadata in the grasps vector.
   */
  virtual void getGrasps(std::vector<GraspWithMetadata> &grasps);
};

/*!
 * Given a grasp, this gets all of the precomputed perturbations of this grasp from the database
 */
class PerturbationGraspRetriever : public DatabaseGraspRetriever
{
private:
  const GraspWithMetadata *gstar_;
protected:
  std::vector<DatabasePerturbationPtr> perturbations_;
  virtual void fetchFromDB();
public:
  PerturbationGraspRetriever(ObjectsDatabasePtr database,
                             const household_objects_database_msgs::DatabaseModelPose &model,
                             const std::string &arm_name);
  void setGrasp(const GraspWithMetadata *gstar)
  {
    gstar_ = gstar;
  }

  void getGrasps(std::vector<GraspWithMetadata> &grasps);
};

class OnlinePerturbationGraspRetriever : GraspRetriever
{
private:
  const GraspWithMetadata *gstar_;

public:
  OnlinePerturbationGraspRetriever(const GraspWithMetadata *gstar, const std::string &arm_name) :
    GraspRetriever(arm_name), gstar_(gstar)
  {

  }

  void getGrasps(std::vector<GraspWithMetadata> &grasps);
};

class ClusterPlannerGraspRetriever : public GraspRetriever
{
private:
  tf::TransformBroadcaster tf_broadcaster;
  ros::ServiceClient cluster_planner_srv_;
  std::vector<object_manipulation_msgs::Grasp> grasps_from_cluster_planner_;
  sensor_msgs::PointCloud cloud_;
protected:
  void appendGraspsFromClusterPlanner(std::vector<GraspWithMetadata> &grasps);
  virtual void fetchFromPlanner(const object_manipulation_msgs::GraspableObject &graspable_object);
public:
  ClusterPlannerGraspRetriever(ros::NodeHandle &nh, const std::string &cluster_planner_name,
                               const object_manipulation_msgs::GraspableObject &graspable_object,
                               const std::string &arm_name);

  /*!
   * Gets the grasps from both the underlying GraspRetriever and the cluster planner call
   */
  virtual void getGrasps(std::vector<GraspWithMetadata> &grasps);
};
} //namespace

#endif
