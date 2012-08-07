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

#ifndef PROBABILISTIC_GRASP_PLANNER_H_
#define PROBABILISTIC_GRASP_PLANNER_H_

#include <object_manipulator/tools/grasp_marker_publisher.h>

#include "probabilistic_grasp_planner/forward_decls.h"

#include "probabilistic_grasp_planner/recognition_probability_computer.h"
#include "probabilistic_grasp_planner/grasp_success_probability_computer.h"
#include "probabilistic_grasp_planner/grasp_retriever.h"

namespace probabilistic_grasp_planner {

class ProbabilisticGraspPlanner
{
private:
  ObjectsDatabasePtr database_;

  ros::NodeHandle nh_;

  //! GMP which shows the grasps returned by the planner, colored by relative probability compared to the best grasp
  object_manipulator::GraspMarkerPublisher *grasp_marker_publisher_;

  //! GMP which shows all grasps and colors based on relative probability
  object_manipulator::GraspMarkerPublisher *debug_grasp_marker_publisher_;

  //! GMP for grasps before pruning
  object_manipulator::GraspMarkerPublisher debug_preprune_grasp_marker_publisher_;

  //! GMP for grasps after pruning but before clustering
  object_manipulator::GraspMarkerPublisher debug_precluster_grasp_marker_publisher_;

  //! GMP for grasps after pruning and clustering but before second pass
  object_manipulator::GraspMarkerPublisher debug_postprune_grasp_marker_publisher_;

  //! GMP which colors grasps based on their rank in the sorted list
  object_manipulator::GraspMarkerPublisher *rank_grasp_marker_publisher_;

  /*!
   * Publishes the grasps to rviz with specific namespaces for grasps belonging to each different model, colored with
   * green being the best grasp and red being the worst.
   */
  void visualizeGrasps(const std::vector<GraspWithMetadata> &grasps, 
                        object_manipulator::GraspMarkerPublisher *grasp_marker_pub, bool color_by_rank);

  /*!
   * Given an initial GraspableObject containing representations from different sources (database fits, clusters, etc),
   * populates a list of Representations each labeled with their type and only storing one representation of the
   * real object (for example, only one DatabaseModelPoseWithScore or only one PointCloud)
   */
  void populateRepresentationsList(std::vector<ObjectRepresentation> &representations,
                           std::string arm_name,
                           const object_manipulation_msgs::GraspableObject &request_object,
                           const bool enable_cluster);

  /*!
   * Creates an object representation based on the cluster, complete with a grasp retriever and 
   * probability computer.
   */
  ObjectRepresentation getObjectRepresentationFromCluster(const object_manipulation_msgs::GraspableObject &request_object,
                                                        std::string arm_name);


  /*!
   * Creates an object representation based on a recognition result, complete with a grasp retriever and 
   * probability computer.
   */
  ObjectRepresentation getObjectRepresentationFromDatabaseObject(
                                const household_objects_database_msgs::DatabaseModelPose &model_with_score,
                                std::string arm_name);

  void printGrasps(const std::vector<GraspWithMetadata> &grasps);

  void appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
                                 std::vector<GraspWithMetadata> &output_list,
                                 const object_manipulation_msgs::GraspableObject &graspable_object);

  void pruneGraspList(std::vector<GraspWithMetadata> &grasps, const double threshold);

  void clusterGrasps(std::vector<GraspWithMetadata> &input_list, std::vector<GraspWithMetadata> &cluster_rep_list);

  void printRepresentations(const std::vector<ObjectRepresentation> &representations);

public:
  /*!
   * Creates a new ProbabilisticDatabasePlanner with a connection to the database
   */
  ProbabilisticGraspPlanner(ObjectsDatabasePtr database) :
    database_(database),
    nh_("~"),
    debug_preprune_grasp_marker_publisher_("debug_preprune_grasp_list","",5.0),
    debug_precluster_grasp_marker_publisher_("debug_precluster_grasp_list","",5.0),
    debug_postprune_grasp_marker_publisher_("debug_postprune_grasp_list","",5.0)
  {
  }

  /*!
   * Given a graspable object describes the seen point cloud, plans which grasp is most likely to
   * successfully manipulate the underlying object.
   * At the end of the computation, the grasps vector stores a list of grasps sorted by probability of successful
   * manipulation.
   * Optionally will publish the grasp markers for visualization.
   */
  void plan(const std::string &arm_name, object_manipulation_msgs::GraspableObject &graspable_object,
            std::vector<object_manipulation_msgs::Grasp> &grasps, bool visualize_results,
            bool prune_grasps);
  /*!
   * Sets the grasp marker publishers that can optionally be used by this Planner. If the publishers are null,
   * the planner won't try to publish anything.
   */
  void setMarkerPublisher(object_manipulator::GraspMarkerPublisher* publisher,
                          object_manipulator::GraspMarkerPublisher* debug_publisher,
                          object_manipulator::GraspMarkerPublisher* rank_publisher)
  {
    grasp_marker_publisher_ = publisher;
    debug_grasp_marker_publisher_ = debug_publisher;
    rank_grasp_marker_publisher_ = rank_publisher;
  }
};
typedef boost::shared_ptr<ProbabilisticGraspPlanner> ProbabilisticGraspPlannerPtr;

} //namespace

#endif /* PROBABILISTIC_GRASP_PLANNER_H_ */
