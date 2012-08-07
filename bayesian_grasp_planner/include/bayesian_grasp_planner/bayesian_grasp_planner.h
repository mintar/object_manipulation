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

#ifndef BAYESIAN_GRASP_PLANNER_H
#define BAYESIAN_GRASP_PLANNER_H

#include <boost/shared_ptr.hpp>
#include <object_manipulator/tools/grasp_marker_publisher.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/Grasp.h>
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"
#include <household_objects_database/objects_database.h>

namespace bayesian_grasp_planner
{

  //class GraspWM;
class ObjectDetector;
class GraspEvaluatorProb;

class BayesianGraspPlanner
{
private:

  ros::NodeHandle nh_;

  household_objects_database::ObjectsDatabasePtr database_;

  //! GMP for grasps before pruning
  object_manipulator::GraspMarkerPublisher debug_preprune_grasp_marker_publisher_;

  //! GMP for grasps after pruning but before clustering
  object_manipulator::GraspMarkerPublisher debug_postprune_grasp_marker_publisher_;

  //! GMP which shows the grasps returned by the planner, colored by probability
  object_manipulator::GraspMarkerPublisher grasp_marker_publisher_;

  /*!
   * Publishes the grasps to rviz with specific namespaces for grasps belonging to each different model, colored with
   * green being the best grasp and red being the worst.
   */
  void visualizeGrasps(const std::vector<GraspWM> &grasps, 
                        object_manipulator::GraspMarkerPublisher *grasp_marker_pub, bool color_by_rank);


  void createMutuallyExclusiveObjectRepresentations(const object_manipulation_msgs::GraspableObject &original_object,
                                                  std::vector<object_manipulation_msgs::GraspableObject> &me_objects );

  void createDatabaseObjectDetectors(const std::vector<object_manipulation_msgs::GraspableObject> &objects,
                                     std::vector< boost::shared_ptr<ObjectDetector> >&detectors);

  void appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
								  std::vector<GraspWM> &output_list,
								  const std::string frame_id);

  void clusterGrasps(std::vector<GraspWM> &input_list, 
					 std::vector<GraspWM> &cluster_rep_list);

  void pruneGraspList(std::vector<GraspWM> &grasps, const double threshold);

public:
  BayesianGraspPlanner(household_objects_database::ObjectsDatabasePtr database) : 
    nh_("~"),database_(database),
	debug_preprune_grasp_marker_publisher_("debug_preprune_grasp_list","",5.0),
	debug_postprune_grasp_marker_publisher_("debug_postprune_grasp_list","",5.0),
	grasp_marker_publisher_("grasp_marker_publisher","",5.0)
	{} 

  void bayesianInference(std::vector<GraspWM> &grasps,
                         const std::vector<object_manipulation_msgs::GraspableObject> &objects,
                         const std::vector< boost::shared_ptr<ObjectDetector> > &object_detectors,
                         const std::vector< boost::shared_ptr<GraspEvaluatorProb> > &prob_evaluators);

  void plan(const std::string &arm_name,
            const object_manipulation_msgs::GraspableObject &graspable_object,
            std::vector<object_manipulation_msgs::Grasp> &final_grasp_list);

};
typedef boost::shared_ptr<BayesianGraspPlanner> BayesianGraspPlannerPtr;

} //namespace

#endif
