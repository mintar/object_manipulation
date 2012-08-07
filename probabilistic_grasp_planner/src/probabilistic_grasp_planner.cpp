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

#include <algorithm>
#include <fstream>

#include <boost/foreach.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

//#define PROF_ENABLED
//#include <profiling/profiling.h>

#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"
#include "probabilistic_grasp_planner/grasp_retriever.h"
#include "probabilistic_grasp_planner/grasp_success_probability_computer.h"
#include "probabilistic_grasp_planner/distribution_evaluator.h"

#include "probabilistic_grasp_planner/probabilistic_grasp_planner.h"

// Grasp Success Probability Computers
//PROF_EXTERN(PRGSPC_PROF);
//PROF_EXTERN(SIMPLE_GSPC_PROF);

// Algorithm subroutines
//PROF_EXTERN(NORMAL_PROF);

// Grasp Retrievers
//PROF_EXTERN(CLUSTER_PLANNER_CREATE_PROF);
//PROF_EXTERN(CLUSTER_REP_CREATE_PROF);

// Planner functions
//PROF_DECLARE(PLAN_PROF);
//PROF_DECLARE(CREATE_GR_PROF);
//PROF_DECLARE(CREATE_PC_PROF);
//PROF_DECLARE(FIRST_STAGE_PROF);
//PROF_DECLARE(SECOND_STAGE_PROF);

namespace probabilistic_grasp_planner {

void ProbabilisticGraspPlanner::plan(const std::string &arm_name,
                                     object_manipulation_msgs::GraspableObject &graspable_object,
                                     std::vector<object_manipulation_msgs::Grasp> &final_grasp_list,
                                     bool visualize_results, bool prune_grasps)
{ 
  //PROF_RESET_ALL;
  //PROF_START_TIMER(PLAN_PROF);

  bool enable_cluster;
  nh_.param("use_cluster",enable_cluster,true);

  //TopHitProbabilityComputer representation_probability_computer;
  bool db_only = !enable_cluster;
  CompositeProbabilityComputer representation_probability_computer(db_only);
  //InverseCurveRecognitionProbabilityComputer representation_probability_computer(0.01,2.198,db_only);
  //LearnedProbabilityComputer representation_probability_computer;

  std::vector<ObjectRepresentation> representations;
  ROS_INFO("Creating object representations");
  populateRepresentationsList(representations, arm_name, graspable_object, enable_cluster);
  ROS_INFO("Computing representation probabilities");
  representation_probability_computer.computeRepresentationProbabilities(representations);

  printRepresentations(representations);

  //PROF_PRINT(CREATE_GR_PROF);
  //PROF_PRINT(CLUSTER_PLANNER_CREATE_PROF);
  //PROF_PRINT(CLUSTER_REP_CREATE_PROF);
  //PROF_PRINT(CREATE_PC_PROF);

  // generate a list of grasps g_stars
  //! TODO: switch this to a std::list, we don't need random access
  std::vector<GraspWithMetadata> grasps;

  // If we were given grasps, we should analyze those instead of getting new grasps
  if (final_grasp_list.size() > 0)
  {
    ROS_INFO("Received grasps to test, using those rather than building own list");
    //Convert Grasp to GraspWithData
    appendMetadataToTestGrasps(final_grasp_list, grasps, graspable_object);
  }
  else 
  {
    BOOST_FOREACH(ObjectRepresentation &representation, representations)
    {
      if (representation.probability > 0.001)
      {
        representation.grasp_retriever->getGrasps(grasps);
      }
    }
  }

  // for each g_star evaluate probability of success
  double recognition_probability(0.0);
  double success_probability(0.0);
  //PROF_START_TIMER(FIRST_STAGE_PROF);
  ROS_INFO("Total grasps %zd",grasps.size());
  for (size_t representation_idx=0; representation_idx < representations.size(); representation_idx++)
  {
    recognition_probability = representations[representation_idx].probability;
	std::vector<double> success_probabilities;
	ROS_INFO("Analyzing representation %zd", representation_idx);
	representations[representation_idx].grasp_success_computer->getProbabilities(grasps, success_probabilities);

    for (size_t grasp_idx=0; grasp_idx < grasps.size(); grasp_idx++)
    {
	  success_probability = success_probabilities[grasp_idx];
      grasps[grasp_idx].debug_probabilities.push_back
        (std::make_pair(representation_idx, std::make_pair(recognition_probability, success_probability)));
      grasps[grasp_idx].success_probability += recognition_probability*success_probability;
    }
  }

  //PROF_STOP_TIMER(FIRST_STAGE_PROF);
  //PROF_PRINT(FIRST_STAGE_PROF);

  // copy probability into grasp message. We can't store in there originally since 
  // cluster planner stores estimate there
  BOOST_FOREACH(GraspWithMetadata &grasp, grasps)
  {
    grasp.grasp_.success_probability = grasp.success_probability;
    grasp.success_probability = 0.0;
  }

  //PROF_PRINT(NORMAL_PROF);
  //PROF_PRINT(PRGSPC_PROF);
  //PROF_PRINT(SIMPLE_GSPC_PROF);
  // rank grasps g_star by probability and return sorted list
  std::sort(grasps.rbegin(), grasps.rend());

  ROS_INFO("First round planning returned %zd grasps.", grasps.size());
  if (!grasps.empty()) ROS_INFO("Top probability: %f", grasps[0].grasp_.success_probability);
  visualizeGrasps(grasps, &debug_preprune_grasp_marker_publisher_, false);

  if (prune_grasps)
  {
    //prune grasps below quality threshold
    double grasp_probability_threshold;
    nh_.param("initial_probability_threshold", grasp_probability_threshold, 0.3);  
    pruneGraspList(grasps, grasp_probability_threshold);
    ROS_INFO("  Pruned to %zd grasps above threshold of %f", grasps.size(), grasp_probability_threshold);
    visualizeGrasps(grasps, &debug_precluster_grasp_marker_publisher_, false);

    //cluster remaining grasps
    std::vector<GraspWithMetadata> clustered_grasps;
    clusterGrasps(grasps, clustered_grasps);
    grasps = clustered_grasps;
    ROS_INFO("  Clustered to %zd grasps", grasps.size());

    int grasps_max;
    nh_.param("quantity_threshold", grasps_max, 100);
    size_t grasps_max_size = grasps_max;
    if (grasps.size() > grasps_max_size)
    {
      ROS_INFO("Taking top %zd grasps after clustering", grasps_max_size);
      grasps.resize(grasps_max_size);
    }    
    visualizeGrasps(grasps, &debug_postprune_grasp_marker_publisher_, false);
  }

  bool second_pass;
  nh_.param("two_pass",second_pass,false);
  if (second_pass)
  {
    //PROF_START_TIMER(SECOND_STAGE_PROF);
    ROS_INFO("Reevaluating %zd grasps using expensive computations", grasps.size());

    int counter = 0;
    BOOST_FOREACH(ObjectRepresentation &representation, representations)
    {
      ROS_DEBUG("  Evaluating representation %d",counter);
      recognition_probability = representation.probability;
      //! note that grasps should now be very small
      BOOST_FOREACH(GraspWithMetadata &grasp, grasps)
      {
        //recompute the probability using the expensive method.
        success_probability = representation.precise_grasp_success_computer->getProbability(grasp);
        grasp.success_probability += recognition_probability*success_probability;
      }
      counter++;
    }

    BOOST_FOREACH(GraspWithMetadata &grasp, grasps)
    {
      grasp.grasp_.success_probability = grasp.success_probability;
    }

    std::sort(grasps.rbegin(), grasps.rend());
    ROS_INFO("Second pass done. Top probability: %f", grasps[0].success_probability);
    double final_grasp_probability_threshold;
    nh_.param("final_probability_threshold",final_grasp_probability_threshold,0.5);
    pruneGraspList(grasps, final_grasp_probability_threshold);
    ROS_INFO("Pruned to %zd grasps above threshold %f", grasps.size(), final_grasp_probability_threshold);
    //PROF_STOP_TIMER(SECOND_STAGE_PROF);
    //PROF_PRINT(SECOND_STAGE_PROF);
  }

  if (visualize_results)
  {
    if (debug_grasp_marker_publisher_ != NULL) visualizeGrasps(grasps, debug_grasp_marker_publisher_, false);
    if (rank_grasp_marker_publisher_ != NULL) visualizeGrasps(grasps, rank_grasp_marker_publisher_, true);
  }

  visualizeGrasps(grasps, grasp_marker_publisher_, false);

  //Need to clear it in case this is a test of the planner and there already were grasps here
  final_grasp_list.clear();
  final_grasp_list.reserve(grasps.size());
  BOOST_FOREACH(GraspWithMetadata &g, grasps)
  {
    final_grasp_list.push_back(g.grasp_);
  }
  //PROF_STOP_TIMER(PLAN_PROF);
  //PROF_PRINT(PLAN_PROF);
}

void ProbabilisticGraspPlanner::pruneGraspList(std::vector<GraspWithMetadata> &grasps, const double threshold)
{
  std::vector<GraspWithMetadata>::iterator it=grasps.begin();
  int erased_count = 0;
  while( it != grasps.end())
  {
    if (it->grasp_.success_probability < threshold)
    {
      erased_count++;
      //ROS_DEBUG("Erasing grasp with probability %g",it->grasp_.success_probability);
      it=grasps.erase(it);
    }
    else{
      it++;
    }
  }

  ROS_INFO("Removed %d grasps below threshold",erased_count);
}

void ProbabilisticGraspPlanner::populateRepresentationsList(std::vector<ObjectRepresentation> &representations,
                                                            std::string arm_name,
                                                const object_manipulation_msgs::GraspableObject &request_object,
                                                const bool enable_cluster)
{
  // First create an object representation for each of the potential fit models.
  BOOST_FOREACH(const household_objects_database_msgs::DatabaseModelPose &model_with_score,
                request_object.potential_models)
  {
    representations.push_back( getObjectRepresentationFromDatabaseObject(model_with_score, arm_name) );
  }

  // Then add an object for the cluster
  if (enable_cluster && request_object.cluster.points.size() > 0)
  {
    representations.push_back(getObjectRepresentationFromCluster(request_object, arm_name));
  }
}

ObjectRepresentation ProbabilisticGraspPlanner::getObjectRepresentationFromCluster(
                                                       const object_manipulation_msgs::GraspableObject &request_object,
                                                       std::string arm_name)
{
  //create the representation
  ObjectRepresentation representation;

  //create the grasp retriever for it  
  const std::string cluster_planner_service_name = "/plan_point_cluster_grasp";
  representation.grasp_retriever.reset(new ClusterPlannerGraspRetriever(nh_, cluster_planner_service_name, 
                                                                        request_object, arm_name));

  //create a copy of the graspable object with a single instance of the database model
  object_manipulation_msgs::GraspableObject request_object_single_instance;
  request_object_single_instance.cluster = request_object.cluster;
  representation.object = request_object_single_instance;

  //create a regular grasp probability computer
  int cluster_computer_type;
  nh_.param("cluster_computer_type", cluster_computer_type, 1);
  std::string cluster_evaluation_service_name;
  nh_.param("test_cluster_grasp_srv", cluster_evaluation_service_name, std::string("/evaluate_point_cluster_grasps"));
  switch (cluster_computer_type)
  {
  case 0: // Simple computer
    {
      ROS_DEBUG("Creating Simple Cluster Computer");
      representation.grasp_success_computer.reset(new SimplePointClusterGSPC);
      break;
    }
  case 1: // Cluster planner computer
    {
      ROS_DEBUG("Creating service caller GSPC");
      representation.grasp_success_computer.reset(new GSPCServiceCaller(cluster_evaluation_service_name,
                                                                        request_object_single_instance));
      break;
    }
  case 2: // Regression computer
    {
      ROS_DEBUG("Creating Regression Cluster Computer");
      shared_ptr<GraspSuccessProbabilityComputer> simple_cluster_computer(new SimplePointClusterGSPC);
      std::vector<GraspWithMetadata> grasps;
      representation.grasp_retriever->getGrasps(grasps);
      double cluster_position_sigma;
      double cluster_orientation_concentration;
      nh_.param("cluster_regression_position_sigma", cluster_position_sigma, 0.01);
      nh_.param("cluster_regression_orientation_concentration", cluster_orientation_concentration, 3.0);      
      representation.grasp_success_computer.reset(new GSPCWithEstimation(grasps, simple_cluster_computer,
                                                                         cluster_position_sigma,
                                                                         cluster_orientation_concentration));
      ROS_DEBUG("Regression Params %g %g",cluster_position_sigma, cluster_orientation_concentration);
      break;
    }
  }
  //create a precise grasp probability computer
  representation.precise_grasp_success_computer.reset(new GSPCServiceCaller(cluster_evaluation_service_name,
                                                                            request_object_single_instance));
  //and return it
  return representation;
}

ObjectRepresentation ProbabilisticGraspPlanner::getObjectRepresentationFromDatabaseObject(
                                            const household_objects_database_msgs::DatabaseModelPose &model_with_score,
                                            std::string arm_name)
{
  //create the representation
  ObjectRepresentation representation;

  //create the grasp retriever for it
  shared_ptr<DatabaseGraspRetriever> retriever(new DatabaseGraspRetriever(database_, model_with_score,arm_name,true,true));
  ROS_INFO("Fetching grasps for model id %d", model_with_score.model_id);
  retriever->fetchFromDB();
  representation.grasp_retriever = retriever;

  double energy_threshold;
  nh_.param("db_max_energy_threshold",energy_threshold, 100.0);

  std::string graspit_computer_service_name, graspit_computer_robust_service_name;
  nh_.param<std::string>("graspit_computer_service_name", graspit_computer_service_name, 
                         "default_graspit_computer_service_name");
  nh_.param<std::string>("graspit_computer_robust_service_name", graspit_computer_robust_service_name, 
                         "default_graspit_computer_robust_service_name");

  //create a copy of the graspable object with a single instance of the database model
  object_manipulation_msgs::GraspableObject request_object_single_instance;
  request_object_single_instance.potential_models.push_back(model_with_score); 
  representation.object = request_object_single_instance;

  //create the regular grasp probability computer for it
  int db_computer_type;
  nh_.param("db_computer_type",db_computer_type,2);
  switch (db_computer_type)
  {
  case 0: //Simple computer
    {
      ROS_DEBUG("Creating Simple DB Computer");
      representation.grasp_success_computer.reset(new SimpleGraspSuccessProbabilityComputer(model_with_score.model_id, 
                                                                                            energy_threshold));
      break;
    }
  case 1: // GraspIt computer
    {
      ROS_DEBUG("Creating GraspIt DB Computer");
      representation.grasp_success_computer.reset( new GSPCServiceCaller(graspit_computer_service_name, 
                                                                         request_object_single_instance) );
      break;
    }
  case 2: // Regression computer
    {
	  //create a separate grasp retriever to get all the database grasps
	  shared_ptr<DatabaseGraspRetriever> all_grasps_retriever(new DatabaseGraspRetriever(database_, model_with_score,arm_name,true,false));
	  all_grasps_retriever->fetchFromDB();

      ROS_DEBUG("Creating Regression DB Computer");
      shared_ptr<GraspSuccessProbabilityComputer> simple_probability_computer(new
                SimpleGraspSuccessProbabilityComputer(model_with_score.model_id, energy_threshold));
      std::vector<GraspWithMetadata> grasps;
      all_grasps_retriever->getGrasps(grasps);
      double db_position_sigma, db_orientation_concentration;
      nh_.param("db_regression_position_sigma", db_position_sigma, 0.01);
      nh_.param("db_regression_orientation_concentration", db_orientation_concentration, 10.0);      
      representation.grasp_success_computer.reset(new GSPCWithEstimation(grasps, simple_probability_computer,
                                                                 db_position_sigma, db_orientation_concentration));
      ROS_DEBUG("Regression Params %g %g",db_position_sigma, db_orientation_concentration);
      break;
    }
  case 3: // Position robust computer
    {
      shared_ptr<NormalDistributionEvaluator> distribution_evaluator(new NormalDistributionEvaluator(0.01, 0.174));
      shared_ptr<GraspSuccessProbabilityComputer> simple_probability_computer
        (new SimpleGraspSuccessProbabilityComputer( model_with_score.model_id,  energy_threshold));
      representation.grasp_success_computer.reset
        (new PositionRobustGraspSuccessProbabilityComputer( database_, model_with_score, simple_probability_computer,
                                                            distribution_evaluator, arm_name));
      break;
    }
  case 4: // GraspIt robust computer
    {
      ROS_DEBUG("Creating Robust GraspIt DB Computer");
      representation.grasp_success_computer.reset( new GSPCServiceCaller(graspit_computer_robust_service_name, 
                                                                         request_object_single_instance) );
      break;
    }
  }
  //create the precise computer for it
  representation.precise_grasp_success_computer.reset(new GSPCServiceCaller(graspit_computer_robust_service_name, 
                                                                            request_object_single_instance));
  //and finally return it
  return representation;
}

void ProbabilisticGraspPlanner::visualizeGrasps(const std::vector<GraspWithMetadata> &grasps,
        object_manipulator::GraspMarkerPublisher *grasp_publisher, bool color_by_rank)
{
  grasp_publisher->clearAllMarkers();
  size_t sz = grasps.size();
  if (sz == 0)
  {
    return;
  }
  double best_score = grasps[0].grasp_.success_probability;
  for (size_t i=0; i < sz; i++) {
    const GraspWithMetadata *grasp = &grasps[i];
    std::string ns_append = boost::lexical_cast<std::string>(grasp->model_id_);
    grasp_publisher->setNamespaceSuffix(ns_append);

    //tf::Stamped<tf::Pose> model_pose_tf(grasp.object_pose_);

    tf::Pose grasp_pose_tf;
    tf::poseMsgToTF(grasp->grasp_.grasp_pose, grasp_pose_tf);

    tf::Stamped<tf::Pose> grasp_in_world(grasp_pose_tf, grasp->object_pose_.stamp_, grasp->object_pose_.frame_id_);

    geometry_msgs::PoseStamped grasp_in_world_msg;
    tf::poseStampedTFToMsg(grasp_in_world, grasp_in_world_msg);

    unsigned int id = grasp_publisher->addGraspMarker(grasp_in_world_msg);

    float colorval;
    if (color_by_rank)
    {
      colorval = i/(1.0*sz);
    }
    else {
      colorval = 1.0 - grasp->grasp_.success_probability / (1.0*best_score);
    }
    grasp_publisher->colorGraspMarker(id, colorval, 1-colorval, 0.0);
  }
}

void ProbabilisticGraspPlanner::printRepresentations(const std::vector<ObjectRepresentation> &representations)
{  
  ROS_INFO("Object representations:");
  BOOST_FOREACH(const ObjectRepresentation &representation, representations)
  {
    ROS_INFO("  Probability: %f", representation.probability);
    if (!representation.object.potential_models.empty())
    {
      ROS_INFO("    Database object with id %d", representation.object.potential_models[0].model_id);
    }
    else
    {
      ROS_INFO("    Cluster with %zu points", representation.object.cluster.points.size());
    }
  } 
}

void ProbabilisticGraspPlanner::printGrasps(const std::vector<GraspWithMetadata> &grasps)
{
  BOOST_FOREACH(const GraspWithMetadata &grasp, grasps)
  {
    ROS_DEBUG_STREAM("Probability: "<< grasp.grasp_.success_probability << " " << grasp.grasp_id_ << " Loc " <<
                    grasp.grasp_.grasp_pose.position.x << " " <<
                    grasp.grasp_.grasp_pose.position.y << " " <<
                    grasp.grasp_.grasp_pose.position.z << " " <<
                    grasp.grasp_.grasp_pose.orientation.x << " " <<
                    grasp.grasp_.grasp_pose.orientation.y << " " <<
                    grasp.grasp_.grasp_pose.orientation.z << " " <<
                    grasp.grasp_.grasp_pose.orientation.w);
    ROS_DEBUG_STREAM("Contributions:");
    for (size_t i=0; i < grasp.debug_probabilities.size(); ++i)
    {
      ROS_DEBUG("%d: %g %g",grasp.debug_probabilities[i].first, grasp.debug_probabilities[i].second.first,
               grasp.debug_probabilities[i].second.second);
    }
  }
}

void 
ProbabilisticGraspPlanner::appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
                                                      std::vector<GraspWithMetadata> &output_list,
                                                      const object_manipulation_msgs::GraspableObject &graspable_object)
{

  output_list.reserve(output_list.size()+input_list.size());

  ROS_INFO("Got %zd grasps to test",input_list.size());
  BOOST_FOREACH(object_manipulation_msgs::Grasp &grasp, input_list)
  {
    GraspWithMetadata grasp_with_metadata;
    grasp_with_metadata.grasp_ = grasp;
    //grasp_with_metadata.grasp_.success_probability = 0.0;
    grasp_with_metadata.model_id_ = -1;
    grasp_with_metadata.grasp_id_ = -1;

    grasp_with_metadata.object_pose_.setIdentity();
    grasp_with_metadata.object_pose_.frame_id_ = graspable_object.reference_frame_id;
    grasp_with_metadata.object_pose_.stamp_ = ros::Time(0);

    tf::Pose grasp_in_base_frame;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_in_base_frame);
    double wrist_to_tool_point_offset_ = 0.13;
    tf::Pose grasp_to_tool_point(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));
    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 13cm from wrist
    grasp_with_metadata.tool_point_pose_ = tf::Stamped<tf::Pose>(
        grasp_in_base_frame * grasp_to_tool_point, grasp_with_metadata.object_pose_.stamp_,
        grasp_with_metadata.object_pose_.frame_id_);

    output_list.push_back(grasp_with_metadata);
  }
  ROS_DEBUG("Created %zd grasps",output_list.size());

}

void ProbabilisticGraspPlanner::clusterGrasps(std::vector<GraspWithMetadata> &input_list, 
                                              std::vector<GraspWithMetadata> &cluster_rep_list)
{
  ROS_INFO("Clustering %zd grasps",input_list.size());

  double clustering_cart_distance_threshold = 0.01;
  double clustering_angle_distance_threshold = 0.266;//30 degrees

  int clusters = 0;
  while (!input_list.empty())
  {
    GraspWithMetadata *repGrasp = &input_list.front();
    repGrasp->grasp_.cluster_rep = true;
    cluster_rep_list.push_back(*repGrasp);
    input_list.erase(input_list.begin());
    ++clusters;

    int cloud=0;
    std::vector<GraspWithMetadata>::iterator it=input_list.begin();
    while (it != input_list.end())
    {
      double cart_distance, angle_distance;
      cluster_rep_list.back().getDistance(*it,cart_distance,angle_distance);
      if (cart_distance < clustering_cart_distance_threshold && 
          angle_distance < clustering_angle_distance_threshold)
      {
        ++cloud;
        /*!
         * TODO: this line adds non-cluster grasps to the result list so that we can see all grasps with cluster reps
         * highlighted. Remove this to actually filter out non-cluster rep grasps.
         */
        //cluster_rep_list.push_back(*it);
        it = input_list.erase(it);
      }
      else {
        ++it;
      }
    }
    ROS_DEBUG("Grouped cluster of size %d",cloud);
  }
  ROS_INFO("Selected %d cluster rep grasps",clusters);
}

} // namespace
