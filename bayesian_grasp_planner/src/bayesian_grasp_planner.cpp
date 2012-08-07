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

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "bayesian_grasp_planner/bayesian_grasp_planner.h"
#include "bayesian_grasp_planner/object_detector.h"
#include "bayesian_grasp_planner/grasp_evaluator.h"
#include "bayesian_grasp_planner/grasp_generator.h"
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"

namespace bayesian_grasp_planner {

void BayesianGraspPlanner::bayesianInference(std::vector<GraspWM> &grasps,
                                          const std::vector<object_manipulation_msgs::GraspableObject> &objects,
                                          const std::vector< boost::shared_ptr<ObjectDetector> > &object_detectors,
                                          const std::vector< boost::shared_ptr<GraspEvaluatorProb> > &prob_evaluators)
{

  // compute the conditional probabilities for all grasps at once for each grasp evaluator
  ROS_INFO("computing grasp conditional probabilities");
  BOOST_FOREACH(const boost::shared_ptr<GraspEvaluatorProb> &prob_eval, prob_evaluators)
  {
    std::vector<double> grasp_success_probs;
    std::vector<double> grasp_failure_probs;
    
    // initialize the success_cond_probs and failure_cond_probs for all grasps
    BOOST_FOREACH(GraspWM &grasp, grasps)
    {
      grasp.success_cond_probs.resize(objects.size());
      grasp.failure_cond_probs.resize(objects.size());
    }
    
    // if the evaluations are linked to a particular object rep, compute the values for each object rep
    if(prob_eval->is_object_dependent())
    {
      for(size_t object_ind = 0; object_ind < objects.size(); object_ind++)
      {
        // ignore the nd-object case
        if(objects[object_ind].potential_models.empty()) continue;
        
        // evaluate all grasps at once
        prob_eval->getProbabilitiesForGraspList(grasps, objects[object_ind], grasp_success_probs, grasp_failure_probs);
        
        // ROS_INFO("evaluting grasps on object %d", (int)object_ind);
        // printf("grasp_success_prob: ");
        // pplist(grasp_success_probs);
        // printf("grasp_failure_prob: ");
        // pplist(grasp_failure_probs);
        
        // put the results in the appropriate list for this object rep
        for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
        {
          grasps[grasp_ind].success_cond_probs[object_ind].push_back(grasp_success_probs[grasp_ind]);
          grasps[grasp_ind].failure_cond_probs[object_ind].push_back(grasp_failure_probs[grasp_ind]);
        }
      }
    }
    
    // evaluations aren't linked to any object rep, compute once and put them in the lists for all object reps
    else
    {
      // evaluate all grasps at once
      prob_eval->getProbabilitiesForGraspList(grasps, objects[objects.size()-1], grasp_success_probs, grasp_failure_probs);
      
      // printf("evaluating grasps with cluster planner");
      // printf("grasp_success_prob: ");
      // pplist(grasp_success_probs);
      // printf("grasp_failure_prob: ");
      // pplist(grasp_failure_probs);
      
      // put the results in the lists for all object reps
      for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
      {
        for (size_t object_ind = 0; object_ind < objects.size(); object_ind++)
        {
          grasps[grasp_ind].success_cond_probs[object_ind].push_back(grasp_success_probs[grasp_ind]);
          grasps[grasp_ind].failure_cond_probs[object_ind].push_back(grasp_failure_probs[grasp_ind]);
        }
      }
    }
  }

  {
    for (size_t object_ind = 0; object_ind < objects.size(); object_ind++)
    {
      // Object prior 
      double object_prior;
      if (objects[object_ind].potential_models.size() > 0) 
      {        
        object_prior = 0.5 / (objects.size()-1);
        printf("Database object %d:\n", objects[object_ind].potential_models[0].model_id);
      }
      else 
      {
        printf("Non-database object:\n");
        object_prior = 0.5;
      }
      
      // Contributions from object detection results
      double detection_prob = 1.0;  //prod over d_j(P(d_j|o_i))
      BOOST_FOREACH(const boost::shared_ptr<ObjectDetector> &detector, object_detectors)
      {
        double obj_det_prob = detector->getProbabilityForDetection(objects[object_ind]);
        //printf("  obj_det_prob for object %d:%0.6f\n", (int)object_ind, obj_det_prob);
        detection_prob *= obj_det_prob;
      }
      printf("  total detection probability: %f\n", detection_prob);
    }
  }

 
  // perform inference to predict P(s|D,E) (prob of grasp success given all detection and grasp eval results)
  ROS_INFO("performing inference");
  for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
  {
    double grasp_overall_success_prob = 0.0;
    double grasp_overall_failure_prob = 0.0;
    for (size_t object_ind = 0; object_ind < objects.size(); object_ind++)
    {
      // Object prior 
      double object_prior;
      if (objects[object_ind].potential_models.size() > 0) object_prior = 0.5 / (objects.size()-1);
      else object_prior = 0.5;
      
      // Contributions from object detection results
      double detection_prob = 1.0;  //prod over d_j(P(d_j|o_i))
      //printf("obj_det_probs: ");
      BOOST_FOREACH(const boost::shared_ptr<ObjectDetector> &detector, object_detectors)
      {
        double obj_det_prob = detector->getProbabilityForDetection(objects[object_ind]);
        //printf("obj_det_prob for object %d:%0.6f\n", (int)object_ind, obj_det_prob);
        //printf("%.6f ", obj_det_prob);
        detection_prob *= obj_det_prob;
      }
      //printf("\n");
      
      // Grasp prior 
      double success_prior = 0.5;  //P(s|o_i)
      double failure_prior = 0.5;  //P(f|o_i)
      
      // Contributions from grasp evaluation results (computed above)
      double grasp_success_prob = 1.0;  //product over e_k of P(e_k|s,o_i)/P(e_k|o_i)
      double grasp_failure_prob = 1.0;  //product over e_k of P(e_k|f,o_i)/P(e_k|o_i)
	  /*
	  if(grasp_ind == 60)
	  {
		printf("grasp_ind: %d\n", (int)grasp_ind);
		printf("success cond probs for object %d: ", (int)object_ind);
		pplist(grasps[grasp_ind].success_cond_probs[object_ind]);
		printf("failure cond probs for object %d: ", (int)object_ind);
		pplist(grasps[grasp_ind].failure_cond_probs[object_ind]);
		}*/
      for (size_t eval_ind = 0; eval_ind < grasps[grasp_ind].success_cond_probs[object_ind].size(); eval_ind ++)
      {
        //P(e_k|o_i,s)
        double succ_prob = grasps[grasp_ind].success_cond_probs[object_ind][eval_ind];
        //P(e_k|o_i,f)
        double fail_prob = grasps[grasp_ind].failure_cond_probs[object_ind][eval_ind];
        //P(e_k|o_i)
        double eval_prob = succ_prob * success_prior + fail_prob * failure_prior; 
	
        grasp_success_prob *= succ_prob / eval_prob;
        grasp_failure_prob *= fail_prob / eval_prob;
	
      }
      
      // Multiply together to get the overall contribution for this object representation
      // P(s|D,E) = alpha * sum over o_i( P(o_i) * prod over d_j(P(d_j|o_i)) * P(s|o_i) * prod over e_k(P(e_k|s,o_i)/P(e_k|o_i)) )
      grasp_overall_success_prob += object_prior * detection_prob * success_prior * grasp_success_prob;
      grasp_overall_failure_prob += object_prior * detection_prob * failure_prior * grasp_failure_prob;
      // ROS_INFO("object %d: detection_prob: %.6f", (int)object_ind, detection_prob);
      // ROS_INFO("grasp_success_prob: %.3f, grasp_overall_success_prob contrib: %.6f", grasp_success_prob, object_prior * detection_prob * success_prior * grasp_success_prob);
      // ROS_INFO("grasp_failure_prob: %.3f, grasp_overall_failure_prob contrib: %.6f", grasp_failure_prob, object_prior * detection_prob * failure_prior * grasp_failure_prob);
      // printf("press enter to continue");
      // char blah[100];
      // std::cin.getline(blah, 100);
    }
    
    // Normalize so that P(s|D,E) + P(f|D,E) = 1 (normalization_factor is alpha)
    if (grasp_overall_success_prob + grasp_overall_failure_prob == 0)
    {
      ROS_WARN("sum of success and failure conditional probs was 0!");
      grasps[grasp_ind].grasp_.success_probability = 0.;
      continue;
    }
    double normalization_factor = 1.0 / (grasp_overall_success_prob + grasp_overall_failure_prob);
    grasps[grasp_ind].grasp_.success_probability = normalization_factor * grasp_overall_success_prob;
	//printf("%.3f ", grasps[grasp_ind].grasp_.success_probability);
    //printf("\n");
  }
}


void BayesianGraspPlanner::plan(const std::string &arm_name,
                                const object_manipulation_msgs::GraspableObject &graspable_object,
                                std::vector<object_manipulation_msgs::Grasp> &final_grasp_list)
{
  //create the probability distributions for the cluster raw results
  boost::shared_ptr<ProbabilityDistribution> cluster_success_distribution( 
										  new BimodalGaussianProbabilityDistribution(.611, .285, .806) );
  boost::shared_ptr<ProbabilityDistribution> cluster_failure_distribution(                                 
										  new BimodalGaussianProbabilityDistribution(.563, .272, .935) );

  //create the probability distributions for graspit raw results
  boost::shared_ptr<ProbabilityDistribution> graspit_success_distribution( 
						 new GaussianProbabilityDistribution(.825, .125, 0.0, 1.0, false) );
  boost::shared_ptr<ProbabilityDistribution> graspit_failure_distribution( 
						 new GaussianProbabilityDistribution(.627, .376, 0.0, 1.0, false) );

  //create mutually exclusive individual objects (nd-object at the end)
  std::vector<object_manipulation_msgs::GraspableObject> objects;
  createMutuallyExclusiveObjectRepresentations(graspable_object, objects);

  //create object detectors
  std::vector< boost::shared_ptr<ObjectDetector> > object_detectors;
  createDatabaseObjectDetectors(objects, object_detectors);

  //create the overall list of grasp generators
  std::vector< boost::shared_ptr<GraspGenerator> > grasp_generators;

  //create the overall list of grasp evaluators
  std::vector< boost::shared_ptr<GraspEvaluatorProb> > grasp_evaluators;

  //generators and evaluators for graspit-based stuff
  //*** add options for expensive/service-based GraspIt! evaluators and perturbation evaluators
  {
    //create the graspit-based regression evaluator
    boost::shared_ptr<MultiplexEvaluator> graspit_multiplexer_regression_eval( new MultiplexEvaluator );

    //for each database object
    BOOST_FOREACH(const household_objects_database_msgs::DatabaseModelPose &model, graspable_object.potential_models)
    {
      //create a grasp generator with all grasps
      boost::shared_ptr<GraspGenerator> grasp_gen( new GraspGeneratorDatabaseRetriever(database_, model, arm_name, false) );
      grasp_gen->generateGrasps();
      
      //create a regression evaluator based on it
      boost::shared_ptr<RawGraspEvaluator> regression_eval( new RawGraspEvaluatorWithRegression(grasp_gen, true) );
      
      //put that in our overall multiplexer graspit-regression-based evaluator
      graspit_multiplexer_regression_eval->addEvaluator(regression_eval, model.model_id);

      //create a grasp generator with only cluster rep grasps
      boost::shared_ptr<GraspGenerator> grasp_gen_reps( new GraspGeneratorDatabaseRetriever(database_, model, arm_name, true) );

      //put it in our overall list of grasp generators
      grasp_generators.push_back( grasp_gen_reps );      
    }

    //create a probabilistic estimator based on the graspit raw evaluator
    boost::shared_ptr<GraspEvaluatorProb> prob_eval( new GraspEvaluatorProb( graspit_success_distribution,
                                                                             graspit_failure_distribution,
                                                                             graspit_multiplexer_regression_eval ) );

    //add it to our overall list of evaluators
    grasp_evaluators.push_back(prob_eval);
  }

  //generators and evaluators for cluster-based stuff
  
  {
    //create a grasp generator based on the cluster
    boost::shared_ptr<GraspGenerator> grasp_gen( new GraspGeneratorServiceCaller(nh_, "/plan_point_cluster_grasp", 
										 graspable_object, arm_name));

    //put it in our overall list of grasp generators
    grasp_generators.push_back(grasp_gen);

    int cluster_computer_type;
    nh_.param("cluster_computer_type",cluster_computer_type, 1);
    ROS_INFO("cluster_computer_type = %d", cluster_computer_type);
    
    boost::shared_ptr<RawGraspEvaluator> cluster_eval;
    switch (cluster_computer_type)
    {
    case 1: //create a service-based evaluator based on it
      {
	std::string test_cluster_grasp_srv;
	nh_.param("test_cluster_grasp_srv", test_cluster_grasp_srv, std::string("/evaluate_point_cluster_grasps"));  
        cluster_eval.reset( new RawGraspEvaluatorServiceCaller(nh_, test_cluster_grasp_srv, false) );
        break;
      }
    case 2: //create a regression evaluator based on it
      {
        cluster_eval.reset( new RawGraspEvaluatorWithRegression(grasp_gen, false) );
        break;
      }
    }
    
    //create a probabilistic estimator based on it
    boost::shared_ptr<GraspEvaluatorProb> prob_eval( new GraspEvaluatorProb( cluster_success_distribution,
                                                                             cluster_failure_distribution,
                                                                             cluster_eval ) );
    //add it to our overall list of evaluators
    grasp_evaluators.push_back(prob_eval);
  }
  
  ROS_INFO("%d grasp generators created", (int)grasp_generators.size());

  //list of grasps to evaluate
  std::vector<GraspWM> grasps;
  ROS_INFO("evaluating grasps");

  //if we're given a list of grasps to test, use those
  if (final_grasp_list.size() > 0)
  {
	//append metadata to the grasps
	appendMetadataToTestGrasps(final_grasp_list, grasps, graspable_object.reference_frame_id);             
  }

  //otherwise, compute the list of grasps (as GraspWMs)
  else
  {
	BOOST_FOREACH(boost::shared_ptr<GraspGenerator> &gen, grasp_generators)
	{
	  ROS_INFO("getting grasps from grasp generator");
	  gen->generateGrasps();
	  gen->getGrasps(grasps);
	}
  }

  ROS_INFO("number of grasps to evaluate: %d", (int)grasps.size());

  //perform the inference
  bayesianInference(grasps, objects, object_detectors, grasp_evaluators);  
  
  /*
  printf("before sorting:");
  BOOST_FOREACH(GraspWM graspwm, grasps)
  {
	printf("%.3f ", graspwm.getGrasp().success_probability);
  }
  printf("\n\n");
  */
  //sort the grasps
  std::sort(grasps.rbegin(), grasps.rend());

  visualizeGrasps(grasps, &debug_preprune_grasp_marker_publisher_, false);
  /*
  printf("after sorting, before clustering:");
  BOOST_FOREACH(GraspWM graspwm, grasps)
  {
	printf("%.3f ", graspwm.getGrasp().success_probability);
  }
  printf("\n\n");
  */
  //if we're not testing a fixed list of grasps, prune grasps with too low probability of success
  if (final_grasp_list.size() == 0)
  {
    double grasp_probability_threshold;
    nh_.param("initial_probability_threshold", grasp_probability_threshold, 0.01);  
    pruneGraspList(grasps, grasp_probability_threshold);
  }

  visualizeGrasps(grasps, &debug_postprune_grasp_marker_publisher_, false);

  //cluster the grasps
  ROS_INFO("clustering");
  std::vector<GraspWM> clustered_grasps;
  clusterGrasps(grasps, clustered_grasps);

  //take at most quantity_threshold grasps
  int grasps_max;
  nh_.param("quantity_threshold", grasps_max, 100);
  size_t grasps_max_size = grasps_max;
  if (clustered_grasps.size() > grasps_max_size)
  {
	ROS_INFO("Taking top %zd grasps after clustering", grasps_max_size);
	clustered_grasps.resize(grasps_max_size);
  }    

  //copy the final grasps over
  final_grasp_list.clear();
  final_grasp_list.reserve(clustered_grasps.size());
  BOOST_FOREACH(GraspWM graspwm, clustered_grasps)
  {
	final_grasp_list.push_back(graspwm.getGrasp());
	//printf("%.3f ", graspwm.getGrasp().success_probability);
  }
  visualizeGrasps(clustered_grasps, &grasp_marker_publisher_, false);

  ROS_INFO("done planning");
}

void BayesianGraspPlanner::createMutuallyExclusiveObjectRepresentations(
                                               const object_manipulation_msgs::GraspableObject &original_object,
                                               std::vector<object_manipulation_msgs::GraspableObject> &me_objects )
{
  //one for each database model/pose detected
  for (size_t i = 0; i < original_object.potential_models.size(); i++)
  {
	object_manipulation_msgs::GraspableObject new_object;
	new_object.potential_models.push_back(original_object.potential_models[i]);
	new_object.reference_frame_id = original_object.reference_frame_id;
	me_objects.push_back(new_object);
  }

  //one for the non-database object case (containing just the cluster)
  object_manipulation_msgs::GraspableObject new_object;
  new_object.cluster = original_object.cluster;
  new_object.reference_frame_id = original_object.reference_frame_id;
  me_objects.push_back(new_object);

  ROS_INFO("%d object representations created", (int)me_objects.size());
}

void BayesianGraspPlanner::createDatabaseObjectDetectors( 
                                    const std::vector<object_manipulation_msgs::GraspableObject> &objects,		
                                    std::vector< boost::shared_ptr<ObjectDetector> >&detectors)
{
  //get params for the available object detectors from a yaml file
  //and initialize correct and incorrect distributions for each database object detector (put into dictionary)
  //***
  boost::shared_ptr<ProbabilityDistribution> correct_distribution(
                               new GaussianProbabilityDistribution(.531, .125, 0., .005, true));  
  boost::shared_ptr<ProbabilityDistribution> incorrect_distribution(
                               new GaussianProbabilityDistribution(.292, .154, 0., .005, true));

  //one detector for each database object 
  for (size_t i = 0; i < objects.size(); i++)
  {
    if (objects[i].potential_models.size() != 0)
    {
      //use the distributions for the object detector that generated this detection
      //***
      boost::shared_ptr<ObjectDetector> detector ( new DatabaseObjectDetector(objects[i], correct_distribution, 
                                                                              incorrect_distribution) );
      detectors.push_back(detector);
    }	
  }		
  ROS_INFO("%d object detectors created", (int)detectors.size());
}

void BayesianGraspPlanner::appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
                                                      std::vector<GraspWM> &output_list, const std::string frame_id)
{

  output_list.reserve(output_list.size()+input_list.size());

  ROS_INFO("Got %zd grasps to test",input_list.size());
  BOOST_FOREACH(object_manipulation_msgs::Grasp &grasp, input_list)
  {
    GraspWM grasp_wm;
    grasp_wm.grasp_ = grasp;
    grasp_wm.grasp_.success_probability = 0.0;
    grasp_wm.model_id_ = -1;
    grasp_wm.grasp_id_ = -1;

    grasp_wm.object_pose_.setIdentity();
    grasp_wm.object_pose_.frame_id_ = frame_id;
    grasp_wm.object_pose_.stamp_ = ros::Time(0);

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 13cm from wrist
    tf::Pose grasp_in_base_frame;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_in_base_frame);
    double wrist_to_tool_point_offset_ = 0.13;
    tf::Pose grasp_to_tool_point(tf::Matrix3x3::getIdentity(),tf::Vector3(wrist_to_tool_point_offset_,0,0));
    grasp_wm.tool_point_pose_ = tf::Stamped<tf::Pose>(
        grasp_in_base_frame * grasp_to_tool_point, grasp_wm.object_pose_.stamp_, frame_id);

    output_list.push_back(grasp_wm);
  }
  ROS_DEBUG("Created %zd grasps", output_list.size());
}


void BayesianGraspPlanner::clusterGrasps(std::vector<GraspWM> &input_list, 
										 std::vector<GraspWM> &cluster_rep_list)
{
  ROS_INFO("Clustering %zd grasps",input_list.size());

  double clustering_cart_distance_threshold = 0.01;
  double clustering_angle_distance_threshold = 0.266;//30 degrees

  int clusters = 0;
  while (!input_list.empty())
  {
    GraspWM *repGrasp = &input_list.front();
    repGrasp->grasp_.cluster_rep = true;
    cluster_rep_list.push_back(*repGrasp);
    input_list.erase(input_list.begin());
    ++clusters;

    int cloud=0;
    std::vector<GraspWM>::iterator it=input_list.begin();
    while (it != input_list.end())
    {
      double cart_distance, angle_distance;
      cluster_rep_list.back().getDistance(*it,cart_distance,angle_distance);
      if (cart_distance < clustering_cart_distance_threshold && 
          angle_distance < clustering_angle_distance_threshold)
      {
        ++cloud;
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

void BayesianGraspPlanner::pruneGraspList(std::vector<GraspWM> &grasps, const double threshold)
{
  std::vector<GraspWM>::iterator it=grasps.begin();
  int erased_count = 0;
  while( it != grasps.end())
  {
    if (it->grasp_.success_probability < threshold)
    {
      erased_count++;
      ROS_DEBUG("Erasing grasp with probability %g",it->grasp_.success_probability);
      it=grasps.erase(it);
    }
    else{
      it++;
    }
  }

  ROS_INFO("Removed %d grasps below threshold",erased_count);
}



void BayesianGraspPlanner::visualizeGrasps(const std::vector<GraspWM> &grasps,
        object_manipulator::GraspMarkerPublisher *grasp_publisher, bool color_by_rank)
{
  grasp_publisher->clearAllMarkers();
  size_t sz = grasps.size();
  if (sz == 0)
  {
    return;
  }

  for (size_t i=0; i < sz; i++) {
    const GraspWM *grasp = &grasps[i];
    std::string ns_append = boost::lexical_cast<std::string>(grasp->model_id_);
    grasp_publisher->setNamespaceSuffix(ns_append);

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
      colorval = 1 - grasp->grasp_.success_probability;
    }
    grasp_publisher->colorGraspMarker(id, colorval, 1-colorval, 0.0);
  }
}


} //namespace
