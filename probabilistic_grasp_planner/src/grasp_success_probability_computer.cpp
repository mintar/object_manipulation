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

#include <boost/foreach.hpp>

//#define PROF_ENABLED
//#include <profiling/profiling.h>

#include "probabilistic_grasp_planner/distribution_evaluator.h"
#include "probabilistic_grasp_planner/grasp_retriever.h"
#include "probabilistic_grasp_planner/grasp_success_probability_computer.h"

//PROF_DECLARE(PRGSPC_PROF);
//PROF_DECLARE(SIMPLE_GSPC_PROF);

namespace probabilistic_grasp_planner {


double SimpleGraspSuccessProbabilityComputer::getProbability(const GraspWithMetadata &grasp)
{
  //PROF_TIMER_FUNC(SIMPLE_GSPC_PROF);
  // if grasp is not from object, return 0
  if (model_id_ != grasp.model_id_) return 0.0;
  // otherwise, return non-zero exponentially decaying probability
  double sigma_sq = 2025.0;
  double prob = 0.95 * std::exp(-0.5 * std::pow(grasp.energy_function_score_,2)/sigma_sq);
  return prob;
}

GSPCServiceCaller::GSPCServiceCaller(std::string service_name, 
                                     const object_manipulation_msgs::GraspableObject &target) :
  service_name_(service_name),
  client_(service_name),
  priv_nh_("~"),
  root_nh_("")
{
  target_ = target;
}

double GSPCServiceCaller::getProbability(const GraspWithMetadata &grasp)
{
  object_manipulation_msgs::GraspPlanning plan; 
  plan.request.target = target_;
  plan.request.grasps_to_evaluate.push_back(grasp.grasp_);
  if (!client_.client().call(plan))
  {
    ROS_ERROR("Grasp success probability computer, failed to call service at %s", service_name_.c_str());
    return 0.0;
  }
  ROS_ASSERT(plan.response.grasps.size() == 1);
  return plan.response.grasps[0].success_probability;  
}

void GSPCServiceCaller::getProbabilities(const std::vector<GraspWithMetadata> &grasps, 
                                         std::vector<double> &probabilities)
{
  probabilities.clear();
  probabilities.resize( grasps.size(), 0.0 );
  object_manipulation_msgs::GraspPlanning plan; 
  plan.request.target = target_;
  BOOST_FOREACH(const GraspWithMetadata &grasp, grasps) 
  {
    plan.request.grasps_to_evaluate.push_back(grasp.grasp_);
  }
  ROS_INFO("calling service to evaluate grasps");
  if (!client_.client().call(plan))
  {
    ROS_ERROR("Grasp success probability computer, failed to call service at %s", service_name_.c_str());
    return;
  }
  ROS_ASSERT(plan.response.grasps.size() == grasps.size());
  for (size_t i=0; i<plan.response.grasps.size(); i++)
  {
    probabilities[i] = plan.response.grasps[i].success_probability;
  }
}

double PositionRobustGraspSuccessProbabilityComputer::getProbability(const GraspWithMetadata &grasp)
{
  //PROF_TIMER_FUNC(PRGSPC_//PROF);
  std::vector<GraspWithMetadata> grasps;
  //! tell the retriever to only get perturbations for this grasp
  perturbation_grasp_retriever_.setGrasp(&grasp);
  //! get the perturbations
  perturbation_grasp_retriever_.getGrasps(grasps);

  double probability = 0.0;
  normal_evaluator_->reset_normalization_term();
  BOOST_FOREACH(GraspWithMetadata &g_n, grasps)
  {
    //Compute the probability of success of that perturbation
    double success_probability = probability_computer_->getProbability(g_n);
    //Compute the likelihood of getting to that perturbation given the originally commanded grasp
    double probability_of_grasp_execution = normal_evaluator_->evaluate(grasp, g_n);
    //Add the product of these two values to the resulting probability
    probability += success_probability*probability_of_grasp_execution;
  }
  //Normalize to turn likelihoods into probabilities
  probability /= normal_evaluator_->get_normalization_term();
  //Return the sum
  return probability;
}


}
