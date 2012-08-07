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

#include <object_manipulation_msgs/GraspPlanning.h>

#include "bayesian_grasp_planner/grasp_evaluator.h"
#include "bayesian_grasp_planner/grasp_generator.h"
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"

namespace bayesian_grasp_planner {

double RawGraspEvaluatorWithRegression::evaluate(const GraspWM &gi, 
                                                 const object_manipulation_msgs::GraspableObject &) const
{
  double value = 0;
  //ROS_INFO("starting regression evaluator");

  //for drop-off function
  double dropoff_pos_bandwidth = 0.01; //1 cm std
  double dropoff_rot_bandwidth = 0.435; //25 degrees std
  
  //for smoothing regression
  double smoothing_pos_bandwidth = dropoff_pos_bandwidth / 2.0;
  double smoothing_rot_bandwidth = dropoff_rot_bandwidth / 2.0;

  //get grasps for the generator
  const std::vector<GraspWM> &grasps = generator_->getGrasps();
  //ROS_INFO("inside regression evaluator, got %d grasps for regression", (int)grasps.size());

  //compute regression contributions from all grasps and find the highest drop-off function value
  double cartesian_dist, rotation_dist;
  double energy = 0.0;
  double normalization_term = 0.0;
  double max_dropoff_value = 0.0;
  double regression_weight = 0.0;
  double dropoff_value = 0.0;
  for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
  {
	grasps[grasp_ind].getDistance(gi, cartesian_dist, rotation_dist);

	if (cartesian_dist < dropoff_pos_bandwidth * 4 && rotation_dist < dropoff_rot_bandwidth * 4)
	{
	  regression_weight = exp(-pow(cartesian_dist,2)/(2*pow(smoothing_pos_bandwidth,2))) * 
	                           exp(-pow(rotation_dist,2)/(2*pow(smoothing_rot_bandwidth,2)));
	  dropoff_value =  exp(-pow(cartesian_dist,2)/(2*pow(dropoff_pos_bandwidth,2))) * 
                             exp(-pow(rotation_dist,2)/(2*pow(dropoff_rot_bandwidth,2)));
	  if (dropoff_value > max_dropoff_value) max_dropoff_value = dropoff_value; 
	  energy += regression_weight * grasps[grasp_ind].energy_function_score;
	  normalization_term += regression_weight;
	  //ROS_INFO("regression_weight: %0.3f", regression_weight);
	  //ROS_INFO("dropoff_value: %0.3f", dropoff_value);
	}
  }

  //no grasps were close enough
  if (normalization_term == 0) 
  {
    //ROS_INFO("no grasps close enough (%zd tested)!", grasps.size());
    return 0.0;
  }	
  //take the minimum of the highest dropoff-value and the regression value
  value = std::min(energy/normalization_term, max_dropoff_value);  
  //ROS_INFO("grasp LWLR energy: %0.3f, max_dropoff_value: %0.3f, final value: %0.3f", 
  //         energy/normalization_term, max_dropoff_value, value);
  return value;
}

RawGraspEvaluatorServiceCaller::RawGraspEvaluatorServiceCaller(ros::NodeHandle &nh, std::string service_name, 
                                                               bool object_dependent)
{
  object_dependent_ = object_dependent;
  service_ = register_service<object_manipulation_msgs::GraspPlanning>(nh, service_name);
}

double RawGraspEvaluatorServiceCaller::evaluate(const GraspWM &gi, 
                                                const object_manipulation_msgs::GraspableObject &go) const
{
  object_manipulation_msgs::GraspPlanning plan; 
  plan.request.target = go;
  plan.request.grasps_to_evaluate.push_back(gi.grasp_);
  if (!service_.call(plan))
  {
    ROS_ERROR("Grasp success probability computer, failed to call service at %s", service_name_.c_str());
    return 0.0;
  }
  ROS_ASSERT(plan.response.grasps.size() == 1);
  return plan.response.grasps[0].success_probability;  
}

void RawGraspEvaluatorServiceCaller::evaluate_list(std::vector<GraspWM> &grasps, 
                                                   const object_manipulation_msgs::GraspableObject &object,
                                                   std::vector<double> &values)
{
  values.clear();
  values.resize(grasps.size(), 0.0);
  object_manipulation_msgs::GraspPlanning plan; 
  plan.request.target = object;
  BOOST_FOREACH(const GraspWM &grasp, grasps)
  {
    plan.request.grasps_to_evaluate.push_back(grasp.grasp_);
  }
  if (!service_.call(plan))
  {
    ROS_ERROR("Grasp success probability computer, failed to call service at %s", service_name_.c_str());
    return;
  }
  ROS_ASSERT(plan.response.grasps.size() == grasps.size());
  for (size_t i=0; i<grasps.size(); i++)
  {
    values[i] = plan.response.grasps[i].success_probability;
  }
}

}
