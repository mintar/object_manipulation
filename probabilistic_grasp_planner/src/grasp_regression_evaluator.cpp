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

#include "probabilistic_grasp_planner/distribution_evaluator.h"
#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"
#include "probabilistic_grasp_planner/grasp_success_probability_computer.h"
#include "probabilistic_grasp_planner/grasp_regression_evaluator.h"

namespace probabilistic_grasp_planner {

double GraspRegressionEvaluator::estimateProbability(const GraspWithMetadata &grasp) const
{
  NormalDistributionEvaluator evaluator(position_bandwidth_, orientation_bandwidth_);
  evaluator.reset_normalization_term();

  double probability = 0.0;

  const GraspWithMetadata* closest_grasp = NULL;
  double closest_dist = 0.0;

  double cartesian_dist, rotation_dist;

  BOOST_FOREACH(const GraspWithMetadata &grasp_for_object, grasps_)
  {
    // Find closest grasp
    if (closest_grasp==NULL)
    {
      closest_grasp = &grasp_for_object;

      grasp.getDistance(grasp_for_object, cartesian_dist, rotation_dist);

      closest_dist = sqrt(pow(position_bandwidth_*cartesian_dist,2) +
                          pow(orientation_bandwidth_*rotation_dist,2));
    }

    grasp.getDistance(grasp_for_object, cartesian_dist, rotation_dist);

    double this_dist = sqrt(pow(position_bandwidth_*cartesian_dist,2) +
                            pow(orientation_bandwidth_*rotation_dist,2));

    if (this_dist < closest_dist)
    {
      closest_grasp = &grasp_for_object;
      closest_dist = this_dist;
    }

    //Add the weighted contribution from grasp_for_object to grasp's probability
    double p_dens = evaluator.evaluate(grasp_for_object, grasp);
    double grasp_probability = simple_computer_->getProbability(grasp_for_object);
    probability += p_dens*grasp_probability;
  }

  // If this is false, then all of the probabilities are zero so we should return that directly
  if (evaluator.get_normalization_term() != 0.0)
  {
    probability /= evaluator.get_normalization_term();

    double p_dens = evaluator.evaluate(*closest_grasp, grasp);
    double grasp_probability = simple_computer_->getProbability(*closest_grasp);
    double probability_from_closest_grasp = p_dens*grasp_probability;

    return std::min(probability, probability_from_closest_grasp);
  }
  else {
    return 0.0;
  }
}

} // end namespace
