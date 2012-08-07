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

#include "probabilistic_grasp_planner/forward_decls.h"

#ifndef GRASP_DENSITY_ESTIMATOR_H_
#define GRASP_DENSITY_ESTIMATOR_H_

namespace probabilistic_grasp_planner {

/*!
 * Given a list of grasps and a class which can be called to compute the probability of a grasp whose energy value is
 * known, creates a density estimator which can estimate the probability density for an arbitrary grasp, using the
 * known data points.
 */
class GraspRegressionEvaluator
{
private:
  //! A local copy of the grasps used to construct this density estimator. TODO: do we need a local copy?
  std::vector<GraspWithMetadata> grasps_;
  boost::shared_ptr<GraspSuccessProbabilityComputer> simple_computer_;

  double position_bandwidth_;
  double orientation_bandwidth_;
public:
  GraspRegressionEvaluator(const std::vector<GraspWithMetadata> &grasps,
                        boost::shared_ptr<GraspSuccessProbabilityComputer> &simple_computer,
                        double position_bandwidth, double orientation_bandwidth) :
                          grasps_(grasps),
                          simple_computer_(simple_computer),
                          position_bandwidth_(position_bandwidth),
                          orientation_bandwidth_(orientation_bandwidth) {}
  double estimateProbability(const GraspWithMetadata &grasp) const;
};

} // end namespace

#endif /* GRASP_DENSITY_ESTIMATOR_H_ */
