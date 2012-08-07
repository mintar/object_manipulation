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

#include <tf/transform_datatypes.h>

//#define PROF_ENABLED
//#include <profiling/profiling.h>

#include <Eigen/Core>
// For inverse
#include <Eigen/LU>

#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"
#include "probabilistic_grasp_planner/distribution_evaluator.h"

using namespace Eigen;

namespace probabilistic_grasp_planner {

/*!
 * Evaluates a 3D multivariate gaussian distribution with mean at g* and no covariance between dimensions
 */
double NormalDistributionEvaluator::evaluate_position(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp)
{

  Vector3d mean(gstar.tool_point_pose_.getOrigin().m_floats);
  Vector3d x(grasp.tool_point_pose_.getOrigin().m_floats);
  Vector3d delta(x-mean);
  Matrix3d covariance = Matrix3d::Zero();
  covariance(0,0) = covariance(1,1) = covariance(2,2) = pow(position_sigma_,2);

  double ex = exp(-0.5*(delta.transpose() * covariance.inverse() * delta)(0,0));
  //double coeff = 1.0/(pow(M_2_PI, 1.5)*pow(covariance.determinant(),0.5)); gets normalized out anyway
  return ex;
}

/*!
 * Evaluates the orientation using a Dimroth-Watson distribution to correctly cover quaternions
 */
double NormalDistributionEvaluator::evaluate_orientation(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp)
{
  double cos_angle = gstar.tool_point_pose_.getRotation().dot(grasp.tool_point_pose_.getRotation());
  double ex = exp(orientation_concentration_ * pow(cos_angle,2));
  double normalization = exp(orientation_concentration_);
  return ex / normalization;
}

double NormalDistributionEvaluator::evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp)
{
  double value =evaluate_position(gstar, grasp)*evaluate_orientation(gstar, grasp);
  normalization_term_ += value;
  return value;
}

double PolarNormalDistributionEvaluator::evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp)
{
  double m_r = 0.0;
  double position_stddev = 0.01; //1cm stddev
  double m_theta = 0.0;
  double orientation_stddev = 0.174; // 10 degrees stddev

  /*
   * Compute "probability" (actually likelihood or probability density) of getting this grasp given that gstar was
   * commanded.
   *
   * The this value is the product of the likelihood for the positioning error and the likelihood for the
   * orientation error.
   */

  double cartesian_distance = 0;
  double rotation_distance = 0;
  gstar.getDistance(grasp,cartesian_distance, rotation_distance);

  double p_position = exp(-pow((cartesian_distance-m_r),2)/(2*pow(position_stddev,2)));

  double p_orientation = exp(-pow((rotation_distance-m_theta),2)/(2*pow(orientation_stddev,2)));

  double pose_probability = p_position*p_orientation;

  normalization_term_ += pose_probability;
  return pose_probability;
}

} // namespace
