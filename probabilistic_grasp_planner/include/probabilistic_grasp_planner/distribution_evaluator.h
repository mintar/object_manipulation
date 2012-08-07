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

#ifndef DISTRIBUTION_COMPUTER_H_
#define DISTRIBUTION_COMPUTER_H_

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

namespace probabilistic_grasp_planner {
//! Forward declarations
class GraspWithMetadata;

class DistributionEvaluator
{
protected:
  double normalization_term_;

protected:
  DistributionEvaluator() :
    normalization_term_(0.0)
  {
  }

public:
  virtual double evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp) = 0;

  virtual void reset_normalization_term()
  {
    normalization_term_ = 0.0;
  }

  virtual double get_normalization_term() const
  {
    return normalization_term_;
  }

  virtual ~DistributionEvaluator() {};
};

typedef shared_ptr<DistributionEvaluator> DistributionEvaluatorPtr;

class UniformDistributionEvaluator : public DistributionEvaluator
{
public:
  double evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp)
  {
    normalization_term_ += 1.0;
    return 1.0;
  }
};

/*!
 * Given a "mean" grasp g* and a grasp point to evaluate, evaluates the point's value in a 6D distribution
 * consisting of a 3D Gaussian distribution over position and a Dimroth-Watson distribution over orientation.
 */
class NormalDistributionEvaluator : public DistributionEvaluator
{
private:

  double position_sigma_, orientation_concentration_;

  double evaluate_position(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp);
  double evaluate_orientation(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp);
public:
  double evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp);
  NormalDistributionEvaluator(double position_sigma, double orientation_concentration) :
    position_sigma_(position_sigma), orientation_concentration_(orientation_concentration)
  {
  }
};

/*!
 * Given a grasp g* which acts as the mean of the distribution, this evaluates a 2D Gaussian PDF over
 * positioning error (sqrt(x^2+y^2+z^2)) and orientation error (quaternion.getAngle()).
 */
class PolarNormalDistributionEvaluator : public DistributionEvaluator
{
public:
  double evaluate(const GraspWithMetadata &gstar,const GraspWithMetadata &grasp);
};

} // namespace

#endif /* DISTRIBUTION_COMPUTER_H_ */
