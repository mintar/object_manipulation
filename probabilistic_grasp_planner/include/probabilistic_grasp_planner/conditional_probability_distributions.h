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

// Author(s): Kaijen Hsiao

#ifndef CONDITIONAL_PROB_DIST_H_
#define CONDITIONAL_PROB_DIST_H_

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

namespace probabilistic_grasp_planner {

//! Base class for functions that compute success/failure conditional probabilities based on some quality value
class ConditionalProbabilityDistributions
{
public:
  virtual void evaluateConditionalProbs(double quality, double &success_cond_prob, double &failure_cond_prob) = 0;
};
typedef boost::shared_ptr<ConditionalProbabilityDistributions> ConditionalProbabilityDistributionsPtr;


//! Approximates both success and failure distributions as simple Gaussians with means and stds
class GaussianCPD : public ConditionalProbabilityDistributions
{
protected:
  double success_mean_;
  double success_std_;
  double failure_mean_;
  double failure_std_;
public:
  GaussianCPD(double success_mean, double success_std, double failure_mean, double failure_std) :
    success_mean_(success_mean),success_std_(success_std),failure_mean_(failure_mean),failure_std_(failure_std) {}

  virtual void evaluateConditionalProbs(double quality, double &success_cond_prob, double &failure_cond_prob) = 0; 
};		 
typedef boost::shared_ptr<GaussianCPD> GaussianCPDPtr;
  
} //namespace

#endif
  

