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

#ifndef PROBABILITY_DISTRIBUTION_H
#define PROBABILITY_DISTRIBUTION_H

namespace bayesian_grasp_planner {

class ProbabilityDistribution
{
public:
  virtual double evaluate(double value) const = 0;
};

class IdentityProbabilityDistribution : public ProbabilityDistribution
{
public:
  virtual double evaluate(double value) const {return value;}
};
 

//! Gaussian distribution (if for an unshifted/unflipped value, use min, max, and/or flip)
class GaussianProbabilityDistribution : public ProbabilityDistribution
{
protected:
  double mean_;    //mean of the shifted distribution
  double std_dev_; //std of the shifted distribution 
  double min_;     //raw value min for shifting
  double max_;     //raw value max for shifting
  bool   flip_;    //whether the raw value should be flipped so that higher is better
public:
  GaussianProbabilityDistribution(double mean, double std_dev, double min, double max, bool flip) : 
                      mean_(mean),std_dev_(std_dev),min_(min),max_(max),flip_(flip){}

  virtual double evaluate(double value) const 
  {
	//flip and shift the raw value
	double clipped_value = value;
	if (value > max_+1e-6)
	{
	  clipped_value = max_;
	  //ROS_WARN("clipping value %.3f to max of %.3f", value, max_);
	}
	if (value < min_-1e-6) 
	{
	  clipped_value = min_;
	  //ROS_WARN("clipping value %.3f to min of %.3f", value, min_);
	}
	double shifted_value;
	if (flip_)
	{
	  shifted_value = (max_ - clipped_value) / (max_ - min_);
	  //printf("flipping, max_=%.3f, min_=%.3f ", max_, min_);
	}
	else
	{
	  shifted_value = (clipped_value - min_) / (max_ - min_);
	  //printf("not flipping, max_=%.3f, min_=%.3f ", max_, min_);
	}
	//printf("mean: %.3f, std_dev: %.3f, value: %.3f, shifted_value: %.3f, prob: %.3f\n", mean_, std_dev_, value, shifted_value, 1/sqrt(2*3.14159*pow(std_dev_, 2)) * exp(-.5*pow(shifted_value - mean_, 2)/pow(std_dev_, 2)));
	return 1/sqrt(2*3.14159*pow(std_dev_, 2)) * exp(-.5*pow(shifted_value - mean_, 2)/pow(std_dev_, 2));
  }
};


//! Specialized distribution for the cluster planner--Gaussian except for 0 values (treated as a separate mode)
class BimodalGaussianProbabilityDistribution : public ProbabilityDistribution
{
protected:
  double mean_;      //mean of the shifted distribution (for all but zero values)
  double std_dev_;   //std of the shifted distribution (for all but zero values)
  double zero_prob_; //probability of getting a zero value
public:
  BimodalGaussianProbabilityDistribution(double mean, double std_dev, double zero_prob) : 
        mean_(mean),std_dev_(std_dev),zero_prob_(zero_prob){}

  virtual double evaluate(double value) const 
  {
	if (value == 0) return zero_prob_;
	return (1-zero_prob_)*(1/sqrt(2*3.14159*pow(std_dev_, 2)) * exp(-.5*pow(value - mean_, 2)/pow(std_dev_, 2)));
  }
};


}

#endif
