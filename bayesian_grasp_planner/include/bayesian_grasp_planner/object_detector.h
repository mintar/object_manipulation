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

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <object_manipulation_msgs/GraspableObject.h>

#include "bayesian_grasp_planner/probability_distribution.h"

namespace bayesian_grasp_planner {

class ObjectDetector
{
public:
  virtual double getProbabilityForDetection(const object_manipulation_msgs::GraspableObject &go) const = 0;
}; 

class DatabaseObjectDetector : public ObjectDetector
{
public:
  DatabaseObjectDetector(const object_manipulation_msgs::GraspableObject &object, 
                         boost::shared_ptr<ProbabilityDistribution> correct_distribution,
                         boost::shared_ptr<ProbabilityDistribution> incorrect_distribution) :
    correct_distribution_(correct_distribution),
    incorrect_distribution_(incorrect_distribution)
  {
    ROS_ASSERT(object.potential_models.size() == 1);
    my_object_ = object;
  }

  double getProbabilityForDetection(const object_manipulation_msgs::GraspableObject &go) const
  {
    if ( go.potential_models.empty() || 
         go.potential_models[0].model_id != my_object_.potential_models[0].model_id )
    {
      return incorrect_distribution_->evaluate(my_object_.potential_models[0].confidence);
    }
    return correct_distribution_->evaluate(my_object_.potential_models[0].confidence);
  }

private:
  object_manipulation_msgs::GraspableObject my_object_;
  boost::shared_ptr<ProbabilityDistribution> correct_distribution_;
  boost::shared_ptr<ProbabilityDistribution> incorrect_distribution_;
};

}

#endif
