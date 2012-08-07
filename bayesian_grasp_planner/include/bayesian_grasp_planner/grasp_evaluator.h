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

#ifndef GRASP_EVALUATOR_H
#define GRASP_EVALUATOR_H

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/Grasp.h>

#include "bayesian_grasp_planner/probability_distribution.h"
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"

namespace bayesian_grasp_planner {

class GraspGenerator;
//class GraspWM;

class RawGraspEvaluator
{
protected:
  bool object_dependent_;  //whether this evaluator cares about the 'correct' object representation

public:
  //! Evalute a single grasp (raw grasp metric)
  virtual double evaluate(const GraspWM &grasp, 
                          const object_manipulation_msgs::GraspableObject &object) const = 0;

  //! Evaluate an entire list of grasps (overwrite if there is a batch operation for an inherited class)
  virtual void evaluate_list(std::vector<GraspWM> &grasps, 
							 const object_manipulation_msgs::GraspableObject &object,
							 std::vector<double> &values)
  {
	values.clear();
	values.resize(grasps.size());
	for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
	{
	  values[grasp_ind] = evaluate(grasps[grasp_ind], object);
	}
  }

  //! Returns whether this evaluator cares about the 'correct' object identity
  bool is_object_dependent()
  {
	return object_dependent_;
  }

};

class MultiplexEvaluator : public RawGraspEvaluator
{
private: 
  std::map<int, boost::shared_ptr<RawGraspEvaluator> > eval_map_;
public:
  MultiplexEvaluator()
  {
	object_dependent_ = true;
  }

  void addEvaluator(boost::shared_ptr<RawGraspEvaluator> eval, int id)
  {
    eval_map_.insert( std::pair<int, boost::shared_ptr<RawGraspEvaluator> >(id, eval) );
  }

  double evaluate(const GraspWM &grasp, 
                  const object_manipulation_msgs::GraspableObject &object) const
  {
	int id = 0;
    if (object.potential_models.empty()) id = -1;  //non-database object case
    else id = object.potential_models[0].model_id;
    std::map< int, boost::shared_ptr<RawGraspEvaluator> > ::const_iterator it = eval_map_.find(id);
    //todo: throw exception
    if (it == eval_map_.end()) return 0;
    return it->second->evaluate(grasp, object);
  }

  void evaluate_list(std::vector<GraspWM> &grasps, 
					 const object_manipulation_msgs::GraspableObject &object,
					 std::vector<double> &values)
  {
	int id = 0;
    if (object.potential_models.empty()) id = -1;  //non-database object case
    else id = object.potential_models[0].model_id;
    std::map< int, boost::shared_ptr<RawGraspEvaluator> > ::const_iterator it = eval_map_.find(id);
    //todo: throw exception
    if (it == eval_map_.end())
	{
	  values.clear();
	  values.resize(grasps.size(), 0);
	  ROS_ERROR("object %d not found in grasp evaluator map!", id);
	  return;
	}
	it->second->evaluate_list(grasps, object, values);
  }
};

class RawGraspEvaluatorWithRegression : public RawGraspEvaluator
{
private:
  boost::shared_ptr<GraspGenerator> generator_;

public:
  RawGraspEvaluatorWithRegression (boost::shared_ptr<GraspGenerator> generator, bool object_dependent) : 
    generator_(generator)
  {
	object_dependent_ = object_dependent;
  }

  double evaluate(const GraspWM &grasp, 
                  const object_manipulation_msgs::GraspableObject &object) const;
};

 class RawGraspEvaluatorServiceCaller : public RawGraspEvaluator
{
private:
  // calling a service is non-const, hence the mutable
  mutable ros::ServiceClient service_;
  std::string service_name_;
public:
  RawGraspEvaluatorServiceCaller(ros::NodeHandle &nh, std::string service_name, bool object_dependent);

  double evaluate(const GraspWM &grasp, 
                  const object_manipulation_msgs::GraspableObject &object) const;

  void evaluate_list(std::vector<GraspWM> &grasps, 
					 const object_manipulation_msgs::GraspableObject &object,
					 std::vector<double> &values);
};



class GraspEvaluatorProb 
{
private:
  boost::shared_ptr<ProbabilityDistribution> success_distribution_;
  boost::shared_ptr<ProbabilityDistribution> failure_distribution_;
  boost::shared_ptr<RawGraspEvaluator> evaluator_;

public:
  GraspEvaluatorProb( boost::shared_ptr<ProbabilityDistribution> success_distribution,
                      boost::shared_ptr<ProbabilityDistribution> failure_distribution,
                      boost::shared_ptr<RawGraspEvaluator> evaluator ) :
    success_distribution_(success_distribution),
    failure_distribution_(failure_distribution),
    evaluator_(evaluator) {}

  //! Returns whether this evaluator cares about the 'correct' object identity
  bool is_object_dependent()
  {
	return evaluator_->is_object_dependent();
  }

  //! Get conditional probabilities for a single grasp
  void getProbabilitiesForGrasp(const GraspWM &grasp, 
								  const object_manipulation_msgs::GraspableObject &object, double &success_prob, double &failure_prob) const
  {
    double raw_score = evaluator_->evaluate(grasp, object);
    success_prob = success_distribution_->evaluate(raw_score);
    failure_prob = failure_distribution_->evaluate(raw_score);
  }

  //! Get conditional probabilities for an entire list of grasps
  void getProbabilitiesForGraspList(std::vector<GraspWM> &grasps, 
									  const object_manipulation_msgs::GraspableObject &object, 
									  std::vector<double> &success_probs, std::vector<double> &failure_probs) const
  {
	success_probs.clear();
	failure_probs.clear();
	success_probs.resize(grasps.size());
	failure_probs.resize(grasps.size());
	std::vector<double> raw_scores;
	evaluator_->evaluate_list(grasps, object, raw_scores);
	for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
	{
	  success_probs[grasp_ind] = success_distribution_->evaluate(raw_scores[grasp_ind]);
	  failure_probs[grasp_ind] = failure_distribution_->evaluate(raw_scores[grasp_ind]);
	}
  }
};



}


#endif
