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

#ifndef _GRASP_SUCCESS_PROBABILITY_COMPUTER_
#define _GRASP_SUCCESS_PROBABILITY_COMPUTER_

#include <vector>
#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

#include <object_manipulation_msgs/GraspableObject.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>

#include <object_manipulation_msgs/GraspPlanning.h>

#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"

#include "probabilistic_grasp_planner/grasp_regression_evaluator.h"

#include "probabilistic_grasp_planner/forward_decls.h"
#include "probabilistic_grasp_planner/grasp_retriever.h"

namespace probabilistic_grasp_planner {

//! Computes the probability of success for a grasp on a given model
class GraspSuccessProbabilityComputer
{
public:
  //virtual double operator()(const GraspWithMetadata &grasp) = 0;
  virtual double getProbability(const GraspWithMetadata &grasp) = 0;

  //! Calls the grasp planning service for all grasps at the same time
  virtual void getProbabilities(const std::vector<GraspWithMetadata> &grasps, 
                                std::vector<double> &probabilities)
  {
	probabilities.resize(grasps.size());
	for (size_t grasp_idx=0; grasp_idx < grasps.size(); grasp_idx++)
	{
	  probabilities[grasp_idx] = getProbability(grasps[grasp_idx]);
	}
  }
};
typedef boost::shared_ptr<GraspSuccessProbabilityComputer> GraspSuccessProbabilityComputerPtr;

class GSPCWithEstimation : public GraspSuccessProbabilityComputer
{
protected:
  GraspRegressionEvaluator estimator_;
public:
  GSPCWithEstimation(const std::vector<GraspWithMetadata> &known_grasps,
                     boost::shared_ptr<GraspSuccessProbabilityComputer> &simple_computer,
                     double position_bandwidth, double orientation_concentration) :
                       estimator_(known_grasps, simple_computer, position_bandwidth, orientation_concentration)
  {

  }

  double getProbability(const GraspWithMetadata &grasp)
  {
    return estimator_.estimateProbability(grasp);
  }

};

//! Simply calls the grasp planning service with a given name
class GSPCServiceCaller : public GraspSuccessProbabilityComputer
{
private:
  //! Remembers the name of the service
  std::string service_name_;
  
  //! The service client used to answer calls
  ServiceWrapper<object_manipulation_msgs::GraspPlanning> client_;

  //! The object that success is computed on
  object_manipulation_msgs::GraspableObject target_;

  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;
public:
  //! Initializes service client and waits for it to become available
  GSPCServiceCaller(std::string service_name, const object_manipulation_msgs::GraspableObject &target);

  //! Simply calls the grasp planning service
  virtual double getProbability(const GraspWithMetadata &grasp);

  //! Calls the grasp planning service for all grasps at the same time
  virtual void getProbabilities(const std::vector<GraspWithMetadata> &grasps, 
                                std::vector<double> &probabilities);
};

//! Returns a simple probability of success based on the grasp's energy score
/*! Will return non-zero probability only if the grasp was originally computed on the same
  model that it is being queried for. In this case, it will still return 0 if the energy
  is above a given threshold, or linearly increasing if it is below that.
*/
class SimpleGraspSuccessProbabilityComputer : public GraspSuccessProbabilityComputer
{
private:
  //! The model id that the grasp's model_id will be compared to
  int model_id_;
  //! The threshold above which grasps have 0 probability of success
  double energy_threshold_;
public:

  SimpleGraspSuccessProbabilityComputer(double model_id, double energy_threshold) :
    model_id_(model_id),
    energy_threshold_(energy_threshold) {}

  virtual double getProbability(const GraspWithMetadata &grasp);
};
typedef boost::shared_ptr<SimpleGraspSuccessProbabilityComputer> SimpleGraspSuccessProbabilityComputerPtr;

class PositionRobustGraspSuccessProbabilityComputer : public GraspSuccessProbabilityComputer
{
protected:
  PerturbationGraspRetriever perturbation_grasp_retriever_;
  GraspSuccessProbabilityComputerPtr probability_computer_;
  shared_ptr<DistributionEvaluator> normal_evaluator_;

public:
  PositionRobustGraspSuccessProbabilityComputer(ObjectsDatabasePtr database,
                                                const household_objects_database_msgs::DatabaseModelPose &model,
                                           GraspSuccessProbabilityComputerPtr success_probability_computer,
                                           shared_ptr<DistributionEvaluator> normal_evaluator,
                                           const std::string &arm_name) :
    perturbation_grasp_retriever_(database,model, arm_name),
    probability_computer_(success_probability_computer),
    normal_evaluator_(normal_evaluator)
  {
    ROS_DEBUG("Created new position robust GSPC");
  }

  virtual double getProbability(const GraspWithMetadata &grasp);

};
typedef boost::shared_ptr<PositionRobustGraspSuccessProbabilityComputer> OverallGraspSuccessProbabilityComputerPtr;

class SimplePointClusterGSPC : public GraspSuccessProbabilityComputer
{
public:
  SimplePointClusterGSPC() {}
  virtual double getProbability(const GraspWithMetadata &grasp)
  {
    //! TODO: do something with cloud_
    if (grasp.model_id_ == -1)
    {
      return grasp.grasp_.success_probability;
    }
    return 0.0;
  }
};


} //namespace

#endif
