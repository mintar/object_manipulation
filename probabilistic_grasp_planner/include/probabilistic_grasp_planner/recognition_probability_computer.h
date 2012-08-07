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

#ifndef RECOGNITION_PROBABILITY_COMPUTER
#define RECOGNITION_PROBABILITY_COMPUTER

#include <map>
#include <vector>

#include <object_manipulation_msgs/GraspableObject.h>
#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"

namespace probabilistic_grasp_planner {

//! Converts a raw recognition result into a list of probabilities that each object was correctly detected
class RecognitionProbabilityComputer
{
public:
  /*!
   * Given a list of GraspableObjects (database models, point clouds, etc.) which describe the real object, computes
   * the probability for each object that the hypothesis "this 'object' is the best description of
   * the real object" is true.
   */
  virtual void computeRepresentationProbabilities(std::vector<ObjectRepresentation> &objects) = 0;
  
};
typedef boost::shared_ptr<RecognitionProbabilityComputer> RecognitionProbabilityComputerPtr;

//! Returns 1.0 probability for the top recognition result, and 0.0 for all others
class TopHitProbabilityComputer : public RecognitionProbabilityComputer
{
private:
  //! Caches the id of the top hit
  int top_hit_model_id_;

public:
  void computeRepresentationProbabilities(std::vector<ObjectRepresentation> &objects);

};

//! Computes the probability of each model by comparing its raw fit score to the score of the best model in the list
class CompositeProbabilityComputer : public RecognitionProbabilityComputer
{
protected:
  //! Raw fit results worse than this threshold get 0.0 probability
  double recognition_threshold_;

  //! Whether we should give cluster planner any weight
  bool db_only_;
  //! Underlying algorithm to convert a recognition score (in meters, representing average distance between
  //! a correspondence pair between the model and the cloud) to a [0,1] probability.
  virtual double getProbabilityForRecognitionScore(const double &score);

public:
  CompositeProbabilityComputer(bool db_only) :
    db_only_(db_only)
  {

  }

  virtual void computeRepresentationProbabilities(std::vector<ObjectRepresentation> &objects);
};

class InverseCurveRecognitionProbabilityComputer : public CompositeProbabilityComputer
{
private:
  //! The value of the curvature parameter for this model
  double curvature_;

protected:
  /*! Underlying algorithm to convert a recognition score (in meters, representing average distance between
   *a correspondence pair between the model and the cloud) to a [0,1] probability.
   */
  virtual double getProbabilityForRecognitionScore(const double &score);
public:
  InverseCurveRecognitionProbabilityComputer(double recognition_threshold, double curvature, bool db_only) :
    CompositeProbabilityComputer(db_only), curvature_(curvature)
  {

  }
};

/*!
 * Computer which assigns static weights to:
 * - The first detected DB model
 * - The cluster planner
 * - All of the rest of the DB models
 *
 * CASES:
 * -If there is only the cluster, the cluster gets 1.0
 * -If there is the cluster and one DB model, the DB model gets 0.667 and the cluster gets 0.333
 * -If there is only a single DB model, the DB model gets 100%
 * -If there are only DB models, the first gets 0.667 and the rest get 0.333 divided evenly
 * -If there are multiple DB models and the cluster, the first DB model gets 0.5, the cluster gets 0.25, and the rest
 *  get 0.25 divided evenly
 *
 */
class LearnedProbabilityComputer : public RecognitionProbabilityComputer
{
private:
  enum Cases {CLUSTER_ONLY,CLUSTER_DB,DB_ONLY,DB_MULTIPLE,CLUSTER_DB_MULTIPLE};
public:
  LearnedProbabilityComputer()
  {

  }

  virtual void computeRepresentationProbabilities(std::vector<ObjectRepresentation> &objects);
};

}

 //namespace

#endif
