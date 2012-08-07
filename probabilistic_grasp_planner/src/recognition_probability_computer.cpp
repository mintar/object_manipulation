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

#include "probabilistic_grasp_planner/recognition_probability_computer.h"
#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"

namespace probabilistic_grasp_planner {

void TopHitProbabilityComputer::computeRepresentationProbabilities(std::vector<ObjectRepresentation> &representations)
{
  if (representations.empty())
  {
    ROS_ERROR("Cannot prepare probabilities list for empty representation list");
    return;
  }

  bool top_found = false;
  BOOST_FOREACH(ObjectRepresentation &representation, representations)
  {
    if (!top_found)
    {
      representation.probability = 1.0;
      top_found = true;
    } else 
    {
      representation.probability = 0.0;
    }
  }
}

/*!
 * do gaussian stuff
 */
double CompositeProbabilityComputer::getProbabilityForRecognitionScore(const double &score)
{
  const double mean_correct = 0.002335; const double std_correct = 0.000622;
  const double mean_incorrect = 0.003524; const double std_incorrect = 0.000769;
  double p_correct = std::exp(-0.5*std::pow(score-mean_correct,2)/std::pow(std_correct,2));
  double p_incorrect = std::exp(-0.5*std::pow(score-mean_incorrect,2)/std::pow(std_incorrect,2));
  
  if (p_incorrect + p_correct < 1e-10) return 0.;
  return 0.8 * p_correct / (p_incorrect+p_correct);
}

/*!
 * Given a list of representations which describe the underlying real world object, assigns probabilities for each 
 * representation that the representation is the best description of the real object.
 *
 * The probability that the object represented by the cluster is even in the database is (hopefully)
 * the probability that the object matches the best model fit. Therefore, all of the model probabilities
 * each get some share of the probability that the object is in the DB. The rest of the probability goes to
 * model id -1, which is the probability that it is NOT in the DB (and therefore we should use the cluster
 * grasp planner).
 */
void CompositeProbabilityComputer::computeRepresentationProbabilities(std::vector<ObjectRepresentation> &representations)
{
  if (representations.empty())
  {
    ROS_ERROR("Cannot prepare probabilities list for empty representation list");
    return;
  }

  double best_recognition_probability = 0.0;
  double sum_of_probabilities = 0.0;
  BOOST_FOREACH(const ObjectRepresentation &representation, representations)
  {
    if (!representation.object.potential_models.empty())
    {
      double prob = getProbabilityForRecognitionScore(representation.object.potential_models[0].confidence);
      sum_of_probabilities += prob;
      best_recognition_probability = std::max(best_recognition_probability, prob);
    }
  }

  BOOST_FOREACH(ObjectRepresentation &representation, representations)
  {
    double probability;
    if (db_only_)
    {
      if (!representation.object.potential_models.empty())
      {
		if (sum_of_probabilities > 1e-10){
		  probability = getProbabilityForRecognitionScore(
                                  representation.object.potential_models[0].confidence) / sum_of_probabilities;
		}
		else probability = 0.;
		representation.probability = probability;
      }
      else
      {
        representation.probability = 0.0;
      }
    }
    else
    {
      if (!representation.object.potential_models.empty())
      {
		if (sum_of_probabilities > 1e-10){
		  probability = best_recognition_probability * getProbabilityForRecognitionScore(
                                   representation.object.potential_models[0].confidence) / sum_of_probabilities;
		}
		else probability = 0.;
        representation.probability = probability;
      }
      else
      {
        //! Assign the rest of the probability to the hypothesis "not in DB" for use by cluster planner
        representation.probability = 1.0-best_recognition_probability;
      }
    }
  }
}

/*!
 * Evaluates:
 * [         1                ]       1
 * [-------------------- - 1/c] * --------
 * [1/c + (s/t)*(c-1/c)       ]    c - 1/c
 */
double InverseCurveRecognitionProbabilityComputer::getProbabilityForRecognitionScore(const double &score)
{
  double q = 1.0/curvature_;
  double val = (1.0/(q+(score/recognition_threshold_ * (curvature_ - q))) - q)*(1.0/(curvature_-q));
  return std::max(0.0,val);
}

void LearnedProbabilityComputer::computeRepresentationProbabilities(std::vector<ObjectRepresentation> &representations)
{
  int this_case = CLUSTER_ONLY;
  //Figure out which case we are dealing with
  bool cluster_found = false;
  int num_db_objects = 0;
  double best_db_score = 100; //Need to figure out which db model is the best in case they aren't sorted
  size_t best_db_idx = 0;

  size_t idx = 0;

  ObjectRepresentation* cluster_ptr = NULL;
  ObjectRepresentation* best_db_model = NULL;
  std::vector<ObjectRepresentation*> other_db_models;
  BOOST_FOREACH(ObjectRepresentation &representation,representations)
  {
    if (!representation.object.potential_models.empty())
    {
      ++num_db_objects;
      if (representation.object.potential_models[0].confidence < best_db_score)
      {
        best_db_score = representation.object.potential_models[0].confidence;
        best_db_idx = idx;
      }
    }
    else
    {
      cluster_found = true;
      cluster_ptr = &representation;
    }
    ++idx;
  }

  best_db_model = &representations[best_db_idx];
  for (size_t i=0; i < representations.size(); ++i)
  {
    if (i != best_db_idx && !representations[i].object.potential_models.empty())
    {
      other_db_models.push_back(&representations[i]);
    }
  }

  if (cluster_found)
  {
    if (num_db_objects == 0)
    {
      this_case = CLUSTER_ONLY;
    }
    else if (num_db_objects == 1)
    {
      this_case = CLUSTER_DB;
    }
    else
    {
      this_case = CLUSTER_DB_MULTIPLE;
    }
  }
  else
  {
    if (num_db_objects == 0)
    {
      ROS_ERROR("No objects given");
      return;
    }
    else if (num_db_objects == 1)
    {
      this_case = DB_ONLY;
    }
    else
    {
      this_case = DB_MULTIPLE;
    }
  }

  //Assign probabilities accordingly
  switch (this_case)
  {
    case CLUSTER_ONLY:
    {
      cluster_ptr->probability = 1.0;
      break;
    }
    case CLUSTER_DB:
    {
      cluster_ptr->probability = 0.333;
      best_db_model->probability = 0.667;
      break;
    }
    case CLUSTER_DB_MULTIPLE:
    {
      cluster_ptr->probability = 0.25;
      best_db_model->probability = 0.50;
      int num_other_models = other_db_models.size();
      BOOST_FOREACH(ObjectRepresentation* representation,other_db_models)
      {
        representation->probability = 0.25/num_other_models;
      }
      break;
    }
    case DB_ONLY:
    {
      best_db_model->probability = 1.0;
      break;
    }
    case DB_MULTIPLE:
    {
      best_db_model->probability = 0.667;
      int num_other_models = other_db_models.size();
      BOOST_FOREACH(ObjectRepresentation* representation,other_db_models)
      {
        representation->probability = 0.333/num_other_models;
      }
      break;
    }
  }
}

}
