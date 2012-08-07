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

//! Author: Peter Brook

// Bring in my package's API, which is what I'm testing
#include "probabilistic_grasp_planner/distribution_evaluator.h"
#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(DistributionEvaluator, testUniformDistribution)
{
  probabilistic_grasp_planner::UniformDistributionEvaluator evaluator;

  probabilistic_grasp_planner::GraspWithMetadata grasp;
  EXPECT_DOUBLE_EQ(evaluator.get_normalization_term(),0);
  double res = evaluator.evaluate(grasp,grasp);
  EXPECT_DOUBLE_EQ(res, 1.0);

}


// Declare another test
TEST(DistributionEvaluator, testNormalDistribution)
{
  double position_sigma = 0.01;
  double orientation_concentration = 10.0;

  probabilistic_grasp_planner::NormalDistributionEvaluator evaluator2(position_sigma, orientation_concentration);
  EXPECT_DOUBLE_EQ(evaluator2.get_normalization_term(),0);

  probabilistic_grasp_planner::GraspWithMetadata gstar;
  gstar.tool_point_pose_.setIdentity();

  probabilistic_grasp_planner::GraspWithMetadata grasp1;
  grasp1.tool_point_pose_.setIdentity();
  grasp1.tool_point_pose_.setOrigin(tf::Vector3(0.01,0,0));

  double res = evaluator2.evaluate(gstar,grasp1);
  EXPECT_DOUBLE_EQ(res, 0.60653065971263342);

  probabilistic_grasp_planner::GraspWithMetadata grasp2;
  grasp2.tool_point_pose_.setIdentity();
  grasp2.tool_point_pose_.setOrigin(tf::Vector3(0.01,0.01,0));
  res = evaluator2.evaluate(gstar,grasp2);
  EXPECT_DOUBLE_EQ(res, 0.36787944117144233);

  probabilistic_grasp_planner::GraspWithMetadata grasp3;
  grasp3.tool_point_pose_.setIdentity();
  tf::Matrix3x3 basis;
  basis.setEulerYPR(0.0,0.0,0.0);
  grasp3.tool_point_pose_.setBasis(basis);
  res = evaluator2.evaluate(gstar,grasp3);
  EXPECT_DOUBLE_EQ(res, 1.0);

  basis.setEulerYPR(0.0,0.0,M_PI_4);
  grasp3.tool_point_pose_.setBasis(basis);
  res = evaluator2.evaluate(gstar,grasp3);
  EXPECT_DOUBLE_EQ(res, 0.23120139832879863);

}

//TEST(TestSuite, )

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
