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
#include "probabilistic_grasp_planner/grasp_success_probability_computer.h"
#include "probabilistic_grasp_planner/probabilistic_planner_tools.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(GraspSuccessProbabilityComputer, testSimpleGraspSuccessProbabilityComputer)
{
  probabilistic_grasp_planner::SimpleGraspSuccessProbabilityComputer computer(18665,10.0);


  probabilistic_grasp_planner::GraspWithMetadata grasp;
  grasp.model_id_ = 18665;
  grasp.energy_function_score_ = 50.0;
  EXPECT_DOUBLE_EQ(computer.getProbability(grasp),0.51243713187574524);

  grasp.energy_function_score_ = 0.0;
  EXPECT_DOUBLE_EQ(computer.getProbability(grasp), 0.95);

  grasp.model_id_ = 0;
  EXPECT_DOUBLE_EQ(computer.getProbability(grasp), 0.0);
}


// Declare another test
TEST(GraspSuccessProbabilityComputer, testGSPCWithEstimation)
{
  //Test that
}

//TEST(TestSuite, )

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
