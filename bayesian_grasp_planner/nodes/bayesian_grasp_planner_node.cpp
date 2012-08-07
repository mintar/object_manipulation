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

#include <utility>
#include <algorithm>
#include <string>
#include <functional>

#include <time.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulation_msgs/GraspPlanningAction.h>
#include <object_manipulation_msgs/GraspPlanningGoal.h>
#include <object_manipulation_msgs/GraspPlanningResult.h>

#include <object_manipulator/tools/grasp_marker_publisher.h>

#include <household_objects_database/objects_database.h>
using household_objects_database::ObjectsDatabasePtr;

#include "bayesian_grasp_planner/bayesian_grasp_planner.h"

namespace bayesian_grasp_planner {

static const std::string PLANNING_MARKERS_OUT_NAME = "grasp_planning_markers";

class BayesianGraspPlannerNode
{
private:

  ros::NodeHandle priv_nh_;
  ros::NodeHandle root_nh_;

  //! Service server for grasp execution
  ros::ServiceServer grasp_execution_srv_;

  //! Action server for grasp planning
  actionlib::SimpleActionServer<object_manipulation_msgs::GraspPlanningAction> grasp_planning_aserv_;
  object_manipulation_msgs::GraspPlanningResult grasp_planning_result_;

  //! Service server for grasp testing
  ros::ServiceServer grasp_test_srv_;

  //! Marker publisher for intermediate grasp lists grouped during the planning process
  object_manipulator::GraspMarkerPublisher debug_grasp_marker_pub_;

  //! Marker publisher for the final grasps list which is returned
  object_manipulator::GraspMarkerPublisher results_grasp_marker_pub_;

  //! Marker publisher for all of the grasps colored by rank
  object_manipulator::GraspMarkerPublisher rank_grasp_marker_pub_;

  ObjectsDatabasePtr database_;

  //bool show_colored_grasps_;

  //! The actual plannner
  BayesianGraspPlannerPtr planner_;

  //! Initializes a connection to the household_objects_database
  bool initializeDatabase();

  //! Callback for the grasp planning service
  bool graspPlanningCB(object_manipulation_msgs::GraspPlanning::Request &request, 
                       object_manipulation_msgs::GraspPlanning::Response &response);

  //! Callback for the grasp test service
  bool graspTestCB(object_manipulation_msgs::GraspPlanning::Request &request,
                   object_manipulation_msgs::GraspPlanning::Response &response);

  //! Execute callback for grasp planning server
  void graspPlanningExecuteCB(const object_manipulation_msgs::GraspPlanningGoalConstPtr &goal);


public:
  BayesianGraspPlannerNode();
  virtual ~BayesianGraspPlannerNode();
};

BayesianGraspPlannerNode::BayesianGraspPlannerNode():
    priv_nh_("~"), 
    root_nh_(""),
    debug_grasp_marker_pub_(PLANNING_MARKERS_OUT_NAME, "", 5.0),
    results_grasp_marker_pub_("final_sorted_grasp_list", "", 5.0),
    rank_grasp_marker_pub_("debug_ranked_grasp_list", "", 5.0),
    grasp_planning_aserv_(root_nh_, "probabilistic_grasp_planning",
        boost::bind(&BayesianGraspPlannerNode::graspPlanningExecuteCB, this, _1), false)

{
  if ( !initializeDatabase() ) ros::shutdown();
  planner_.reset(new BayesianGraspPlanner(database_));

  //Advertise the service
  grasp_execution_srv_ = root_nh_.advertiseService("probabilistic_grasp_planning", 
                                                   &BayesianGraspPlannerNode::graspPlanningCB, this);


  grasp_test_srv_ = root_nh_.advertiseService("test_grasp_on_probabilistic_planner",
                                              &BayesianGraspPlannerNode::graspTestCB, this);

  grasp_planning_aserv_.start();

  ROS_INFO("Robust grasp planner service ready");
}

  
bool BayesianGraspPlannerNode::initializeDatabase()
{
  //try to open a connection to the model database
  std::string database_host, database_port, database_user, database_pass, database_name;
  root_nh_.param<std::string> ("household_objects_database/database_host", database_host, "wgs36");
  //the param server decides by itself that this is an int, so we have to get it as one
  int port_int;
  root_nh_.param<int> ("household_objects_database/database_port", port_int, 5432);
  //convert it to a string
  std::stringstream ss;
  ss << port_int;
  database_port = ss.str();
  root_nh_.param<std::string> ("household_objects_database/database_user", database_user, "willow");
  root_nh_.param<std::string> ("household_objects_database/database_pass", database_pass, "willow");
  root_nh_.param<std::string> ("household_objects_database/database_name", database_name, "household_objects");
  database_.reset(new household_objects_database::ObjectsDatabase(database_host, database_port, database_user,
                                                              database_pass, database_name));
  if (!database_->isConnected())
  {
    ROS_ERROR("Failed to open model database on host %s, port %s, user %s with password %s, database %s",
              database_host.c_str(), database_port.c_str(), database_user.c_str(), database_pass.c_str(),
              database_name.c_str());
    database_.reset(); //Reset the shared ptr
    return false;
  }
  return true;
}

bool BayesianGraspPlannerNode::graspPlanningCB(object_manipulation_msgs::GraspPlanning::Request &request,
                                               object_manipulation_msgs::GraspPlanning::Response &response)
{
  //If we are actually requesting that these grasps are evaluated in our planner, tell the planner to use those
  if (request.grasps_to_evaluate.size() > 0)
  {
    ROS_WARN("Grasps in test list, but grasp planner was called");
  }
  else
  {
    planner_->plan(request.arm_name, request.target, response.grasps);
  }
  ROS_INFO("Probabilistic grasp planner: returning %zd grasps", response.grasps.size());
  response.error_code.value = response.error_code.SUCCESS;
  return true;
}

void BayesianGraspPlannerNode::graspPlanningExecuteCB(
    const object_manipulation_msgs::GraspPlanningGoalConstPtr &goal)
{
  //If we are actually requesting that these grasps are evaluated in our planner, tell the planner to use those
  if (goal->grasps_to_evaluate.size() > 0)
  {
    ROS_WARN("Grasps in test list, but grasp planner was called");
  }
  else
  {
    planner_->plan(goal->arm_name, goal->target, grasp_planning_result_.grasps);
  }
  ROS_INFO("Probabilistic grasp planner: returning %zd grasps", grasp_planning_result_.grasps.size());
  grasp_planning_result_.error_code.value = grasp_planning_result_.error_code.SUCCESS;
  grasp_planning_aserv_.setSucceeded(grasp_planning_result_);
}


bool BayesianGraspPlannerNode::graspTestCB(object_manipulation_msgs::GraspPlanning::Request &request,
										   object_manipulation_msgs::GraspPlanning::Response &response)
{
  //If we are actually requesting that these grasps are evaluated in our planner, tell the planner to use those
  if (request.grasps_to_evaluate.size() > 0)
  {
    planner_->plan(request.arm_name, request.target, request.grasps_to_evaluate);
    response.grasps = request.grasps_to_evaluate;
  }
  else
  {
    ROS_WARN("Grasp test service called but no grasps given to test");
  }
  ROS_INFO("Probabilistic grasp planner: tested %zd grasps", response.grasps.size());
  response.error_code.value = response.error_code.SUCCESS;
  return true;
}


BayesianGraspPlannerNode::~BayesianGraspPlannerNode()
{
}

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bayesian_grasp_planner_node");
  bayesian_grasp_planner::BayesianGraspPlannerNode node;
  ros::spin();
  return 0;
}
