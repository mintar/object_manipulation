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

#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulation_msgs/GraspPlanningAction.h>
#include <object_manipulation_msgs/GraspPlanningGoal.h>
#include <object_manipulation_msgs/GraspPlanningResult.h>

#include <household_objects_database/objects_database.h>
using household_objects_database::ObjectsDatabasePtr;

#include "bayesian_grasp_planner/grasp_evaluator.h"
#include "bayesian_grasp_planner/grasp_generator.h"
#include "bayesian_grasp_planner/bayesian_grasp_planner_tools.h"

namespace bayesian_grasp_planner {

class RegressionGraspEvaluatorNode
{
private:

  ros::NodeHandle priv_nh_;
  ros::NodeHandle root_nh_;

  //! Service server for grasp evaluation
  ros::ServiceServer grasp_eval_srv_;

  ObjectsDatabasePtr database_;

  //bool show_colored_grasps_;

  //! Initializes a connection to the household_objects_database
  bool initializeDatabase();

  //! Callback for the grasp eval service
  bool graspEvalCB(object_manipulation_msgs::GraspPlanning::Request &request,
                   object_manipulation_msgs::GraspPlanning::Response &response);

  void appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
  								  std::vector<GraspWM> &output_list,
  								  const std::string frame_id);

public:
  RegressionGraspEvaluatorNode();
  virtual ~RegressionGraspEvaluatorNode() {};
};

RegressionGraspEvaluatorNode::RegressionGraspEvaluatorNode():
    priv_nh_("~"),
    root_nh_("")
{
  if ( !initializeDatabase() ) ros::shutdown();

  grasp_eval_srv_ = root_nh_.advertiseService("evaluate_grasp_regression",
                                              &RegressionGraspEvaluatorNode::graspEvalCB, this);

  ROS_INFO("Regression grasp evaluator service ready");
}


bool RegressionGraspEvaluatorNode::initializeDatabase()
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


bool RegressionGraspEvaluatorNode::graspEvalCB(object_manipulation_msgs::GraspPlanning::Request &request,
										   object_manipulation_msgs::GraspPlanning::Response &response)
{
   //create the probability distributions for graspit raw results
   boost::shared_ptr<ProbabilityDistribution> graspit_success_distribution(
 						 new GaussianProbabilityDistribution(.825, .125, 0.0, 1.0, false) );
   boost::shared_ptr<ProbabilityDistribution> graspit_failure_distribution(
 						 new GaussianProbabilityDistribution(.627, .376, 0.0, 1.0, false) );

   //one for each database model/pose detected
   std::vector<object_manipulation_msgs::GraspableObject> objects;
   for (size_t i = 0; i < request.target.potential_models.size(); i++)
   {
 	object_manipulation_msgs::GraspableObject new_object;
 	new_object.potential_models.push_back(request.target.potential_models[i]);
 	new_object.reference_frame_id = request.target.reference_frame_id;
 	objects.push_back(new_object);
   }

   //create the graspit-based regression evaluator
   boost::shared_ptr<MultiplexEvaluator> graspit_multiplexer_regression_eval( new MultiplexEvaluator );

   //for each database object
   BOOST_FOREACH(const household_objects_database_msgs::DatabaseModelPose &model, request.target.potential_models)
   {
     //create a grasp generator with all grasps
     boost::shared_ptr<GraspGenerator> grasp_gen( new GraspGeneratorDatabaseRetriever(database_, model, request.arm_name, false) );
     grasp_gen->generateGrasps();

     //create a regression evaluator based on it
     boost::shared_ptr<RawGraspEvaluator> regression_eval( new RawGraspEvaluatorWithRegression(grasp_gen, true) );

     //put that in our overall multiplexer graspit-regression-based evaluator
     graspit_multiplexer_regression_eval->addEvaluator(regression_eval, model.model_id);
   }

   //create a probabilistic estimator based on the graspit raw evaluator
   boost::shared_ptr<GraspEvaluatorProb> prob_eval( new GraspEvaluatorProb( graspit_success_distribution,
                                                                            graspit_failure_distribution,
                                                                            graspit_multiplexer_regression_eval ) );

   //add metadata to grasps we need to evaluate
   std::vector<GraspWM> grasps;
   appendMetadataToTestGrasps(request.grasps_to_evaluate, grasps, request.target.reference_frame_id);

   std::vector<double> grasp_success_probs;
   std::vector<double> grasp_failure_probs;

   // initialize the success_cond_probs and failure_cond_probs for all grasps
   BOOST_FOREACH(GraspWM &grasp, grasps)
   {
     grasp.success_cond_probs.resize(objects.size());
     grasp.failure_cond_probs.resize(objects.size());
   }

   // evaluate the grasps
   for(size_t object_ind = 0; object_ind < objects.size(); object_ind++)
   {
     // ignore the nd-object case
     if(objects[object_ind].potential_models.empty()) continue;

     // evaluate all grasps at once
     prob_eval->getProbabilitiesForGraspList(grasps, objects[object_ind], grasp_success_probs, grasp_failure_probs);

     // ROS_INFO("evaluting grasps on object %d", (int)object_ind);
     //printf("grasp_success_prob: ");
     //pplist(grasp_success_probs);
     //printf("grasp_failure_prob: ");
     //pplist(grasp_failure_probs);

     // put the results in the appropriate list for this object rep
     for (size_t grasp_ind = 0; grasp_ind < grasps.size(); grasp_ind++)
     {
       grasps[grasp_ind].success_cond_probs[object_ind].push_back(grasp_success_probs[grasp_ind]);
       grasps[grasp_ind].failure_cond_probs[object_ind].push_back(grasp_failure_probs[grasp_ind]);
     }
   }

   response.grasps.reserve(grasps.size());
   for(size_t grasp_i = 0; grasp_i < grasps.size(); grasp_i++)
   {
	   double p_success;

	   p_success = grasp_success_probs[grasp_i] / (grasp_success_probs[grasp_i] + grasp_failure_probs[grasp_i]);
	   response.grasps.push_back(grasps[grasp_i].getGrasp());
	   response.grasps[grasp_i].success_probability = p_success;
   }

  ROS_INFO("Regression grasp evaluator: tested %zd grasps", response.grasps.size());
  response.error_code.value = response.error_code.SUCCESS;
  return true;
}

void RegressionGraspEvaluatorNode::appendMetadataToTestGrasps(std::vector<object_manipulation_msgs::Grasp> &input_list,
                                                      std::vector<GraspWM> &output_list, const std::string frame_id)
{

  output_list.reserve(output_list.size()+input_list.size());

  ROS_INFO("Got %zd grasps to test",input_list.size());
  BOOST_FOREACH(object_manipulation_msgs::Grasp &grasp, input_list)
  {
    GraspWM grasp_wm;
    grasp_wm.grasp_ = grasp;
    grasp_wm.grasp_.success_probability = 0.0;
    grasp_wm.model_id_ = -1;
    grasp_wm.grasp_id_ = -1;

    grasp_wm.object_pose_.setIdentity();
    grasp_wm.object_pose_.frame_id_ = frame_id;
    grasp_wm.object_pose_.stamp_ = ros::Time(0);

    //! The "tool point" is roughly in the middle of the object enclosed by the grasp, so roughly 13cm from wrist
    tf::Pose grasp_in_base_frame;
    tf::poseMsgToTF(grasp.grasp_pose, grasp_in_base_frame);
    double wrist_to_tool_point_offset_ = 0.13;
    tf::Pose grasp_to_tool_point(btMatrix3x3::getIdentity(),btVector3(wrist_to_tool_point_offset_,0,0));
    grasp_wm.tool_point_pose_ = tf::Stamped<tf::Pose>(
        grasp_in_base_frame * grasp_to_tool_point, grasp_wm.object_pose_.stamp_, frame_id);

    output_list.push_back(grasp_wm);
  }
  ROS_DEBUG("Created %zd grasps", output_list.size());
}

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "regression_grasp_evaluator_node");
  bayesian_grasp_planner::RegressionGraspEvaluatorNode node;
  ros::spin();
  return 0;
}
