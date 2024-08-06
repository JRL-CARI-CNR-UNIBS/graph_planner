#pragma once
/*
Copyright (c) 2024, Manuel Beschi UNIBS manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <graph_core/metrics/goal_cost_function_base.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/callback_queue.h>

#include <graph_core/plugins/metrics/metrics_base_plugin.h>
#include <graph_core/plugins/metrics/hamp_metrics_base_plugin.h>
#include <graph_core/plugins/metrics/goal_cost_function_base_plugin.h>
#include <graph_core/plugins/metrics/hamp_goal_cost_function_base_plugin.h>
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/collision_checkers/collision_checker_base_plugin.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>
#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>
#include <graph_display/graph_display.h>

#include <cnr_class_loader/multi_library_class_loader.hpp>

#include <fstream>
#include <iostream>


namespace graph {
namespace planner {

/**
 * @class GraphPlanner
 * @brief The GraphPlanner class implements a planning context using graph_core.
 */
class GraphPlanner: public planning_interface::PlanningContext
{
public:
  /**
   * @brief Constructor for the GraphPlanner class.
   * @param name The name of the planning context.
   * @param group The name of the planning group.
   * @param model The robot model pointer.
   * @param logger The logger pointer.
   */
  GraphPlanner ( const std::string& name,
                     const std::string& group,
                     const moveit::core::RobotModelConstPtr& model,
                     const cnr_logger::TraceLoggerPtr& logger
                     );

  /**
   * @brief Initialize the GraphPlanner.
   * @return True if initialization is successful, false otherwise.
   */
  bool init();

  /**
   * @brief Solve the motion planning problem.
   * @param res The motion plan response.
   * @return True if planning is successful, false otherwise.
   */
  virtual bool solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Solve the motion planning problem with detailed response.
   * @param res The motion plan detailed response.
   * @return True if planning is successful, false otherwise.
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Terminate the ongoing solve process.
   * @return False if termination is not possible, true if solve() is not running or termination is successful.
   */
  virtual bool terminate() override;

  /**
   * @brief Clear the planner's data structures.
   */
  virtual void clear() override;

  /**
   * @brief Callback for receiving human pose messages.
   * @param msg The received message containing human poses.
   */
  void humanPoseCb(const geometry_msgs::PoseArrayConstPtr& msg);
//  void humanVelocityCb(const geometry_msgs::TwistArrayConstPtr& msg);



protected:
  moveit::core::RobotModelConstPtr robot_model_; ///< The robot model pointer.
  ros::NodeHandle nh_; ///< ROS node handle.
  std::shared_ptr<graph::display::Display> display_; ///< Display pointer for visualization.

  graph_duration max_refining_time_; ///< Maximum refining time duration.
  ros::CallbackQueue queue_; ///< ROS callback queue.

  unsigned int dof_; ///< Degrees of freedom.
  std::vector<std::string> joint_names_; ///< Joint names.
  Eigen::VectorXd lower_bounds_; ///< Lower bounds for joint limits.
  Eigen::VectorXd upper_bounds_; ///< Upper bounds for joint limits.
  Eigen::VectorXd max_speed_; ///< Maximum joint speeds.
  Eigen::VectorXd scale_; ///< Scaling factors.
  std::string group_; ///< Planning group name.

  graph::collision_check::MoveitCollisionCheckerPtr checker_; ///< Collision checker pointer.
  graph::core::SamplerPtr sampler_; ///< Sampler pointer.
  graph::core::TreeSolverPtr solver_; ///< Solver pointer.
  graph::core::MetricsPtr metrics_; ///< Metrics pointer.
  graph::core::HampMetricsPtr hamp_metrics_; ///< HAMP metrics pointer.
  graph::core::GoalCostFunctionPtr goal_cost_fcn_; ///< Goal cost function pointer.
  graph::core::HampGoalCostFunctionPtr hamp_goal_cost_fcn_; ///< HAMP goal cost function pointer.

  ros::Subscriber human_poses_sub_; ///< Subscriber for human pose messages.
  ros::Publisher solver_performance_; ///< Publisher for solver performance.

  bool is_running_ = false; ///< Flag indicating if the planner is running.
  bool stop_ = false; ///< Flag to stop the planner.
  bool hamp_ = false; ///< Flag indicating if HAMP is used.

  cnr_logger::TraceLoggerPtr logger_; ///< Logger pointer.

  cnr_class_loader::MultiLibraryClassLoader loader_; ///< Multi-library class loader.
  std::string parameter_namespace_; ///< Parameter namespace.
  std::string checker_name_; ///< Checker name.
  std::string sampler_name_; ///< Sampler name.

  bool display_flag_ = false; ///< Display flag.
  bool display_tree_ = false; ///< Display tree flag.
  graph_duration display_tree_period_; ///< Display tree period.
};

} // namespace planner
} // namespace graph
