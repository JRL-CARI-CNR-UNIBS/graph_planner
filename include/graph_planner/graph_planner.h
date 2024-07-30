#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
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
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/collision_checkers/collision_checker_base_plugin.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>
#include <graph_core/plugins/goal_cost_functions/goal_cost_function_base_plugin.h>

#include <graph_display/graph_display.h>

#include <fstream>
#include <iostream>


namespace pathplan {
namespace dirrt_star {

class MultigoalPlanner: public planning_interface::PlanningContext
{
public:
  MultigoalPlanner ( const std::string& name,
                const std::string& group,
                const moveit::core::RobotModelConstPtr& model
              );


  virtual bool solve(planning_interface::MotionPlanResponse& res) override;
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).*/
  virtual bool terminate() override;

  /** \brief Clear the data structures used by the planner */
  virtual void clear() override;

  void humanCb(const geometry_msgs::PoseArrayConstPtr& msg);

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  //planning_scene::PlanningSceneConstPtr pl
  ros::NodeHandle m_nh;
//  std::shared_ptr<pathplan::Display> display;

  ros::WallDuration m_max_refining_time;
  ros::CallbackQueue m_queue;

  unsigned int m_dof;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd m_lb;
  Eigen::VectorXd m_ub;
  Eigen::VectorXd m_max_speed_;
  std::string group_;
  bool display_flag_=false;
  bool display_tree_=false;
  double display_tree_period_=1.0;

  graph::core::CollisionCheckerPtr checker_;
  graph::core::SamplerPtr sampler_;
  graph::core::TreeSolverPtr solver_;
  graph::core::MetricsPtr metrics_;
  graph::core::HampMetricsPtr hamp_metrics_;
  graph::core::GoalCostFunctionPtr goal_cost_fcn_;


  ros::Subscriber m_centroid_sub;
  ros::Publisher m_solver_performance;

  double collision_distance_=0.04;
  double collision_thread_=5;
  bool m_is_running=false;
  bool m_stop=false;
  bool hamp_=false;



};

}
}
