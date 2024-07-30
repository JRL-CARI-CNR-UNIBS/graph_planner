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


#include <graph_planner/graph_planner.h>
#include <cnr_class_loader/multi_library_class_loader.hpp>


namespace pathplan {
namespace dirrt_star {

MultigoalPlanner::MultigoalPlanner ( const std::string& name,
                                     const std::string& group,
                                     const moveit::core::RobotModelConstPtr& model ) :
  PlanningContext ( name, group ),
  group_(group)
{
  m_nh=ros::NodeHandle(name);
  m_nh.setCallbackQueue(&m_queue);


  ROS_DEBUG("create MultigoalPlanner, name =%s, group = %s", name.c_str(),group.c_str());
  robot_model_=model;
  if (!robot_model_)
  {
    ROS_ERROR("robot model is not initialized");
  }

  const moveit::core::JointModelGroup* jmg=robot_model_->getJointModelGroup(group);
  if (jmg==NULL)
    ROS_ERROR("unable to find JointModelGroup for group %s",group.c_str());


  joint_names_=jmg->getActiveJointModelNames();
  m_dof=joint_names_.size();
  m_lb.resize(m_dof);
  m_ub.resize(m_dof);
  m_max_speed_.resize(m_dof);

  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      m_lb(idx)=bounds.min_position_;
      m_ub(idx)=bounds.max_position_;
      m_max_speed_(idx)=bounds.max_velocity_;
    }
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");


  if (!m_nh.getParam("display_bubbles",display_flag_))
  {
    ROS_DEBUG("display_flag is not set, default=false");
    display_flag_=false;
  }

  if (!m_nh.getParam("display_tree",display_tree_))
  {
    ROS_DEBUG("display_tree is not set, default=false");
    display_tree_=false;
  }
  if (display_tree_)
  {
    if (!m_nh.getParam("display_tree_period",display_tree_period_))
    {
      ROS_DEBUG("display_tree_rate is not set, default=1.0");
      display_tree_period_=1.0;
    }
  }


  cnr_class_loader::MultiLibraryClassLoader loader(true);

  std::vector<std::string> libraries;
  if(not m_nh.getParam("libraries",libraries))
  {
    ROS_WARN("libraries are not set. Use the default ones: libgraph_core.so, libmoveit_collision_checker.so");
    libraries.push_back("libgraph_core.so");
    libraries.push_back("libmoveit_collision_checker.so");
  }

  for(const std::string& lib:libraries)
    loader.loadLibrary(lib);


  std::string class_name;
  std::string library_path="graph_core/libgraph_core.so";

  if (!m_nh.getParam("metrics/name",class_name))
  {
    ROS_WARN("metrics/name is not set, default=graph::core::InformedSampler");
    class_name="graph::core::InformedSampler";
  }
  std::shared_ptr<graph::core::MetricsBasePlugin> metrics_plugin = loader.createInstance<graph::core::MetricsBasePlugin>(class_name);
  metrics_ = metrics_plugin->getMetrics();


  if (hamp_)
  {
    if (!m_nh.getParam("hamp_metrics/name",class_name))
    {
      ROS_ERROR("hamp_metrics/name is not set");
      throw std::invalid_argument("hamp_metrics/name is not set");
    }
    std::shared_ptr<graph::core::HampMetricsBasePlugin> hamp_metrics_plugin = loader.createInstance<graph::core::HampMetricsBasePlugin>(class_name);
    hamp_metrics_ = hamp_metrics_plugin->getMetrics();
  }
  else
  {
    hamp_metrics_.reset();
  }

  if (!m_nh.getParam("checker/name",class_name))
  {
    ROS_ERROR("checker/name is not set");
    throw std::invalid_argument("checker/name is not set");
  }
  std::shared_ptr<graph::core::CollisionCheckerBasePlugin> checker_plugin = loader.createInstance<graph::core::CollisionCheckerBasePlugin>(class_name);
  checker_ = checker_plugin->getCollisionChecker();

  if (!m_nh.getParam("sampler/name",class_name))
  {
    ROS_ERROR("sampler/name is not set");
    throw std::invalid_argument("sampler/name is not set");
  }
  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader.createInstance<graph::core::SamplerBasePlugin>(class_name);
  sampler_ = sampler_plugin->getSampler();


  if (!m_nh.getParam("solver/name",class_name))
  {
    ROS_ERROR("sampler/name is not set");
    throw std::invalid_argument("solver/name is not set");
  }
  std::shared_ptr<graph::core::TreeSolverPlugin> solver_plugin = loader.createInstance<graph::core::TreeSolverPlugin>(class_name);
  solver_ = solver_plugin->getSolver();


  if (!m_nh.getParam("goal_cost/name",class_name))
  {
    ROS_WARN("goal_cost/name is not set, default = NONE");
    class_name="";
    goal_cost_fcn_.reset();
  }
  else
  {
    std::shared_ptr<graph::core::GoalCostFunctionBasePlugin> goal_cost_plugin = loader.createInstance<graph::core::GoalCostFunctionBasePlugin>(class_name);
    goal_cost_fcn_ = goal_cost_plugin->getCostFunction();
    solver_->setGoalCostFunction(goal_cost_fcn_);
  }

  ROS_DEBUG("created MultigoalPlanner");

  double refining_time=0;
  if (!m_nh.getParam("max_refine_time",refining_time))
  {
    ROS_DEBUG("refining_time is not set, default=30");
    refining_time=30;
  }
  m_max_refining_time=ros::WallDuration(refining_time);

  m_solver_performance=m_nh.advertise<std_msgs::Float64MultiArray>("/solver_performance",1000);

}



void MultigoalPlanner::clear()
{

}




bool MultigoalPlanner::solve ( planning_interface::MotionPlanDetailedResponse& res )
{

  std::vector<const moveit::core::AttachedBody*> attached_body;
  planning_scene_->getCurrentState().getAttachedBodies(attached_body);

  if (attached_body.size()>0)
  {
    ROS_DEBUG("number of attached objects = %zu", attached_body.size());
    for (const moveit::core::AttachedBody* obj:  attached_body)
    {
      ROS_DEBUG("attached object =%s to link=%s", obj->getName().c_str(),obj->getAttachedLinkName().c_str());
    }
  }


  ros::WallDuration max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  m_is_running=true;

  std_msgs::Float64MultiArray performace_msg;
  performace_msg.layout.dim.resize(4);
  performace_msg.layout.dim.at(0).label="time";
  performace_msg.layout.dim.at(0).size=1;
  performace_msg.layout.dim.at(1).label="iteration";
  performace_msg.layout.dim.at(1).size=1;
  performace_msg.layout.dim.at(2).label="cost";
  performace_msg.layout.dim.at(2).size=1;
  performace_msg.layout.dim.at(3).label=m_nh.getNamespace();
  performace_msg.layout.dim.at(3).size=0;

  if (!planning_scene_)
  {
    ROS_ERROR("No planning scene available");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    m_is_running=false;
    return false;
  }


  //  if (display_flag_ || display_tree_)
  //  {
  //    if (!display)
  //      display=std::make_shared<graph::core::Display>(planning_scene_,group_);
  //    else
  //      display->clearMarkers();
  //  }

  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);

  //checker_->init(); // dovremmo ereditare da MoveItCollisionChecker


  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);
  if (request_.start_state.joint_state.position.size()==0)
    start_state=planning_scene_->getCurrentState();
  else
    moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);

  start_state.update();
  start_state.updateCollisionBodyTransforms();

  const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_);
  if (!start_state.satisfiesBounds(jmg))
  {

    std::vector<const moveit::core::JointModel*> joint_models= jmg->getJointModels();
    for (const moveit::core::JointModel*& jm: joint_models)
    {
      if (!start_state.satisfiesPositionBounds(jm))
      {
        ROS_ERROR("joint %s is out of bound, actual position=%f",jm->getName().c_str(),*start_state.getJointPositions(jm->getName()));
        std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();

        for (const moveit::core::VariableBounds& b: bound)
        {
          ROS_ERROR("joint %s has bound pos = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_);
        }
      }
      if (!start_state.satisfiesVelocityBounds(jm))
      {
        ROS_ERROR("joint %s is out of bound, actual velocity=%f",jm->getName().c_str(),*start_state.getJointVelocities(jm->getName()));
        std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();
        for (const moveit::core::VariableBounds& b: bound)
        {
          ROS_ERROR("joint %s has bound pos=[%f, %f], vel = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_,b.min_velocity_,b.max_velocity_);
        }
      }
    }
    ROS_FATAL("Start point is  Out of bound");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }

  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);

  if (!checker_->check(start_conf))
  {
    ROS_ERROR("Start point is in collision");

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.contacts = true;
    col_req.group_name=group_;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }
    else
    {
      ROS_FATAL("you shouldn't be here!");
    }
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }



  solver_->resetProblem();
  if (!solver_->config("solver")) // metti namespace
  {
    ROS_ERROR("Unable to configure the planner");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    m_is_running=false;
    return false;
  }

  graph::core::NodePtr start_node;
  start_node=std::make_shared<graph::core::Node>(start_conf);
  solver_->addStart(start_node);

  m_queue.callAvailable();
  bool at_least_a_goal=false;

  // joint goal
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    ROS_DEBUG("Processing goal %u",iGoal);

    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);
    if (goal.joint_constraints.size()==0)
    {
      ROS_DEBUG("Goal %u is a Cartesian goal",iGoal);

      if (goal.position_constraints.size()!=1 || goal.orientation_constraints.size()!=1)
      {
        ROS_DEBUG("Goal %u has no position or orientation",iGoal);
        continue;
      }

      Eigen::Quaterniond q_w_tool;
      q_w_tool.x()=goal.orientation_constraints.at(0).orientation.x;
      q_w_tool.y()=goal.orientation_constraints.at(0).orientation.y;
      q_w_tool.z()=goal.orientation_constraints.at(0).orientation.z;
      q_w_tool.w()=goal.orientation_constraints.at(0).orientation.w;


      Eigen::Affine3d T_w_tool;
      T_w_tool.setIdentity();
      T_w_tool=q_w_tool;
      T_w_tool.translation()(0)=goal.position_constraints.at(0).target_point_offset.x;
      T_w_tool.translation()(1)=goal.position_constraints.at(0).target_point_offset.y;
      T_w_tool.translation()(2)=goal.position_constraints.at(0).target_point_offset.z;

      // chiama ik
    }
    else // joint constraints
    {
      ROS_DEBUG("Goal %u is a joint goal",iGoal);
      Eigen::VectorXd goal_configuration( goal.joint_constraints.size() );
      moveit::core::RobotState goal_state(robot_model_);

      for (auto c: goal.joint_constraints)
        goal_state.setJointPositions(c.joint_name,&c.position);
      goal_state.copyJointGroupPositions(group_,goal_configuration);

      goal_state.updateCollisionBodyTransforms();

      if (!checker_->check(goal_configuration))
      {
        ROS_DEBUG("goal %u is in collision",iGoal);

        if (request_.goal_constraints.size()<5)
        {

          if (!goal_state.satisfiesBounds())
          {
            ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is  Out of bound");
          }

          collision_detection::CollisionRequest col_req;
          collision_detection::CollisionResult col_res;
          col_req.contacts = true;
          col_req.group_name=group_;
          planning_scene_->checkCollision(col_req,col_res,goal_state);
          if (col_res.collision)
          {
            ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is colliding");
            for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
            {
              ROS_INFO("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
            }
          }
        }
        continue;
      }

      graph::core::NodePtr goal_node=std::make_shared<graph::core::Node>(goal_configuration);
      solver_->addGoal(goal_node);
      at_least_a_goal=true;
    }
  }


  if (!at_least_a_goal)
  {
    ROS_ERROR("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    m_is_running=false;
    return false;
  }

  solver_->finalizeProblem();

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================


  // searching initial solutions
  graph::core::PathPtr solution;
  bool found_a_solution=false;
  unsigned int iteration=0;
  ros::WallTime display_time = ros::WallTime::now();
  while((ros::WallTime::now()-start_time)<max_planning_time)
  {

    performace_msg.data.push_back((ros::WallTime::now()-start_time).toSec());
    performace_msg.data.push_back(iteration);
    performace_msg.data.push_back(solver_->getCost());

    //    if (display_tree_)
    //    {
    //      if ((ros::WallTime::now()-display_time).toSec()>display_tree_period_)
    //      {
    //        display_time = ros::WallTime::now();
    //        display->displayTree(solver->getStartTree());
    //        std::vector<TreePtr> goal_trees = solver->getGoalTrees();
    //        for (auto& goal_tree: goal_trees)
    //          display->displayTree(goal_tree);
    //      }
    //    }
    iteration++;
    if (m_stop)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      m_is_running=false;
      return false;
    }

    solver_->update(solution);
    if (!found_a_solution && solver_->solved())
    {
      assert(solution);
      ROS_INFO("Find a first solution (cost=%f) in %f seconds",solver_->cost(),(ros::WallTime::now()-start_time).toSec());
      ROS_DEBUG_STREAM(*solver_);
      found_a_solution=true;
      refine_time = ros::WallTime::now();
    }
    if (solver_->completed())
    {
      ROS_INFO("Optimization completed (cost=%f) in %f seconds (%u iterations)",solver_->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }

    if (found_a_solution && ((ros::WallTime::now()-refine_time)>m_max_refining_time))
    {
      ROS_INFO("Refine time expired (cost=%f) in %f seconds (%u iterations)",solver_->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }
  }


  m_solver_performance.publish( performace_msg);

  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    m_is_running=false;
    //    if (display_flag_)
    //      display->displayTree(solver->getStartTree());
    return false;
  }
  //  if (display_flag)
  //    display->displayPath(solution);

  if (!solver_->completed())
  {
    ROS_INFO("Stopped (cost=%f) after %f seconds (%u iterations)",solver_->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
  }

  // =========================
  // END OF THE IMPORTANT PART
  // =========================
  std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();// PUT WAY POINTS
  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  for (const Eigen::VectorXd& waypoint: waypoints )
  {
    moveit::core::RobotState wp_state=start_state;
    wp_state.setJointGroupPositions(group_,waypoint);
    wp_state.update();
    trj->addSuffixWayPoint(wp_state,0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;


  res.processing_time_.push_back(wd.toSec());
  res.description_.emplace_back("plan");
  res.trajectory_.push_back(trj);

  res.error_code_.val=moveit_msgs::MoveItErrorCodes::SUCCESS;
  m_is_running=false;

  return true;
}

bool MultigoalPlanner::solve ( planning_interface::MotionPlanResponse& res )
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.at(0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = wd.toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool MultigoalPlanner::terminate()
{
  m_stop=true;
  ros::Time t0=ros::Time::now();
  ros::Rate lp(50);
  while (ros::ok())
  {
    if (!m_is_running)
      return true;
    lp.sleep();
    if ((ros::Time::now()-t0).toSec()>5)
    {
      ROS_ERROR("Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
  }
  return false;
}

void MultigoalPlanner::humanCb(const geometry_msgs::PoseArrayConstPtr& msg)
{

  if (!hamp_)
    return;
  //  if (use_avoidance_goal_)
  //    goal_cost_fcn_->cleanPoints();
  //  if (use_avoidance_metrics_)
  //    avoidance_metrics_->cleanPoints();
  Eigen::Vector3d point;
  for (const geometry_msgs::Pose& p: msg->poses)
  {
    point(0)=p.position.x;
    point(1)=p.position.y;
    point(2)=p.position.z;
    //    if (use_avoidance_goal_)
    //      goal_cost_fcn_->addPoint(point);
    //    ros::Duration(0.1).sleep();
    //    if (use_avoidance_metrics_)
    //      avoidance_metrics_->addPoint(point);
  }
  //  goal_cost_fcn_->publishPoints();
}


}  // namespace dirrt_star
}  // namespace graph::core
