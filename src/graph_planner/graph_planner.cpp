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


#include <graph_planner/graph_planner.h>
#include <thread>


namespace graph {
namespace planner {

GraphPlanner::GraphPlanner (const std::string& name,
                            const std::string& group,
                            const moveit::core::RobotModelConstPtr& model ,
                            const cnr_logger::TraceLoggerPtr &logger) :
  PlanningContext ( name, group ),
  robot_model_(model),
  group_(group),
  logger_(logger),
  loader_(true)
{
  parameter_namespace_=name;
  nh_=ros::NodeHandle(name);
  nh_.setCallbackQueue(&queue_);
  display_tree_period_=graph_duration(1.0);
}

bool GraphPlanner::init()
{
  CNR_TRACE(logger_,"create CARI planner, name =%s, group = %s", parameter_namespace_.c_str(),group_.c_str());
  if (!robot_model_)
  {
    CNR_ERROR(logger_,"robot model is not initialized");
    return false;
  }

  const moveit::core::JointModelGroup* jmg=robot_model_->getJointModelGroup(group_);
  if (jmg==NULL)
  {
    CNR_ERROR(logger_,"unable to find JointModelGroup for group %s",group_.c_str());
    return false;
  }


  joint_names_=jmg->getActiveJointModelNames();
  dof_=joint_names_.size();
  lower_bounds_.resize(dof_);
  upper_bounds_.resize(dof_);
  max_speed_.resize(dof_);
  scale_.resize(dof_);
  scale_.setOnes();

  for (unsigned int idx=0;idx<dof_;idx++)
  {
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      lower_bounds_(idx)=bounds.min_position_;
      upper_bounds_(idx)=bounds.max_position_;
      max_speed_(idx)=bounds.max_velocity_;
    }
    CNR_TRACE(logger_," joint " << joint_names_.at(idx) << " bounds: upper = " << upper_bounds_(idx) << " lower = " << lower_bounds_(idx) << " velocity = " << max_speed_(idx));
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");


  if(not graph::core::get_param(logger_,parameter_namespace_,"display_path",display_flag_))
  {
    CNR_DEBUG(logger_,"display_flag is not set, default=false");
    display_flag_=false;
  }

  if(not graph::core::get_param(logger_,parameter_namespace_,"display_tree",display_tree_))
  {
    CNR_DEBUG(logger_,"display_tree is not set, default=false");
    display_tree_=false;
  }
  if (display_tree_)
  {
    double display_tree_period;
    if(not graph::core::get_param(logger_,parameter_namespace_,"display_tree_period",display_tree_period))
    {
      CNR_DEBUG(logger_,"display_tree_period is not set, default=1.0");
      display_tree_period=1.0;
    }
    display_tree_period_=graph_duration(display_tree_period);
  }



  std::vector<std::string> libraries;
  if(not graph::core::get_param(logger_,parameter_namespace_,"libraries",libraries))
  {
    CNR_WARN(logger_,"libraries are not set. Use the default ones: libgraph_core.so, libmoveit_collision_checker.so");
    libraries.push_back("libgraph_core.so");
    libraries.push_back("libmoveit_collision_checker.so");
  }

  for(const std::string& lib:libraries)
    loader_.loadLibrary(lib);


  std::string class_name;

  if(not graph::core::get_param(logger_,parameter_namespace_,"metrics_plugin",class_name))
  {
    CNR_WARN(logger_,"metrics_plugin is not set, default=graph::core::EuclideanMetricsPlugin");
    class_name="graph::core::EuclideanMetricsPlugin";
  }
  std::shared_ptr<graph::core::MetricsBasePlugin> metrics_plugin = loader_.createInstance<graph::core::MetricsBasePlugin>(class_name);
  metrics_plugin->init(parameter_namespace_,logger_);
  metrics_ = metrics_plugin->getMetrics();
  CNR_TRACE(logger_,"CREATED METRICS");

  if(not graph::core::get_param(logger_,parameter_namespace_,"use_hamp",hamp_))
  {
    CNR_WARN(logger_,"use_hamp is not set, default=false");
    hamp_=false;
  }
  if (hamp_)
  {

    if(not graph::core::get_param(logger_,parameter_namespace_,"hamp_metrics_plugin",class_name))
    {
      CNR_ERROR(logger_,"hamp_metrics_plugin is not set");
      return false;
    }
    std::shared_ptr<graph::core::HampMetricsBasePlugin> hamp_metrics_plugin = loader_.createInstance<graph::core::HampMetricsBasePlugin>(class_name);
    hamp_metrics_plugin->init(parameter_namespace_,logger_);
    hamp_metrics_ = hamp_metrics_plugin->getMetrics();
    CNR_TRACE(logger_,"CREATED HAMP METRICS");

    if(not graph::core::get_param(logger_,parameter_namespace_,"hamp_metrics_plugin",class_name))
    {
      CNR_ERROR(logger_,"hamp_metrics_plugin is not set");
      return false;
    }
    std::shared_ptr<graph::core::HampGoalCostFunctionBasePlugin> hamp_goal_cost_plugin = loader_.createInstance<graph::core::HampGoalCostFunctionBasePlugin>(class_name);
    hamp_goal_cost_plugin->init(parameter_namespace_,logger_);
    hamp_goal_cost_fcn_ = hamp_goal_cost_plugin->getCostFunction();
    CNR_TRACE(logger_,"CREATED HAMP GOAL COST FUNCTION");
  }
  else
  {
    hamp_metrics_.reset();
  }


  if(not graph::core::get_param(logger_,parameter_namespace_,"checker_plugin",checker_name_))
  {
    CNR_WARN(logger_,"checker_plugin is not set, default=graph::collision_check::ParallelMoveitCollisionCheckerPlugin");
    checker_name_="graph::collision_check::ParallelMoveitCollisionCheckerPlugin";
  }



  if(not graph::core::get_param(logger_,parameter_namespace_,"sampler_plugin",sampler_name_))
  {
    CNR_WARN(logger_,"sampler_plugin is not set, default=graph::core::InformedSamplerPlugin");
    sampler_name_="graph::core::InformedSamplerPlugin";
  }


  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader_.createInstance<graph::core::SamplerBasePlugin>(sampler_name_);
  sampler_plugin->init(parameter_namespace_,
                       lower_bounds_,
                       upper_bounds_,
                       lower_bounds_,
                       upper_bounds_,
                       upper_bounds_-lower_bounds_,
                       logger_);
  sampler_ = sampler_plugin->getSampler();


  if(not graph::core::get_param(logger_,parameter_namespace_,"goal_cost_plugin",class_name))
  {
    CNR_WARN(logger_,"goal_cost_plugin is not set, default = graph::core::NullGoalCostFunctionPlugin");
    class_name="graph::core::NullGoalCostFunctionPlugin";
  }

  std::shared_ptr<graph::core::GoalCostFunctionBasePlugin> goal_cost_plugin = loader_.createInstance<graph::core::GoalCostFunctionBasePlugin>(class_name);

  goal_cost_plugin->init(parameter_namespace_,
                         logger_);

  goal_cost_fcn_ = goal_cost_plugin->getCostFunction();

  CNR_TRACE(logger_,"CREATED GOAL COST FUNCTION");


  if(not graph::core::get_param(logger_,parameter_namespace_,"solver_plugin",class_name))
  {
    CNR_WARN(logger_,"solver_plugin is not set, default=graph::core::RRTPlugin");
    class_name="graph::core::RRTPlugin";
  }
  std::shared_ptr<graph::core::TreeSolverPlugin> solver_plugin = loader_.createInstance<graph::core::TreeSolverPlugin>(class_name);
  solver_plugin->init(parameter_namespace_,
                      metrics_,
                      checker_,
                      sampler_,
                      goal_cost_fcn_,
                      logger_);

  solver_ = solver_plugin->getSolver();

  if (!solver_->config(parameter_namespace_)) // metti namespace
  {
    CNR_ERROR(logger_,"Unable to configure the planner");
    return false;
  }


  CNR_TRACE(logger_,"CREATED SOLVER");





  double refining_time=0;
  if(not graph::core::get_param(logger_,parameter_namespace_,"max_refine_time",refining_time))
  {
    CNR_DEBUG(logger_,"refining_time is not set, default=30 seconds");
    refining_time=30;
  }
  max_refining_time_=graph_duration(refining_time);

  solver_performance_=nh_.advertise<std_msgs::Float64MultiArray>("/solver_performance",1000);

  human_poses_sub_=nh_.subscribe("human_poses",
                                  1,
                                  &GraphPlanner::humanPoseCb,
                                  this);
//  human_vel_sub_=m_nh.subscribe("human_velocities",
//                                1,
//                                &GraphPlanner::humanVelocityCb,
//                                this);


  CNR_TRACE(logger_,"created CARI planner");

  return true;
}



void GraphPlanner::clear()
{

}




bool GraphPlanner::solve ( planning_interface::MotionPlanDetailedResponse& res )
{
  std::shared_ptr<graph::collision_check::MoveitCollisionCheckerBasePlugin> checker_plugin = loader_.createInstance<graph::collision_check::MoveitCollisionCheckerBasePlugin>(checker_name_);

  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);

  checker_plugin->init(parameter_namespace_,ptr,logger_);

  checker_ = checker_plugin->getCollisionChecker();
  CNR_TRACE(logger_,"CREATED CHECKER");

  std::vector<const moveit::core::AttachedBody*> attached_body;
  planning_scene_->getCurrentState().getAttachedBodies(attached_body);

  solver_->setChecker(checker_);
  solver_->resetProblem();
  sampler_->setCost(std::numeric_limits<double>::infinity()); // reset sampler

  if (attached_body.size()>0)
  {
    CNR_DEBUG(logger_,"number of attached objects = %zu", attached_body.size());
    for (const moveit::core::AttachedBody* obj:  attached_body)
    {
      CNR_DEBUG(logger_,"attached object =%s to link=%s", obj->getName().c_str(),obj->getAttachedLinkName().c_str());
    }
  }


  graph_duration max_planning_time=graph_duration(request_.allowed_planning_time);
  graph_time_point start_time = graph_time::now();
  graph_time_point refine_time = graph_time::now();
  is_running_=true;

  std_msgs::Float64MultiArray performace_msg;
  performace_msg.layout.dim.resize(4);
  performace_msg.layout.dim.at(0).label="time";
  performace_msg.layout.dim.at(0).size=1;
  performace_msg.layout.dim.at(1).label="iteration";
  performace_msg.layout.dim.at(1).size=1;
  performace_msg.layout.dim.at(2).label="cost";
  performace_msg.layout.dim.at(2).size=1;
  performace_msg.layout.dim.at(3).label=parameter_namespace_;
  performace_msg.layout.dim.at(3).size=0;

  if (!planning_scene_)
  {
    CNR_ERROR(logger_,"No planning scene available");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    is_running_=false;
    return false;
  }



  if (display_tree_)
  {
    if (!display_)
    {
      display_=std::make_shared<graph::display::Display>(planning_scene_,group_);
      display_->clearMarkers();
    }
    else
      display_->clearMarkers();
  }

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
        CNR_ERROR(logger_,"joint %s is out of bound, actual position=%f",jm->getName().c_str(),*start_state.getJointPositions(jm->getName()));
        std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();

        for (const moveit::core::VariableBounds& b: bound)
        {
          CNR_ERROR(logger_,"joint %s has bound pos = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_);
        }
      }
      if (!start_state.satisfiesVelocityBounds(jm))
      {
        CNR_ERROR(logger_,"joint %s is out of bound, actual velocity=%f",jm->getName().c_str(),*start_state.getJointVelocities(jm->getName()));
        std::vector<moveit::core::VariableBounds> bound=jm->getVariableBounds();
        for (const moveit::core::VariableBounds& b: bound)
        {
          CNR_ERROR(logger_,"joint %s has bound pos=[%f, %f], vel = [%f, %f]",jm->getName().c_str(),b.min_position_,b.max_position_,b.min_velocity_,b.max_velocity_);
        }
      }
    }
    CNR_FATAL(logger_,"Start point is  Out of bound");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    is_running_=false;
    return false;
  }

  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);

  if (!checker_->check(start_conf))
  {
    CNR_ERROR(logger_,"Start point is in collision");

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.contacts = true;
    col_req.group_name=group_;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      CNR_ERROR(logger_,"Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        CNR_ERROR(logger_,"contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }
    else
    {
      CNR_FATAL(logger_,"you shouldn't be here!");
    }
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    is_running_=false;
    return false;
  }
  solver_->resetProblem();


  graph::core::NodePtr start_node;
  start_node=std::make_shared<graph::core::Node>(start_conf);
  if (not solver_->addStart(start_node))
  {
    CNR_ERROR(logger_,"unable to add start to solver");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_INVALID;
    is_running_=false;
    return false;
  }

  queue_.callAvailable();
  bool at_least_a_goal=false;


  // joint goal
  Eigen::VectorXd goal_configuration( start_conf.size() );
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    CNR_DEBUG(logger_,"Processing goal %u",iGoal);

    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);
    if (goal.joint_constraints.size()==0)
    {
      CNR_DEBUG(logger_,"Goal %u is a Cartesian goal",iGoal);

      if (goal.position_constraints.size()!=1 || goal.orientation_constraints.size()!=1)
      {
        CNR_DEBUG(logger_,"Goal %u has no position or orientation",iGoal);
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
      CNR_DEBUG(logger_,"Goal %u is a joint goal",iGoal);

      moveit::core::RobotState goal_state(robot_model_);

      for (auto c: goal.joint_constraints)
        goal_state.setJointPositions(c.joint_name,&c.position);
      goal_state.copyJointGroupPositions(group_,goal_configuration);

      goal_state.updateCollisionBodyTransforms();

      if (!checker_->check(goal_configuration))
      {
        CNR_DEBUG(logger_,"goal %u is in collision",iGoal);

        if (request_.goal_constraints.size()<5)
        {

          if (!goal_state.satisfiesBounds())
          {
            CNR_INFO(logger_,"End state: " << goal_configuration.transpose()<<" is  Out of bound");
          }

          collision_detection::CollisionRequest col_req;
          collision_detection::CollisionResult col_res;
          col_req.contacts = true;
          col_req.group_name=group_;
          planning_scene_->checkCollision(col_req,col_res,goal_state);
          if (col_res.collision)
          {
            CNR_INFO(logger_,"End state: " << goal_configuration.transpose()<<" is colliding");
            for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
            {
              CNR_INFO(logger_,"contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
            }
          }
        }
        continue;
      }

      graph::core::NodePtr goal_node=std::make_shared<graph::core::Node>(goal_configuration);

      if (not solver_->addGoal(goal_node))
      {
        CNR_WARN(logger_,"unable to add goal to solver");
      }

      at_least_a_goal=true;
    }
  }


  if (!at_least_a_goal)
  {
    CNR_ERROR(logger_,"All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    is_running_=false;
    return false;
  }

  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader_.createInstance<graph::core::SamplerBasePlugin>(sampler_name_);
  sampler_plugin->init(parameter_namespace_,
                       start_conf,
                       goal_configuration,
                       lower_bounds_,
                       upper_bounds_,
                       scale_,
                       logger_,
                       solver_->getCost());
  sampler_ = sampler_plugin->getSampler();
  solver_->setSampler(sampler_);

  solver_->finalizeProblem();

  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================


  // searching initial solutions
  graph::core::PathPtr solution;
  bool found_a_solution=false;
  unsigned int iteration=0;

  graph_time_point display_time = graph_time::now();
  while((graph_time::now()-start_time)<max_planning_time)
  {

    performace_msg.data.push_back(toSeconds(graph_time::now(),start_time) );
    performace_msg.data.push_back(iteration);
    performace_msg.data.push_back(solver_->getCost());

    if (display_tree_)
    {
      if ((graph_time::now()-display_time)>display_tree_period_)
      {
        display_time = graph_time::now();
        display_->displayTree(solver_->getStartTree());
      }
    }
    iteration++;
    if (stop_)
    {
      CNR_INFO(logger_,"Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      is_running_=false;
      return false;
    }

    solver_->update(solution);
    if (!found_a_solution && solver_->solved())
    {
      assert(solution);

      CNR_INFO(logger_,"Find a first solution (cost=%f) in %f seconds",solver_->cost(),toSeconds(graph_time::now(),start_time));
      CNR_DEBUG(logger_,*solver_);
      found_a_solution=true;
      refine_time = graph_time::now();
    }
    if (not solver_->canImprove())
    {
      CNR_INFO(logger_,"Optimization completed (cost=%f) in %f seconds (%u iterations)",solver_->cost(),toSeconds(graph_time::now(),start_time),iteration);
      break;
    }

    if (found_a_solution && ((graph_time::now()-refine_time)>max_refining_time_))
    {
      CNR_INFO(logger_,"Refine time expired (cost=%f) in %f seconds (%u iterations)",solver_->cost(),toSeconds(graph_time::now(),start_time),iteration);
      break;
    }
  }


  solver_performance_.publish( performace_msg);

  if (!found_a_solution)
  {
    CNR_ERROR(logger_,"unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    is_running_=false;
    if (display_tree_)
      display_->displayTree(solver_->getStartTree());
    return false;
  }
  if (display_flag_)
    display_->displayPath(solution);

  if (solver_->canImprove())
  {
    CNR_INFO(logger_,"Stopped (cost=%f) after %f seconds (%u iterations)",solver_->cost(),toSeconds(graph_time::now(),start_time),iteration);
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

  res.processing_time_.push_back(toSeconds(graph_time::now(), start_time));
  res.description_.emplace_back("plan");
  res.trajectory_.push_back(trj);

  res.error_code_.val=moveit_msgs::MoveItErrorCodes::SUCCESS;
  is_running_=false;

  return true;
}

bool GraphPlanner::solve ( planning_interface::MotionPlanResponse& res )
{
  graph_time_point start_time = graph_time::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.at(0);
  }
  res.planning_time_ = toSeconds(graph_time::now(),start_time);
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool GraphPlanner::terminate()
{
  stop_=true;
  graph_time_point t0=graph_time::now();
  graph_duration period(20ms);
  graph_duration timeout(5s);
  while (ros::ok())
  {
    if (!is_running_)
      return true;
    if ((graph_time::now()-t0)>timeout)
    {
      CNR_ERROR(logger_,"Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
    std::this_thread::sleep_for(period);
  }
  return false;
}

void GraphPlanner::humanPoseCb(const geometry_msgs::PoseArrayConstPtr& msg)
{

  if (!hamp_)
    return;



  Eigen::Matrix<double,3,-1> human_positions(3,msg->poses.size());

  for (size_t ic=0;ic<msg->poses.size();ic++)
  {
    const geometry_msgs::Pose& p=msg->poses.at(ic);
    human_positions(0,ic)=p.position.x;
    human_positions(1,ic)=p.position.y;
    human_positions(2,ic)=p.position.z;
    if (hamp_metrics_)
      hamp_metrics_->setHumanPositions(human_positions);
    if (hamp_goal_cost_fcn_)
      hamp_goal_cost_fcn_->setHumanPositions(human_positions);
  }
}


}  // namespace dirrt_star
}  // namespace graph::core
