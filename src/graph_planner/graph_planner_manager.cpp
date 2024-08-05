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

#include <graph_planner/graph_planner_manager.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(graph::planner::GraphPlannerManager, planning_interface::PlannerManager)

namespace graph {
namespace planner {

XmlRpc::XmlRpcValue inheritConfig(ros::NodeHandle& nh,const std::string& config_name)
{
  XmlRpc::XmlRpcValue config;
  if (not  nh.getParam(config_name,config))
  {
    throw std::invalid_argument(config_name+" does not exist");
  }

  if (not config.hasMember("inherit_from"))
    return config;

  XmlRpc::XmlRpcValue parent_config=inheritConfig(nh,config["inherit_from"]);

  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=parent_config.begin();it!=parent_config.end();it++)
  {
    std::string key=it->first;
    if (not config.hasMember(key))
    {
      config[key]=it->second;
    }
  }

  nh.setParam(config_name,config);

  return config;
}

bool GraphPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{

  std::string package_name="graph_planner";
  std::string package_path = ros::package::getPath(package_name);
  std::string logger_file="config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr tmp_logger = std::make_shared<cnr_logger::TraceLogger>("graph_planner",package_path+"/"+logger_file);

  CNR_INFO(tmp_logger,"Creating logger for this scene");

  parameter_namespace_=ns;

  if(not graph::core::get_param(tmp_logger,parameter_namespace_,"logger_param_file",logger_file))
  {
    CNR_ERROR(tmp_logger,"logger_param_file config not set");
    return false;
  }

  if(not graph::core::get_param(tmp_logger,parameter_namespace_,"package_name",package_name))
  {
    CNR_ERROR(tmp_logger,"package_name config not set");
    return false;
  }

  package_path = ros::package::getPath(package_name);

  CNR_INFO(tmp_logger,"creating logger " << package_name << " using config from "<<package_path+"/"+logger_file);
  logger_ = std::make_shared<cnr_logger::TraceLogger>(package_name,package_path+"/"+logger_file);
  CNR_DEBUG(logger_,"starting planners creation");


  m_nh=ros::NodeHandle(ns);



  std::map<std::string,std::string> planner_map;

  if(not graph::core::get_param(logger_,parameter_namespace_,"group_names_map",planner_map))
  {
    CNR_WARN(logger_,"group_names_map planning config not set");
    return false;
  }

  m_nh.getParam("group_names_map",planner_map);

  CNR_INFO(logger_,"creating " << planner_map.size() << " planners: ");

  for (std::pair<std::string,std::string> p: planner_map)
  {

    //inheritConfig(m_nh,p.first);

    CNR_INFO(logger_,"creating planner " << p.first << " for gruop "<<p.second << " usign configuration " << parameter_namespace_+"/"+p.first);


    std::shared_ptr<GraphPlanner> ptr;
    ptr= std::make_shared<GraphPlanner>(parameter_namespace_+"/"+p.first,p.second,model,logger_);
    if (ptr->init())
    {
      CNR_INFO(logger_,"Planned Id=%s on group %s",p.first.c_str(),p.second.c_str());
      m_planners.insert(std::pair<std::string, std::shared_ptr<GraphPlanner>>(p.first,ptr));
    }
    else
    {
      CNR_ERROR(logger_,"Planned Id=%s on group %s fails during initialization, skip it",p.first.c_str(),p.second.c_str());
    }
  }

  if (m_planners.size()==0)
  {
    CNR_ERROR(logger_,"no planner availables");
    return false;
  }

  if(not graph::core::get_param(logger_,parameter_namespace_,"default_planner_config",m_default_planner_config))
  {
    m_default_planner_config=planner_map.begin()->first;
    CNR_WARN(logger_,"default planning config not set, using " << m_default_planner_config);
    return false;
  }
  return true;
}

bool GraphPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  bool ok=false;
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
  {
    if (!planner.second->getGroupName().compare(req.group_name))
    {
      ROS_INFO("Planned Id=%s can be used for group %s",planner.first.c_str(),req.group_name.c_str());
      ok=true;
    }
  }
  return ok;
}


planning_interface::PlanningContextPtr GraphPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr &planning_scene,
  const planning_interface::MotionPlanRequest &req,
  moveit_msgs::MoveItErrorCodes &error_code) const
{
  bool any_planner=false;
  std::string planner_id;
  if (req.planner_id.size()==0)
  {
    if (m_default_planner_config.size()==0)
    {
      ROS_DEBUG("Search a planner for group %s",req.group_name.c_str());
      any_planner=true;
    }
    else
    {
      ROS_DEBUG("Use default planner %s",m_default_planner_config.c_str());
      planner_id=m_default_planner_config;
    }
  }
  else
  {
    ROS_DEBUG("Search planner %s for group %s",req.planner_id.c_str(),req.group_name.c_str());
    planner_id=req.planner_id;
  }

  bool ok=false;
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
  {
    ROS_DEBUG("planner %s for group %s",planner.first.c_str(),planner.second->getGroupName().c_str());
    if (!planner.second->getGroupName().compare(req.group_name))
    {
      if (any_planner)
      {
        planner_id=planner.first;
        ok=true;
        break;
      }
      else if (!planner.first.compare(planner_id))
      {
        ROS_DEBUG("Planned Id=%s can be used for group %s",planner.first.c_str(),req.group_name.c_str());
        ok=true;
      }
    }
  }
  if (!ok)
  {
    ROS_ERROR("Planner %s not found for group %s.", planner_id.c_str(), req.group_name.c_str());
    ROS_ERROR("Available planners are:\n");
    for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
      ROS_ERROR("- planner %s, group  %s",planner.first.c_str(),planner.second->getGroupName().c_str());
    return nullptr;
  }

  std::shared_ptr<planning_interface::PlanningContext> planner = m_planners.at(planner_id);
  if (!planner)
  {
    ROS_ERROR("Planner not found");
    return planner;
  }
  else
  {
    ROS_INFO("Using  planner %s for planning on the group %s",planner->getName().c_str(),req.group_name.c_str());
  }

  planner->setPlanningScene(planning_scene);
  planner->setMotionPlanRequest(req);
  return planner;
}


void GraphPlannerManager::getPlanningAlgorithms ( std::vector< std::string >& algs ) const
{
  for (std::pair<std::string, std::shared_ptr<planning_interface::PlanningContext>> planner: m_planners)
    algs.push_back(planner.first);
}

void GraphPlannerManager::setPlannerConfigurations ( const planning_interface::PlannerConfigurationMap& pcs )
{
  planning_interface::PlannerManager::setPlannerConfigurations ( pcs );
}



}
}
