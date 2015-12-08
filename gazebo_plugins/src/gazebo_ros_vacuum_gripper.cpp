/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboVacuumGripper plugin for manipulating objects in Gazebo
   Author: Kentaro Wada
   Date: 7 Dec 2015
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_vacuum_gripper.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVacuumGripper::GazeboRosVacuumGripper()
{
  status_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVacuumGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("Loading gazebo_ros_vacuum_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("vacuum_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_);
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL("gazebo_ros_vacuum_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("serviceName"))
  {
    ROS_FATAL("vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    service_name_ = _sdf->GetElement("serviceName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    service_name_, boost::bind(&GazeboRosVacuumGripper::ServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv_ = rosnode_->advertiseService(aso);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVacuumGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVacuumGripper::UpdateChild, this));

  ROS_INFO("Loaded gazebo_ros_vacuum_gripper");
}

bool GazeboRosVacuumGripper::ServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
  } else {
    status_ = true;
  }
  ROS_INFO("status: %s", status_ ? "true" : "false");
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVacuumGripper::UpdateChild()
{
  if (!status_) {
    return;
  }
  // apply force
  lock_.lock();
  math::Pose parent_pose = link_->GetWorldPose();
  physics::Model_V models = world_->GetModels();
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName())
    {
      continue;
    }
    bool printed = false;
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
      math::Pose link_pose = links[j]->GetWorldPose();
      math::Pose diff = parent_pose - link_pose;
      double norm = diff.pos.GetLength();
      if (norm < 0.3) {
        links[j]->SetLinearAccel(link_->GetWorldLinearAccel());
        links[j]->SetAngularAccel(link_->GetWorldAngularAccel());
        links[j]->SetLinearVel(link_->GetWorldLinearVel());
        links[j]->SetAngularVel(link_->GetWorldAngularVel());
        double norm_force = 5 / norm;
        if (!printed) {
          ROS_INFO_STREAM("Inhaling " << models[i]->GetName() << " with force " << norm_force);
          printed = true;
        }
        if (norm_force > 10) {
          norm_force = 10;  // max_force
        }
        math::Vector3 force = norm_force * diff.pos.Normalize();
        links[j]->AddForce(force);
      }
    }
  }
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVacuumGripper::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
