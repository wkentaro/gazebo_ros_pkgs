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

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_plugins/gazebo_ros_vacuum_gripper.h>


namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVacuumGripper::GazeboRosVacuumGripper()
{
  gripper_state_connect_count_ = 0;
  gripper_pose_connect_count_ = 0;
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

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Parameters for Vacuum Mechanism
  // offset from the link
  if (_sdf->HasElement("xyzOffset")) {
    offset_.pos = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  } else {
    offset_.pos = math::Vector3(0, 0, 0);
  }
  if (_sdf->HasElement("rpyOffset")) {
    offset_.rot = _sdf->GetElement("rpyOffset")->Get<math::Vector3>();
  } else {
    offset_.rot = math::Vector3(0, 0, 0);
  }
  // max force: [N]
  if (_sdf->HasElement("maxForce")) {
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  } else {
    max_force_ = 20;
  }
  // max distance to apply force: [m]
  if (_sdf->HasElement("maxDistance")) {
    max_distance_ = _sdf->GetElement("maxDistance")->Get<double>();
  } else {
    max_distance_ = 0.05;
  }
  // distance to apply friction: [m]
  if (_sdf->HasElement("minDistance")) {
    min_distance_ = _sdf->GetElement("minDistance")->Get<double>();
  } else {
    min_distance_ = 0.01;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue for gripper state
  ros::AdvertiseOptions gripper_state_ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&GazeboRosVacuumGripper::GripperStateConnect, this),
    boost::bind(&GazeboRosVacuumGripper::GripperStateDisconnect, this),
    ros::VoidPtr(), &queue_);
  pub_gripper_state_ = rosnode_->advertise(gripper_state_ao);

  // Custom Callback Queue for gripper pose
  ros::AdvertiseOptions gripper_pose_ao = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>(
    "gripper_pose", 1,
    boost::bind(&GazeboRosVacuumGripper::GripperPoseConnect, this),
    boost::bind(&GazeboRosVacuumGripper::GripperPoseDisconnect, this),
    ros::VoidPtr(), &queue_);
  pub_gripper_pose_ = rosnode_->advertise(gripper_pose_ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "on", boost::bind(&GazeboRosVacuumGripper::OnServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "off", boost::bind(&GazeboRosVacuumGripper::OffServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVacuumGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVacuumGripper::UpdateChild, this));

  ROS_INFO("Loaded gazebo_ros_vacuum_gripper");
}

bool GazeboRosVacuumGripper::OnServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    ROS_WARN("gazebo_ros_vacuum_gripper: already status is 'on'");
  } else {
    status_ = true;
    ROS_INFO("gazebo_ros_vacuum_gripper: status: off -> on");
  }
  return true;
}
bool GazeboRosVacuumGripper::OffServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO("gazebo_ros_vacuum_gripper: status: on -> off");
  } else {
    ROS_WARN("gazebo_ros_vacuum_gripper: already status is 'off'");
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVacuumGripper::UpdateChild()
{
  lock_.lock();

  // compute gripper pose and publish
  math::Pose parent_pose = link_->GetWorldPose();
  ROS_ERROR_STREAM("pose: " << parent_pose << ", offset: " << offset_);
  math::Pose offset_world_pose;
  offset_world_pose.pos = offset_.pos;
  offset_world_pose.rot = offset_.rot;
  offset_world_pose.pos = parent_pose.rot.RotateVectorReverse(offset_world_pose.pos);
  offset_world_pose.rot = offset_world_pose.rot * parent_pose.rot.GetInverse();
  parent_pose.pos = parent_pose.pos + offset_world_pose.pos;
  parent_pose.rot = offset_world_pose.rot * parent_pose.rot;
  parent_pose.rot.Normalize();
  ROS_ERROR_STREAM("pose with offset: " << parent_pose);
  common::Time cur_time = world_->GetSimTime();
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp.sec = cur_time.sec;
  pose_msg.header.stamp.nsec = cur_time.nsec;
  pose_msg.pose.position.x = parent_pose.pos.x;
  pose_msg.pose.position.y = parent_pose.pos.y;
  pose_msg.pose.position.z = parent_pose.pos.z;
  pose_msg.pose.orientation.x = parent_pose.rot.x;
  pose_msg.pose.orientation.y = parent_pose.rot.y;
  pose_msg.pose.orientation.z = parent_pose.rot.z;
  pose_msg.pose.orientation.w = parent_pose.rot.w;
  pub_gripper_pose_.publish(pose_msg);

  // check gripper status
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
    pub_gripper_state_.publish(grasping_msg);
    lock_.unlock();
    return;
  }

  // apply force
  physics::Model_V models = world_->GetModels();
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName())
    {
      continue;
    }
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
      math::Pose link_pose = links[j]->GetWorldPose();
      math::Pose diff = parent_pose - link_pose;
      double norm = diff.pos.GetLength();
      if (norm <= max_distance_) {
        links[j]->SetLinearAccel(link_->GetWorldLinearAccel());
        links[j]->SetAngularAccel(link_->GetWorldAngularAccel());
        links[j]->SetLinearVel(link_->GetWorldLinearVel());
        links[j]->SetAngularVel(link_->GetWorldAngularVel());
        if (norm <= min_distance_) {
          // apply friction like force
          // TODO(unknown): should apply friction actually
          link_pose.Set(parent_pose.pos, link_pose.rot);
          links[j]->SetWorldPose(link_pose);
        } else {
          double norm_force = 1 / norm;
          if (norm_force > max_force_) {
            norm_force = max_force_;
          }
          math::Vector3 force = norm_force * diff.pos.Normalize();
          links[j]->AddForce(force);
        }
        grasping_msg.data = true;
      }
    }
  }
  pub_gripper_state_.publish(grasping_msg);
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

////////////////////////////////////////////////////////////////////////////////
// For gripper state
void GazeboRosVacuumGripper::GripperStateConnect()
{
  gripper_state_connect_count_++;
}
void GazeboRosVacuumGripper::GripperStateDisconnect()
{
  gripper_state_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// For gripper pose
void GazeboRosVacuumGripper::GripperPoseConnect()
{
  gripper_pose_connect_count_++;
}
void GazeboRosVacuumGripper::GripperPoseDisconnect()
{
  gripper_pose_connect_count_--;
}

}  // namespace gazebo
