/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_VACUUM_GRIPPER_HH
#define GAZEBO_ROS_VACUUM_GRIPPER_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosVacuumGripper Plugin XML Reference and Example

  \brief Ros Vacuum Gripper Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_vacuum_gripper.so" name="gazebo_ros_vacuum_gripper">
          <bodyName>box_body</bodyName>
          <serviceName>box_force</serviceName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/


class GazeboRosVacuumGripper : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosVacuumGripper();

  /// \brief Destructor
  public: virtual ~GazeboRosVacuumGripper();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  private: bool ServiceCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res);

  private: bool status_;

  private: physics::ModelPtr parent_;

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;
  private: ros::ServiceServer srv_;

  /// \brief ROS Wrench topic name inputs
  private: std::string service_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
};
/** \} */
/// @}
}
#endif
