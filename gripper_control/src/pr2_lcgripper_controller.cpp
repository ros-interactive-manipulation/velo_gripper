/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "gripper_control/pr2_lcgripper_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(gripper_control, Pr2LCGripperController, controller::Pr2LCGripperController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

Pr2LCGripperController::Pr2LCGripperController()
: joint_state_(NULL),
  loop_count_(0), robot_(NULL), last_time_(0)
{
	stall_timeout_ = 3000; // 3000 iterations of update, ie, 3 seconds.
	position_threshold_ = 0.001; // 1mm threshold.
	last_setpoint_ = 0.0;
	last_max_effort_ = 0.0;
	stall_counter_ = 0.0;
}

Pr2LCGripperController::~Pr2LCGripperController()
{
  sub_command_.shutdown();
}

bool Pr2LCGripperController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_WARN("Pr2LCGripperController::init");
  assert(robot);
  node_ = n;
  robot_ = robot;
  ROS_WARN("LCGController Init!");
  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot_->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint named \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (joint_state_->joint_->type != urdf::Joint::PRISMATIC)
  {
    ROS_ERROR("The joint \"%s\" was not prismatic (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s is not calibrated (namespace: %s)",
              joint_state_->joint_->name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  ROS_WARN("Before PID INIT");
  if (!pid_.init(ros::NodeHandle(node_, "pid")))	  
    return false;
  
  ros::NodeHandle pid_node(node_, "pid");
  pid_node.param("filter_coeff", lambda_, 0.0);
  pid_node.param("v_thres", v_thres_, 0.0);

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<pr2_controllers_msgs::Pr2GripperCommand>(
    "command", 1, &Pr2LCGripperController::commandCB, this);
  ROS_WARN("Initialised LCG Controller");
  return true;
}

void Pr2LCGripperController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  ros::Time time = robot_->getTime();
  assert(joint_state_->joint_);
  ros::Duration dt = time - last_time_;

  pr2_controllers_msgs::Pr2GripperCommandConstPtr command;
  command_box_.get(command);
  assert(command);

  // Computes the position error
  error = joint_state_->position_ - command->position;
  
  
  // TODO: FILTER VELOCITY HERE.  
  filtered_velocity_ = (1.0-lambda_)*filtered_velocity_ + lambda_*joint_state_->velocity_;
  
  // Sets the effort 
   double effort = pid_.updatePid(error, filtered_velocity_, dt);
  
  // If the position has not changed since the last iteration, add to a counter.
   double delta_position = joint_state_->position_ - last_position_; 
  if ( fabs(delta_position) < position_threshold_ && command->position == last_setpoint_ && command->max_effort == last_max_effort_)
  {
	  stall_counter_++;
  }
  else
  {
	  stall_counter_ = 0;
  }
  
  // If the stall counter is over a timeout parameter, limit the control output to the holding torque.
  if (stall_counter_ > stall_timeout_)
  {
	  effort = holding_torque_;
  }

  // Limit the effort to the specified bounds.
  if (command->max_effort >= 0.0)
  {
    effort = std::max(-command->max_effort, std::min(effort, command->max_effort));
  }
  joint_state_->commanded_effort_ = effort;

  
  // Real time publisher
  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command->position;
      controller_state_publisher_->msg_.process_value = joint_state_->position_;
      controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt.toSec();
      controller_state_publisher_->msg_.command = effort;

      double dummy;
      pid_.getGains(controller_state_publisher_->msg_.p,
                    controller_state_publisher_->msg_.i,
                    controller_state_publisher_->msg_.d,
                    controller_state_publisher_->msg_.i_clamp,
                    dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
  last_position_ = joint_state_->position_;
  last_setpoint_ = command->position;
  last_max_effort_ = command->max_effort;
}

void Pr2LCGripperController::commandCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg)
{
  command_box_.set(msg);
}

}
