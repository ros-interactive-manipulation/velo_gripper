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
	stall_threshold_ = 0.001; // 1mm threshold.
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
  assert(robot);
  node_ = n;
  robot_ = robot;
 
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

  // Init the PID controllers
  if (!position_pid_.init(ros::NodeHandle(node_, "pid/position")))	  
    return false;
  if (!velocity_pid_.init(ros::NodeHandle(node_, "pid/velocity")))	  
      return false;
  
  ros::NodeHandle pid_node(node_, "pid");
  ros::NodeHandle position_pid_node(pid_node, "position");
  ros::NodeHandle velocity_pid_node(pid_node, "velocity");
  
  // Velocity filter coefficients for the control loop
  pid_node.param("filter_coeff", lambda_, 0.0);
  pid_node.param("v_thres", v_thres_, 0.0);
  
  // Position holding parameters for the control loop
  pid_node.getParam("position_holding/stall_timeout", stall_timeout_);
  pid_node.getParam("position_holding/stall_threshold", stall_threshold_);
  pid_node.getParam("position_holding/holding_torque", holding_torque_);
  
  pid_node.getParam("velocity/v_limit", v_limit_);
  
  pid_node.getParam("torque_limit", torque_limit_);
  
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<pr2_controllers_msgs::Pr2GripperCommand>(
    "command", 1, &Pr2LCGripperController::commandCB, this);

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
  double pos_error = joint_state_->position_ - command->position;   
  filtered_velocity_ = (1.0-lambda_)*filtered_velocity_ + lambda_*joint_state_->velocity_;  // TODO: FILTER VELOCITY HERE.
  double pos_effort = position_pid_.updatePid(pos_error, filtered_velocity_, dt);// Sets the effort - in this case desired velocity.
  
  // Apply the velocity limit to the input to the velocity controller
  if (v_limit_ >= 0.0)
  {
    pos_effort = std::max(-v_limit_, std::min(pos_effort, v_limit_));
  }  

  // Compute the velocity error
  double vel_error = joint_state_->velocity_ - pos_effort; 
  double vel_effort = velocity_pid_.updatePid(vel_error, dt);
     
  // Check for stall. If the gripper position hasn't moved by less than a threshold for at greater than some timeout, limit the output to a holding torque.
  double delta_position = joint_state_->position_ - stall_start_position_; 
  if ( fabs(delta_position) < stall_threshold_ && command->position == last_setpoint_ && command->max_effort == last_max_effort_)
  {
	  // Reset the stall check if the position or max effort changes ... ie, a new command was sent.
	  ros::Duration stall_length = time - stall_start_time_;
	  if (stall_length.toSec() > stall_timeout_ && vel_effort > holding_torque_) // Don't increase the torque if the controller is requesting a lower torque.
	  {
		  vel_effort = holding_torque_;
	  }
  }
  else // if we're not stalled, update the stored copy of the current position + time parameters.
  {
	  stall_start_position_ = joint_state_->position_;
	  stall_start_time_ = time;
  }
 
  // Limit the effort to the specified bounds from the user.
  if (command->max_effort >= 0.0)
  {
    vel_effort = std::max(-command->max_effort, std::min(vel_effort, command->max_effort));
  }
  
  // Final bounds check. Don't let the torque exceed the fixed torque_limit (from the parameter server). This is a safety check.
  if (torque_limit_ >= 0.0)
  {
    vel_effort = std::max(-torque_limit_, std::min(vel_effort, torque_limit_));
  }
  
  // Set the motor torque.
  joint_state_->commanded_effort_ = vel_effort;

 
  
  
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
      controller_state_publisher_->msg_.command = pos_effort;

      double dummy;
      position_pid_.getGains(controller_state_publisher_->msg_.p,
                    controller_state_publisher_->msg_.i,
                    controller_state_publisher_->msg_.d,
                    controller_state_publisher_->msg_.i_clamp,
                    dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  // Update stall checking parameters
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
