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

#include "gripper_control/lcgripper_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"

using namespace std;
using namespace controller;

PLUGINLIB_DECLARE_CLASS(gripper_control, LCGripperCalibrationController,
                        controller::LCGripperCalibrationController, pr2_controller_interface::Controller)

namespace controller
{

LCGripperCalibrationController::LCGripperCalibrationController()
  : next_publish_time_(0), joint_(NULL)
{
}

LCGripperCalibrationController::~LCGripperCalibrationController()
{
}

bool LCGripperCalibrationController::init(pr2_mechanism_model::RobotState *robot,
                                        ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;
  node_ = n;

  node_.param("stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.0001);

  XmlRpc::XmlRpcValue other_joint_names;
  if (node_.getParam("other_joints", other_joint_names))
  {
    if (other_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("\"other_joints\" was not an array (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    else
    {
      for (int i = 0; i < other_joint_names.size(); ++i)
      {
        pr2_mechanism_model::JointState *j;
        std::string name = (std::string)other_joint_names[i];
        if ((j = robot->getJointState(name))){
          other_joints_.push_back(j);
        }
        else {
          ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
                    name.c_str(), node_.getNamespace().c_str());
          return false;
        }
      }
    }
  }

  if (!node_.getParam("velocity", search_velocity_))
  {
    ROS_ERROR("No velocity given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  std::string joint_name;
  if (!node_.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  std::string actuator_name;
  if (!node_.getParam("actuator", actuator_name))
  {
    ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  /*****
  if (actuator_->state_.zero_offset_ != 0){
    ROS_INFO("Joint %s is already calibrated at offset %f", joint_name.c_str(), actuator_->state_.zero_offset_);
    joint_->calibrated_ = true;
    for (size_t i = 0; i < other_joints_.size(); ++i)
      other_joints_[i]->calibrated_ = true;
    state_ = CALIBRATED;
  }
  */
  else{
    ROS_INFO("Joint '%s' will always (re)calibrate when loaded.", joint_name.c_str());
    state_ = INITIALIZED;
    joint_->calibrated_ = false;
  }



  if (!vc_.init(robot, node_))
    return false;

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &LCGripperCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}


void LCGripperCalibrationController::starting()
{
  state_ = INITIALIZED;
  actuator_->state_.zero_offset_ = 0.0;
  joint_->calibrated_ = false;
}


bool LCGripperCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
						pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}


void LCGripperCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  // Always
  if ( !joint_->calibrated_ && fabs(joint_->velocity_) < this->stopped_velocity_tolerance_)
    stop_count_++;
  else
    stop_count_ = 0;

  switch (state_)
  {
  case INITIALIZED:
    state_ = BEGINNING;
    return;

  case BEGINNING:
    close_count_ = 0;
    stop_count_ = 0;

    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;

    vc_.setCommand( -0.020 ); // More than full travel
    state_ = CLOSING;
    break;

  case CLOSING:
    // Makes sure the gripper is stopped for a while before cal
    if (stop_count_ > 250)
    {
      stop_count_ = 0;
      // ALWAYS RESET THE ACTUATOR TO ZERO AT THE BOTTOM
      actuator_->state_.zero_offset_ = actuator_->state_.position_;

      close_count_++;
      if ( close_count_ < 2 )
      {
        // BACK OFF A TIME OR TWO TO MAKE SURE WE ARE AT THE END OF TRAVEL
        vc_.setCommand( 0.003 ); // Bump out from our new zero
        state_ = BACK_OFF;
      }
      else
      {
        // FOUND THE BOTTOM OF TRAVEL
        vc_.setCommand( 0.0135 ); // Gripper installation position
        state_ = HOME;
      }
    }

    break;

  case BACK_OFF: // Back off so we can reset from a known good position
    if (stop_count_ > 500)
    {
      stop_count_ = 0;
      vc_.setCommand( -0.020 ); // More than full travel
      state_ = CLOSING;
    }
    break;

  case HOME:
    if (stop_count_ > 1000)
    {
      if ( joint_->position_ < 0.009)
      {
        ROS_WARN("Gripper NOT installed properly!  Please reinstall and recalibrate.  (pos=%6.4fm)",joint_->position_);
      }
      else if ( joint_->position_ > 0.014)
      {
        ROS_WARN("Gripper NOT installed!  Please install and recalibrate.  (pos=%6.4fm)",joint_->position_);
      }

      joint_->calibrated_ = true;
      for (size_t i = 0; i < other_joints_.size(); ++i)
        other_joints_[i]->calibrated_ = true;
      state_ = CALIBRATED;
    }
    break;

  case CALIBRATED:
    if ( pub_calibrated_ && next_publish_time_ > robot_->getTime() )
    {
      if (pub_calibrated_->trylock())
      {
        next_publish_time_ = robot_->getTime() + ros::Duration(0.5);
        pub_calibrated_->unlockAndPublish();
      }
    }
    break;
  }

  // RUN THE CONTROLLER UPDATE
  if (state_ != CALIBRATED)
    vc_.update();

}
} // namespace
