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
  std::string joint_name, actuator_name;

  assert(robot);
  robot_ = robot;
  node_ = n;

  getNodeParam<std::string>("joint", joint_name);
  getNodeParam<std::string>("actuator", actuator_name);
  getNodeParam<double>("velocity", search_velocity_);
  getNodeParam<double>("stopped_velocity_tolerance", stopped_velocity_tolerance_);
//  getNodeParam<double>("error_max", error_max_); // NOT NEEDED HERE. PICKED UP IN vc_ (THE CAPPED CONTROLLER).

  if (!(joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue other_joint_names;
  if ( node_.getParam("other_joints", other_joint_names) )
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

  /*****
  if (actuator_->state_.zero_offset_ != 0){
    ROS_INFO("Joint %s is already calibrated at offset %f", joint_name.c_str(), actuator_->state_.zero_offset_);
    joint_->calibrated_ = true;
    for (size_t i = 0; i < other_joints_.size(); ++i)
      other_joints_[i]->calibrated_ = true;
    state_ = CALIBRATED;
  }
  */
  if ( false ) {}
  else{
    ROS_INFO("Joint '%s' will always (re)calibrate when loaded.", joint_name.c_str());
    state_ = INITIALIZED;
    joint_->calibrated_ = false;
  }



  if (!(vc_.init(robot, node_)))
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
  s0_ = state_;

  ROS_INFO("%d : INITIALIZED", INITIALIZED);
  ROS_INFO("%d : STARTING", STARTING);
  ROS_INFO("%d : CLOSING", CLOSING);
  ROS_INFO("%d : BACK_OFF", BACK_OFF);
  ROS_INFO("%d : HOME", HOME);
  ROS_INFO("%d : CALIBRATED", CALIBRATED);
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

  // DEBUG STATE TRANSITIONS
  if ( s0_ != state_ || (stop_count_ && !(stop_count_ % 100)) )
  { ROS_WARN("STATE: %d --> %d    (c=%d)  jvel=%.4lf",s0_,state_,stop_count_,joint_->velocity_);
    s0_=state_;
  }

  // Always
  if ( !(joint_->calibrated_) && fabs(joint_->velocity_) < stopped_velocity_tolerance_)
    stop_count_++;
  else
    stop_count_=0;

  double LCGCC_mttop   =  0.0165;
  double LCGCC_empty   =  0.0140;
  double LCGCC_install =  0.0135;
  double LCGCC_wrong   =  0.0090;
  double LCGCC_backoff =  0.0030;
  double LCGCC_mtclosed= -0.0040;
  double LCGCC_mtbottom= -0.0200;

  switch (state_)
  {
  case INITIALIZED:
    state_ = STARTING;
    break;

  case STARTING:
    close_count_ = 0;
    stop_count_ = 0;

    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;

    vc_.setCommand( LCGCC_mtbottom ); // More than full travel
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
        ROS_INFO("FOUND Bottom, now heading to BACKOFF");
        vc_.setCommand( LCGCC_backoff ); // Bump out from our new zero
        state_ = BACK_OFF;
      }
      else
      {
        // FOUND THE BOTTOM OF TRAVEL
        ROS_INFO("FOUND Bottom, now heading to TOP");
        vc_.setCommand( LCGCC_mttop ); // Gripper installation/fully-open position.
        state_ = TOP;
      }
    }
    break;

  case BACK_OFF: // Back off so we can reset from a known good position
    if (stop_count_ > 400)
    {
      stop_count_ = 0;
      vc_.setCommand( LCGCC_mtclosed );
      state_ = CLOSING;
    }
    break;

  case TOP:
    /* PUSHING THE BALLSCREW ALL THE WAY OUT FROM CLOSED TELLS US THAT WE HAVE A GRIPPER INSTALLED CORRECTLY */
    if (stop_count_ > 600)
    {
      if ( joint_->position_ < LCGCC_wrong )
      {
        ROS_WARN("Gripper NOT installed properly!  Please reinstall and recalibrate.  (pos=%6.4fm)",joint_->position_);
        vc_.setCommand( LCGCC_backoff ); // Go to a safe place, not at either end.

      }
      else if ( joint_->position_ > LCGCC_empty )
      {
        ROS_WARN("Gripper NOT installed!  Please install and recalibrate.  (pos=%6.4fm)",joint_->position_);
        vc_.setCommand( LCGCC_install ); // Gripper installation/fully-open position.
      }
      state_ = HOME;
    }
    break;

  case HOME:
    if (stop_count_ > 400)
    {
      stop_count_ = 0;
      joint_->calibrated_ = true;
      for (size_t i = 0; i < other_joints_.size(); ++i)
        other_joints_[i]->calibrated_ = true;
      vc_.setGains(0.0,0.0,0.0,0.0,0.0);
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
