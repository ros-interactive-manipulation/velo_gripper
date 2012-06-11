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

// Original version: Melonee Wise <mwise@willowgarage.com>

#include "gripper_control/lcgripper_pid.h"
#include <tinyxml.h>

namespace lcg_controller {

LCGPid::LCGPidPid(double P, double I, double D, double I1, double I2) :
  p_gain_(P), i_gain_(I), d_gain_(D), i_max_(I1), i_min_(I2)
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  cmd_ = 0.0;
}

LCGPid::~LCGPid()
{
}


void LCGPid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  cmd_ = 0.0;
  v_thres_ = 0.0;
}


bool LCGPid::init(const ros::NodeHandle &node)
{
  ros::NodeHandle n(node);
  if (!n.getParam("p", p_gain_)) {
    ROS_ERROR("No p gain specified for pid.  Namespace: %s", n.getNamespace().c_str());
    return false;
  }
  n.param("i", i_gain_, 0.0);
  n.param("d", d_gain_, 0.0);
  n.param("i_clamp", i_max_, 0.0);
  n.param("v_thres", v_thres_, 0.0);
  i_min_ = -i_max_;

  reset();
  return true;
}


double LCGPid::updatePid(double error, double error_dot, ros::Duration dt)
{
  double p_term, d_term, i_term;
  p_error_ = error; //this is pError = pState-pTarget
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || isnan(error) || isinf(error) || isnan(error_dot) || isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  // Calculate the integral error
  if (fabs(error_dot) > v_thres_)
  {
	i_error_ = 0.0;  
  } 
  else // If the gripper is stationary (below a certain threshold) and not yet at the destination, allow the integral error to build up
  {
	  i_error_ = i_error_ + dt.toSec() * p_error_;
  }

  //Calculate integral contribution to command
  i_term = i_gain_ * i_error_;

  // Limit i_term so that the limit is meaningful in the output
  if (i_term > i_max_)
  {
    i_term = i_max_;
    i_error_=i_term/i_gain_;
  }
  else if (i_term < i_min_)
  {
    i_term = i_min_;
    i_error_=i_term/i_gain_;
  }

  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = -p_term - i_term - d_term;

  return cmd_;
}



void LCGPid::setCurrentCmd(double cmd)
{
  cmd_ = cmd;
}

double LCGPid::getCurrentCmd()
{
  return cmd_;
}

void LCGPid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = p_error_;
  *ie = i_error_;
  *de = d_error_;
}

}

