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


#include "gripper_control/lcgripper_pid.h"
#include <tinyxml.h>

namespace lcg_controller {

LCGPid::LCGPid(double P, double I, double D, double I1, double I2)
{
  setGains(P, I, D, I1, I2);
  reset();
}

LCGPid::~LCGPid()
{
}


void LCGPid::reset()
{
  Pid::reset();
  v_thres_ = 0.0;
}


bool LCGPid::init(const ros::NodeHandle &node)
{
  Pid::init(node);
  
  ros::NodeHandle n(node);
  n.param("v_thres", v_thres_, 0.0);
  return true;
}


double LCGPid::updatePid(double error, double error_dot, ros::Duration dt)
{

	double p_gain, i_gain, d_gain, i_min, i_max; 
	double cmd;
	getGains(p_gain, i_gain, d_gain, i_max, i_min);
			
	double p_term, d_term, i_term;
	proportional_error_ = error; //this is pError = pState-pTarget
	derivative_error_ = error_dot;
	
	
	
	if (dt == ros::Duration(0.0) || isnan(error) || isinf(error) || isnan(error_dot) || isinf(error_dot))
	return 0.0;
	
	
	// Calculate proportional contribution to command
	p_term = p_gain * proportional_error_;
	
	// Calculate the integral error
	if (fabs(error_dot) > v_thres_)
	{
	integral_error_ = 0.0;  
	} 
	else // If the gripper is stationary (below a certain threshold) and not yet at the destination, allow the integral error to build up
	{
	  integral_error_ = integral_error_ + dt.toSec() * proportional_error_;
	}
	
	//Calculate integral contribution to command
	i_term = i_gain * integral_error_;
	
	// Limit i_term so that the limit is meaningful in the output
	if (i_term > i_max)
	{
	i_term = i_max;
	integral_error_=i_term/i_gain;
	}
	else if (i_term < i_min)
	{
	i_term = i_min;
	integral_error_=i_term/i_gain;
	}
	
	// Calculate derivative contribution to command
	d_term = d_gain * derivative_error_;
	cmd = -p_term - i_term - d_term;
	
	return cmd;
}

}

