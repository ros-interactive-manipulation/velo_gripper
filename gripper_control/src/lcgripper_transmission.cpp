/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: J Hawke
 */

/*
 * propagatePosition (from as to js)
 *   as position and velocity are doubled to get js, since gripper is two sided
 *   as torque is directly converted to gap_effort
 *   as torque to passive joint is /4 since there are 4 links
 * propagateEffort (from js to as)
 *   popluate only as->commanded_.effort_
 *     this is directly transferred as torque and gap effort is 1to1
 *
 * below only for simulation
 *
 * propagatePositionBackwards (from js to as)
 *   as position and velocity transfers 1to1 to joint_angle (1-sided)
 *   as last_measured_effort_ should be 1to1 gap_effort of non-passive js->commanded_effort
 * propagateEffortBackwards
 *   non-passive js->commanded_effort_ is 1to1 with MT
 *   passive js->commanded_effort_ is 1/2?? of MT converted to joint torques
 */
#include "gripper_control/lcgripper_transmission.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <numeric>
#include <angles/angles.h>
#include <boost/lexical_cast.hpp>
#include <iostream>

#include <math.h>

using namespace pr2_hardware_interface;
using namespace pr2_mechanism_model;

PLUGINLIB_DECLARE_CLASS(gripper_control, LCGripperTransmission,
                         pr2_mechanism_model::LCGripperTransmission,
                         pr2_mechanism_model::Transmission)

bool LCGripperTransmission::initParameters(TiXmlElement *j, Robot *robot)
{
	length_to_gap_coeffs_.reserve(5);
	gap_to_length_coeffs_.reserve(5);
	gap_to_effective_dist_coeffs_.reserve(5);
	
	std::cout << "Init Parameters" << std::endl;
	const char *joint_name = j->Attribute("name");
	if (!joint_name)
	{
		ROS_ERROR("LCGripperTransmission did not specify joint name");
		return false;
	}
	
	if (robot)
	{
		const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
		if (!joint)
		{
			ROS_ERROR("LCGripperTransmission could not find joint named \"%s\"", joint_name);
			return false;
		}
	}
	
	gap_joint_ = std::string(joint_name);
	joint_names_.push_back(joint_name);  // The first joint is the gap joint
	
	// get the gear_ratio
	const char *gear_reduction_str = j->Attribute("gear_reduction");
	if (gear_reduction_str == NULL)
	{
		gear_reduction_ = 15.0; // Gear ratio of the motor to ball screw shaft. ie, MotorSpeed/GearRatio -> ballscrew speed.
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: gear_ratio, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gear_reduction_ = boost::lexical_cast<double>(gear_reduction_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("gear_reduction (%s) is not a float",gear_reduction_str);
			return false;
		}
	}
	
	const char *gear_efficiency_str = j->Attribute("gear_efficiency");
	if (gear_efficiency_str == NULL)
	{
		gear_efficiency_ = 1.0; 
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: gear_efficiency, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gear_efficiency_ = boost::lexical_cast<double>(gear_efficiency_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("gear_efficiency (%s) is not a float", gear_efficiency_str);
			return false;
		}
	}
	
	const char *screw_reduction_str = j->Attribute("screw_reduction");
	if (screw_reduction_str == NULL)
	{
		screw_reduction_ = 1000.0/3.25; // turns per m of travel (from 3.25mm screw lead). (Gear ratio of the ball screw, ie motor -> tendon movement)
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: screw_reduction, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			screw_reduction_ = boost::lexical_cast<double>(screw_reduction_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("screw_reduction (%s) is not a float", screw_reduction_str);
			return false;
		}
	}
	
	const char *screw_efficiency_str = j->Attribute("screw_efficiency");
	if (screw_efficiency_str == NULL)
	{
		screw_efficiency_ = 1.0; // turns per m of travel. (Gear ratio of the ball screw, ie motor -> tendon movement)
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: screw_efficiency, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			screw_efficiency_ = boost::lexical_cast<double>(screw_efficiency_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("screw_efficiency (%s) is not a float", screw_efficiency_str);
			return false;
		}
	}
	
	const char *screw_lead_str = j->Attribute("screw_lead");
	if (screw_lead_str == NULL)
	{
		screw_lead_ = 3.25/1000.0; // turns per m of travel. (Gear ratio of the ball screw, ie motor -> tendon movement)
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: screw_lead, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			screw_lead_ = boost::lexical_cast<double>(screw_lead_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("screw_lead (%s) is not a float", screw_lead_str);
			return false;
		}
	}
	
	
	// Get Joint parameters
	// get the Joint 0 x coordinate
	const char *j0x_str = j->Attribute("j0x");
	if (j0x_str == NULL)
	{
		j0x_ = -35.0; // mm
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: j0x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			j0x_ = boost::lexical_cast<double>(j0x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("J0x (%s) is not a float", j0x_str);
			return false;
		}
	}
	// get the Joint 0 y coordinate
	const char *j0y_str = j->Attribute("j0y");
	if (j0y_str == NULL)
	{
		j0y_ = 0.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: j0y, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			j0y_ = boost::lexical_cast<double>(j0y_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("phi0 (%s) is not a float",j0y_str);
			return false;
		}
	}
	// get the Joint 1 x coordinate
	const char *j1x_str = j->Attribute("j1x");
	if (j1x_str == NULL)
	{
		j1x_ = -60.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: j1x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			j1x_ = boost::lexical_cast<double>(j1x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("j1x (%s) is not a float",j1x_str);
			return false;
		}
	}
	// get the Joint 1 y coordinate
	const char *j1y_str = j->Attribute("j1y");
	if (j1y_str == NULL)
	{
		j1y_ = 0.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: j1y, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			j1y_ = boost::lexical_cast<double>(j1y_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("j1y (%s) is not a float",j1y_str);
			return false;
		}
	}
	// get the P0 x coordinate
	const char *p0x_str = j->Attribute("p0x");
	if (p0x_str == NULL)
	{
		p0x_ = -25.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p0x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p0x_ = boost::lexical_cast<double>(p0x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p0x (%s) is not a float",p0x_str);
			return false;
		}
	}
	// get the P0 y coordinate
	const char *p0y_str = j->Attribute("p0y");
	if (p0y_str == NULL)
	{
		p0y_ = 2.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p0y, using default for LCG v1.46.", joint_name);
	}
	else
	try
	{
		p0y_ = boost::lexical_cast<double>(p0y_str);
	}
	catch (boost::bad_lexical_cast &e)
	{
		ROS_ERROR("p0y (%s) is not a float",p0y_str);
		return false;
	}
	
	// get the P1 x coordinate
	const char *p1x_str = j->Attribute("p1x");
	if (p1x_str == NULL)
	{
		p1x_ = -45.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p1x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p1x_ = boost::lexical_cast<double>(p1x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p1x (%s) is not a float",p1x_str);
			return false;
		}
	}
	
	// get the P1 y coordinate
	const char *p1y_str = j->Attribute("p1y");
	if (p1y_str == NULL)
	{
		p1y_ = -0.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p1y, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p1y_ = boost::lexical_cast<double>(p1y_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p1y (%s) is not a float",p1y_str);
			return false;
		}
	}
	// get the P2 x coordinate
	const char *p2x_str = j->Attribute("p2x");
	if (p2x_str == NULL)
	{
		p2x_ = -8.4;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p2x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p2x_ = boost::lexical_cast<double>(p2x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p2x (%s) is not a float",p2x_str);
			return false;
		}
	}
	
	// get the P2 y coordinate
	const char *p2y_str = j->Attribute("p2y");
	if (p2y_str == NULL)
	{
		p2y_ = 2.3;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p2y, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p2y_ = boost::lexical_cast<double>(p2y_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p2y (%s) is not a float",p2y_str);
			return false;
		}
	}
	// get the P3 x coordinate
	const char *p3x_str = j->Attribute("p3x");
	if (p3x_str == NULL)
	{
		p3x_ = -44.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p3x, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p3x_ = boost::lexical_cast<double>(p3x_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p3x (%s) is not a float",p3x_str);
			return false;
		}
	}
	
	// get the P3 y coordinate
	const char *p3y_str = j->Attribute("p3y");
	if (p3y_str == NULL)
	{
		p3y_ = -5.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p3y, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p3y_ = boost::lexical_cast<double>(p3y_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p3y (%s) is not a float",p3y_str);
			return false;
		}
	}
	// get the L0 length coefficient (palm)
	const char *l0_str = j->Attribute("l0");
	if (l0_str == NULL)
	{
		l0_ = 35.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l0, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			l0_ = boost::lexical_cast<double>(l0_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l0 (%s) is not a float",l0_str);
			return false;
		}
	}
	
	// get the L1 length coefficient (proximal link)
	const char *l1_str = j->Attribute("l1");
	if (l1_str == NULL)
	{
		l1_ = 60.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l1, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			l1_ = boost::lexical_cast<double>(l1_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l1 (%s) is not a float",l1_str);
			return false;
		}
	}
	
	// get the L2 length coefficient (distal link)
	const char *l2_str = j->Attribute("l2");
	if (l2_str == NULL)
	{
		l2_ = 50.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			l2_ = boost::lexical_cast<double>(l2_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2 (%s) is not a float",l2_str);
			return false;
		}
	}
	
	// get the P0 radius coefficient
	const char *p0_radius_str = j->Attribute("p0_radius");
	if (p0_radius_str == NULL)
	{
		p0_radius_ = 4.0; // mm
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: p0_radius, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			p0_radius_ = boost::lexical_cast<double>(p0_radius_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("p0_radius (%s) is not a float",p0_radius_str);
			return false;
		}
	}
	
	// get the J1 radius coefficient
	const char *j1_radius_str = j->Attribute("j1_radius");
	if (j1_radius_str == NULL)
	{
		j1_radius_ = 3.2; // mm
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: j1_radius, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			j1_radius_ = boost::lexical_cast<double>(j1_radius_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("j1_radius (%s) is not a float",j1_radius_str);
			return false;
		}
	}
	// get the distal thickness coefficient (mm)
	const char *thickness_str = j->Attribute("thickness");
	if (thickness_str == NULL)
	{
		thickness_ = 6.0; // mm
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: thickness, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			thickness_ = boost::lexical_cast<double>(thickness_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("thickness (%s) is not a float",thickness_str);
			return false;
		}
	}
	
	// get the proximal angle when gripper is open (theta1 - deg)
	const char *theta_open_str = j->Attribute("theta_open");
	if (theta_open_str == NULL)
	{
		theta_open_ = 20.0; // mm
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: theta_open, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			theta_open_ = boost::lexical_cast<double>(theta_open_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("theta_open (%s) is not a float",theta_open_str);
			return false;
		}
	}
	// get the proximal angle when the gripper is fully closed (theta1 - deg)
	const char *theta_closed_str = j->Attribute("theta_closed");
	if (theta_closed_str == NULL)
	{
		theta_closed_ = 101.5;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: theta_closed, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			theta_closed_ = boost::lexical_cast<double>(theta_closed_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("theta_closed (%s) is not a float",theta_closed_str);
			return false;
		}
	}
	
	const char *gap_open_str = j->Attribute("gap_open");
	if (gap_open_str == NULL)
	{
		gap_open_ = 0.135763; //m
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: gap_open, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_open_ = boost::lexical_cast<double>(gap_open_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("gap_open (%s) is not a float", gap_open_str);
			return false;
		}
	}
	
	const char *gap_closed_str = j->Attribute("gap_closed");
	if (gap_closed_str == NULL)
	{
		gap_closed_ = 0.0;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: gap_closed, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_closed_ = boost::lexical_cast<double>(gap_closed_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("gap_closed (%s) is not a float", gap_closed_str);
			return false;
		}
	}
	
	
	// Polynomial coefficients
	// LENGTH TO GAP polynomial.
	const char *l2g_coeffs_0_str = j->Attribute("l2g_coeffs_0");
	if (l2g_coeffs_0_str == NULL)
	{
		length_to_gap_coeffs_[0] = 1.35959902e-01;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2g_coeffs_0, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			length_to_gap_coeffs_[0] = boost::lexical_cast<double>(l2g_coeffs_0_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2g_coeffs_0 (%s) is not a float",l2g_coeffs_0_str);
			return false;
		}
	}
	
	const char *l2g_coeffs_1_str = j->Attribute("l2g_coeffs_1");
	if (l2g_coeffs_1_str == NULL)
	{
		length_to_gap_coeffs_[1] = 1.10396557e+01;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2g_coeffs_1, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			length_to_gap_coeffs_[1] = boost::lexical_cast<double>(l2g_coeffs_1_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2g_coeffs_1 (%s) is not a float",l2g_coeffs_1_str);
			return false;
		}
	}
	
	const char *l2g_coeffs_2_str = j->Attribute("l2g_coeffs_2");
	if (l2g_coeffs_2_str == NULL)
	{
		length_to_gap_coeffs_[2] = -7.24526160e+02;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2g_coeffs_2, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			length_to_gap_coeffs_[2] = boost::lexical_cast<double>(l2g_coeffs_2_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2g_coeffs_2 (%s) is not a float",l2g_coeffs_2_str);
			return false;
		}
	}
	
	const char *l2g_coeffs_3_str = j->Attribute("l2g_coeffs_3");
	if (l2g_coeffs_3_str == NULL)
	{
		length_to_gap_coeffs_[3] = -7.95557419e+04;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2g_coeffs_3, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			length_to_gap_coeffs_[3] = boost::lexical_cast<double>(l2g_coeffs_3_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2g_coeffs_3 (%s) is not a float",l2g_coeffs_3_str);
			return false;
		}
	}
	
	const char *l2g_coeffs_4_str = j->Attribute("l2g_coeffs_4");
	if (l2g_coeffs_4_str == NULL)
	{
		length_to_gap_coeffs_[4] = -2.57497099e+06;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: l2g_coeffs_4, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			length_to_gap_coeffs_[4] = boost::lexical_cast<double>(l2g_coeffs_4_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("l2g_coeffs_4 (%s) is not a float",l2g_coeffs_4_str);
			return false;
		}
	}
	
	// GAP TO LENGTH Polynomial Coefficients
	const char *g2l_coeffs_0_str = j->Attribute("g2l_coeffs_0");
	if (g2l_coeffs_0_str == NULL)
	{
		gap_to_length_coeffs_[0] = -0.01057876;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2l_coeffs_0, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_length_coeffs_[0] = boost::lexical_cast<double>(g2l_coeffs_0_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2l_coeffs_0 (%s) is not a float",g2l_coeffs_0_str);
			return false;
		}
	}
	
	const char *g2l_coeffs_1_str = j->Attribute("g2l_coeffs_1");
	if (g2l_coeffs_1_str == NULL)
	{
		gap_to_length_coeffs_[1] = 0.08412417;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2l_coeffs_1, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_length_coeffs_[1] = boost::lexical_cast<double>(g2l_coeffs_1_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2l_coeffs_1 (%s) is not a float",g2l_coeffs_1_str);
			return false;
		}
	}
	
	const char *g2l_coeffs_2_str = j->Attribute("g2l_coeffs_2");
	if (g2l_coeffs_2_str == NULL)
	{
		gap_to_length_coeffs_[2] = -0.04850085;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2l_coeffs_2, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_length_coeffs_[2] = boost::lexical_cast<double>(g2l_coeffs_2_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2l_coeffs_2 (%s) is not a float",g2l_coeffs_2_str);
			return false;
		}
	}
	
	const char *g2l_coeffs_3_str = j->Attribute("g2l_coeffs_3");
	if (g2l_coeffs_3_str == NULL)
	{
		gap_to_length_coeffs_[3] = -0.87391894;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2l_coeffs_3, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_length_coeffs_[3] = boost::lexical_cast<double>(g2l_coeffs_3_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2l_coeffs_3 (%s) is not a float",g2l_coeffs_3_str);
			return false;
		}
	}
	
	const char *g2l_coeffs_4_str = j->Attribute("g2l_coeffs_4");
	if (g2l_coeffs_4_str == NULL)
	{
		gap_to_length_coeffs_[4] = 6.51653529;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2l_coeffs_4, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_length_coeffs_[4] = boost::lexical_cast<double>(g2l_coeffs_4_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2l_coeffs_4 (%s) is not a float",g2l_coeffs_4_str);
			return false;
		}
	}	
	
	// GAP TO EFFECTIVE DISTANCE
	const char *g2ed_coeffs_0_str = j->Attribute("g2ed_coeffs_0");
	if (g2ed_coeffs_0_str == NULL)
	{
		gap_to_effective_dist_coeffs_[0] = 1.33852307e-02;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2ed_coeffs_0, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_effective_dist_coeffs_[0] = boost::lexical_cast<double>(g2ed_coeffs_0_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2ed_coeffs_0 (%s) is not a float",g2ed_coeffs_0_str);
			return false;
		}
	}
	
	const char *g2ed_coeffs_1_str = j->Attribute("g2ed_coeffs_1");
	if (g2ed_coeffs_1_str == NULL)
	{
		gap_to_effective_dist_coeffs_[1] = -1.50212267e-02;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2ed_coeffs_1, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_effective_dist_coeffs_[1] = boost::lexical_cast<double>(g2ed_coeffs_1_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2ed_coeffs_1 (%s) is not a float",g2ed_coeffs_1_str);
			return false;
		}
	}
	
	const char *g2ed_coeffs_2_str = j->Attribute("g2ed_coeffs_2");
	if (g2ed_coeffs_2_str == NULL)
	{
		gap_to_effective_dist_coeffs_[2] = -4.00341247e-01;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2ed_coeffs_2, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_effective_dist_coeffs_[2] = boost::lexical_cast<double>(g2ed_coeffs_2_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2ed_coeffs_2 (%s) is not a float",g2ed_coeffs_2_str);
			return false;
		}
	}
	
	const char *g2ed_coeffs_3_str = j->Attribute("g2ed_coeffs_3");
	if (g2ed_coeffs_3_str == NULL)
	{
		gap_to_effective_dist_coeffs_[3] = 4.45724019e+00;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2ed_coeffs_3, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_effective_dist_coeffs_[3] = boost::lexical_cast<double>(g2ed_coeffs_3_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2ed_coeffs_3 (%s) is not a float",g2ed_coeffs_3_str);
			return false;
		}
	}
	
	const char *g2ed_coeffs_4_str = j->Attribute("g2ed_coeffs_4");
	if (g2ed_coeffs_4_str == NULL)
	{
		gap_to_effective_dist_coeffs_[4] = -2.30064782e+01;
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: g2ed_coeffs_4, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			gap_to_effective_dist_coeffs_[4] = boost::lexical_cast<double>(g2ed_coeffs_4_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("g2ed_coeffs_4 (%s) is not a float",g2ed_coeffs_4_str);
			return false;
		}
	}		

	const char *enc_ticks_str = j->Attribute("enc_ticks");
	if (enc_ticks_str == NULL)
	{
		enc_ticks_ = 120.0; // 120 pulses per rev
		ROS_WARN("LCGripperTransmission's joint \"%s\" has no coefficient: enc_ticks, using default for LCG v1.46.", joint_name);
	}
	else
	{
		try
		{
			enc_ticks_ = boost::lexical_cast<double>(enc_ticks_str);
		}
		catch (boost::bad_lexical_cast &e)
		{
			ROS_ERROR("enc_ticks (%s) is not a float", enc_ticks_str);
			return false;
		}
	}	
	
	
	
	return true;
}

bool LCGripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
	initPolynomialCoefficients();
	
	const char *name = config->Attribute("name");
	name_ = name ? name : "";
	//myfile.open("transmission_data.txt");
	TiXmlElement *ael = config->FirstChildElement("actuator");
	const char *actuator_name = ael ? ael->Attribute("name") : NULL;
	if (!actuator_name || !robot->getActuator(actuator_name))
	{
		ROS_ERROR("LCGripperTransmission could not find actuator named \"%s\"", actuator_name);
		return false;
	}
	robot->getActuator(actuator_name)->command_.enable_ = true;
	actuator_names_.push_back(actuator_name);
	
	for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
	{
		initParameters(j, robot);		
	}
		
	// Print all coefficients
	ROS_DEBUG("LCGripper transmission parameters for %s: j0x=%f, j0y=%f, j1x=%f, j1y=%f, p0x=%f, p0y=%f, p1x=%f, p1y=%f, p2x=%f, p2y=%f, p3x=%f, p3y=%f, l0=%f, l1=%f, l2=%f, p0_radius=%f, j1_radius=%f, thickness=%f, theta_open=%f, theta_closed=%f, gear_reduction=%f",
	name_.c_str(), j0x_, j0y_, j1x_, j1y_, p0x_, p0y_, p1x_, p1y_, p2x_, p2y_, p3x_, p3y_, l0_, l1_, l2_, p0_radius_, j1_radius_, thickness_, theta_open_, theta_closed_, gear_reduction_);
	
	  // Get passive joint informations
	  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
	  {
	    const char *joint_name = j->Attribute("name");
	    if (!joint_name)
	    {
	      ROS_ERROR("PR2GripperTransmission did not specify joint name");
	      return false;
	    }
	    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);

	    if (!joint)
	    {
	      ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
	      return false;
	    }

	    // add joint name to list
	    joint_names_.push_back(joint_name);  // Adds the passive joints after the gap joint
	    passive_joints_.push_back(joint_name);
	  }

	  // Get screw joint informations
	  for (TiXmlElement *j = config->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
	  {
	    const char *joint_name = j->Attribute("name");
	    if (!joint_name)
	    {
	      ROS_ERROR("PR2GripperTransmission simulated_actuated_joint did snot specify joint name");
	      use_simulated_actuated_joint_=false;
	    }
	    else
	    {
	      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
	      if (!joint)
	      {
	        ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
	        use_simulated_actuated_joint_=false;
	      }
	      else
	      {
	        use_simulated_actuated_joint_=true;
	        joint_names_.push_back(joint_name);  // The first joint is the gap joint

	        // get the thread pitch
	        const char *simulated_reduction = j->Attribute("simulated_reduction");
	        if (!simulated_reduction)
	        {
	          ROS_ERROR("PR2GripperTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
	          return false;
	        }
	        try
	        {
	          simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
	        }
	        catch (boost::bad_lexical_cast &e)
	        {
	          ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
	          return false;
	        }

	        // get any additional joint introduced from this screw joint implementation
	        // for the gripper, this is due to the limitation that screw constraint
	        // requires axis of rotation to be aligned with line between CG's of the two
	        // connected bodies.  For this reason, an additional slider joint was introduced
	        // thus, requiring joint state to be published for motion planning packages
	        // and that's why we're here.
	        const char *passive_actuated_joint_name = j->Attribute("passive_actuated_joint");
	        if (passive_actuated_joint_name)
	        {
	          const boost::shared_ptr<const urdf::Joint> passive_actuated_joint = robot->robot_model_.getJoint(passive_actuated_joint_name);
	          if (passive_actuated_joint)
	          {
	            has_simulated_passive_actuated_joint_ = true;
	            joint_names_.push_back(passive_actuated_joint_name);  // The first joint is the gap joint
	          }
	        }

	      }
	    }
	  }

	  // assuming simulated gripper prismatic joint exists, use it
	
	return true;
}

bool LCGripperTransmission::initXml(TiXmlElement *config)
{
	initPolynomialCoefficients();
	
	const char *name = config->Attribute("name");
	name_ = name ? name : "";

	//myfile.open("transmission_data.txt");
	TiXmlElement *ael = config->FirstChildElement("actuator");
	const char *actuator_name = ael ? ael->Attribute("name") : NULL;

	if (!actuator_name)
	{
		ROS_ERROR("LCGripperTransmission could not find actuator named \"%s\"", actuator_name);
		return false;
	}
	actuator_names_.push_back(actuator_name);
	
	for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
	{
		if (initParameters(j, 0) == false)
		{
			return false;
		}
	}
		
	// Print all coefficients
	ROS_DEBUG("LCGripper transmission parameters for %s: j0x=%f, j0y=%f, j1x=%f, j1y=%f, p0x=%f, p0y=%f, p1x=%f, p1y=%f, p2x=%f, p2y=%f, p3x=%f, p3y=%f, l0=%f, l1=%f, l2=%f, p0_radius=%f, j1_radius=%f, thickness=%f, theta_open=%f, theta_closed=%f, gear_reduction=%f",
	name_.c_str(), j0x_, j0y_, j1x_, j1y_, p0x_, p0y_, p1x_, p1y_, p2x_, p2y_, p3x_, p3y_, l0_, l1_, l2_, p0_radius_, j1_radius_, thickness_, theta_open_, theta_closed_, gear_reduction_);
	
	// Get passive joint informations
	for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
	{
		const char *joint_name = j->Attribute("name");
		if (!joint_name)
		{
			ROS_ERROR("LCGripperTransmission did not specify joint name");
			return false;
		}
		
		// add joint name to list
		joint_names_.push_back(joint_name);  // Adds the passive joints after the gap joint
		passive_joints_.push_back(joint_name);
	}
		
	// Get screw joint informations
	for (TiXmlElement *j = config->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
	{
		const char *joint_name = j->Attribute("name");
		if (!joint_name)
		{
			ROS_ERROR("LCGripperTransmission screw joint did not specify joint name");
			use_simulated_actuated_joint_=false;
		}
		else
		{
			use_simulated_actuated_joint_=true;
			joint_names_.push_back(joint_name);  // The first joint is the gap joint
			
			// get the thread pitch
			const char *simulated_reduction = j->Attribute("simulated_reduction");
			if (!simulated_reduction)
			{
				ROS_ERROR("LCGripperTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
				return false;
			}
			try
			{
				simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
			}
			catch (boost::bad_lexical_cast &e)
			{
				ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
				return false;
			}
			
			// get any additional joint introduced from this screw joint implementation
			// for the gripper, this is due to the limitation that screw constraint
			// requires axis of rotation to be aligned with line between CG's of the two
			// connected bodies.  For this reason, an additional slider joint was introduced
			// thus, requiring joint state to be published for motion planning packages
			// and that's why we're here.
			const char *passive_actuated_joint_name = j->Attribute("passive_actuated_joint");
			if (passive_actuated_joint_name)
			{
				has_simulated_passive_actuated_joint_ = true;
				joint_names_.push_back(passive_actuated_joint_name);  // The first joint is the gap joint
			}
	
		}
	}
	
	// if simulated gripper prismatic joint exists, use it
	if (config->FirstChildElement("use_simulated_gripper_joint")) use_simulated_gripper_joint = true;
	
	return true;
}

///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state; ie, Tendon length -> gripper gap.
/// all passive joints are assigned by single actuator state through mimic?
void LCGripperTransmission::propagatePosition(std::vector<Actuator*>& as, std::vector<JointState*>& js)
{

	// TODO - CHECK THESE. Suspect that the screw joint should be removed. Leave passive joints + gap joint.
	ROS_ASSERT(as.size() == 1); // Only one actuator
	// js has passive joints and 1 gap joint and 1 screw joint
	if (use_simulated_actuated_joint_ && has_simulated_passive_actuated_joint_)
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);
	}
	else if (use_simulated_actuated_joint_) 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
	}
	else 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size());
	}
	
	double motor_pos 		= -as[0]->state_.position_;
	double motor_vel 		= getMotorVelFromEncoderVel(as[0]->state_.velocity_); 
	double motor_torque 	= getMotorTorqueFromEffort(as[0]->state_.last_measured_effort_); // Convert current -> Nm
	
	double tendon_length 	= getLengthFromMotorPos(motor_pos);
	double tendon_vel		= getTendonLengthVelFromMotorVel(motor_vel);
	double tendon_force 	= getTendonForceFromMotorTorque(motor_torque);
	
	double gap_size 		= fabs(getGapFromTendonLength(tendon_length) - gap_open_); // Gap size is in mm
	gap_size 			= validateGapSize(gap_size); // Check bounds
	double gap_vel			= getGapVelFromTendonLengthVel(tendon_length, tendon_vel);
	double gap_force		= getGripperForceFromTendonForce(tendon_force, gap_size);
	
	//ROS_INFO("PropagatePosition(): ENC_POS = %f --> MOTOR_POS = %f --> TENDON_LENGTH = %f --> GAP_SIZE = %f", as[0]->state_.position_, motor_pos, tendon_length, gap_size);
	
	double theta 			= getThetaFromGap(gap_size);
	
	// Determines the state of the gap joint.
	js[0]->position_        = gap_size;
	js[0]->velocity_        = gap_vel; // each finger is moving with this velocity.
	js[0]->measured_effort_ = gap_force;	
	
	// Determines the states of the passive joints.
	// we need to do this for each finger, in simulation, each finger has it's state filled out
	
	double joint_angle = (theta_closed_*M_PI/180.0) - theta;
	double joint_vel = getThetaVelFromGapVel(gap_vel, gap_size);
	//ROS_INFO("PropagatePosition(): joint_angle %f", (joint_angle*180.0/M_PI) );
	for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
	{
		if(i == 1 || i == 2 || i == 4)
			joint_angle = -joint_angle;
	
		//ROS_INFO("Joint %s, position %f", js[i]->joint_->name.c_str(), joint_angle*180.0/M_PI );
		js[i]->position_           = joint_angle; 
		js[i]->velocity_           = joint_vel;
		js[i]->measured_effort_    = 1.0;// TODO: Old.MT / dtheta_dMR / RAD2MR;
	}
	
	if (use_simulated_actuated_joint_)
	{
		// screw joint state is not important to us, fill with zeros
		js[passive_joints_.size()+1]->position_           = 0.0;
		js[passive_joints_.size()+1]->velocity_           = 0.0;
		js[passive_joints_.size()+1]->measured_effort_    = 0.0;
		js[passive_joints_.size()+1]->reference_position_ = 0.0;
		js[passive_joints_.size()+1]->calibrated_         = true; // treat passive simulation joints as "calibrated"
	}
	if (has_simulated_passive_actuated_joint_)
	{
		// screw joint state is not important to us, fill with zeros
		js[passive_joints_.size()+2]->position_           = 0.0;
		js[passive_joints_.size()+2]->velocity_           = 0.0;
		js[passive_joints_.size()+2]->measured_effort_    = 0.0;
		js[passive_joints_.size()+2]->reference_position_ = 0.0;
		js[passive_joints_.size()+2]->calibrated_         = true; // treat passive simulation joints as "calibrated"
	}
}

// this is needed for simulation, so we can recover encoder value given joint angles
// Use joint positions to generate an actuator position.
void LCGripperTransmission::propagatePositionBackwards(std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
	ROS_ASSERT(as.size() == 1); // Only one actuator
	// js has passive joints and 1 gap joint and 1 screw joint
	if (use_simulated_actuated_joint_ && has_simulated_passive_actuated_joint_)
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);
	}
	else if (use_simulated_actuated_joint_) 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
	}
	else 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size());
	}
	
	//ROS_WARN("Read js[0]: %f, js[1]: %f, js[2]: %f, js[3] %f, js[4] %f", js[0]->position_, js[1]->position_*180/M_PI, js[2]->position_*180/M_PI, js[3]->position_*180/M_PI, js[4]->position_*180/M_PI);
	
	double theta1 			= -js[2]->position_ + theta_closed_*M_PI/180.0; // Proximal joint angle, radians 
	double theta1_vel 		= js[2]->velocity_;
	double torqueJ1 		= js[3]->commanded_effort_; // Joints 3/4 are the distal joints.
	
	double gap_size 		= getGapFromTheta(theta1);	
	double tendon_length 	= getTendonLengthFromGap(gap_size);
	double motor_pos 		= getMotorPosFromLength(tendon_length);
	double enc_pos 			= getEncoderPosFromMotorPos(motor_pos) + 5762.49;
	//ROS_ERROR("PropagatePositionBackwards(): Theta1: %f, GAP SIZE: %f, TENDON LENGTH: %f, MOTOR POS: %f, ENC POS: %f", theta1, gap_size, tendon_length, motor_pos, enc_pos);
	
	double gap_rate         = theta1_vel*cos(theta1);
	double tendon_rate		= getTendonLengthVelFromGapVel(gap_rate, gap_size);
	double motor_vel		= getMotorVelFromTendonLengthVel(tendon_rate);
	
	double tendon_force 	= getTendonForceFromTorqueJ1(torqueJ1);
	double motor_torque		= getMotorTorqueFromTendonForce(tendon_force);
	double motor_effort		= getMotorEffortFromTorque(motor_torque);

	as[0]->state_.position_             = enc_pos;
	as[0]->state_.velocity_             = motor_vel;
	as[0]->state_.last_measured_effort_ = motor_effort;
	
	// Update the timing (making sure it's initialized).
	if (! simulated_actuator_timestamp_initialized_)
	{
		// Set the time stamp to zero (it is measured relative to the start time).
		as[0]->state_.sample_timestamp_ = ros::Duration(0);
		
		// Try to set the start time.  Only then do we claim initialized.
		if (ros::isStarted())
		{
			simulated_actuator_start_time_ = ros::Time::now();
			simulated_actuator_timestamp_initialized_ = true;
		}
	}
	else
	{
		// Measure the time stamp relative to the start time.
		as[0]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
	}
	// Set the historical (double) timestamp accordingly.
	as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();
	
	// simulate calibration sensors by filling out actuator states
	this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
}

void LCGripperTransmission::propagateEffort(
		std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
	ROS_ASSERT(as.size() == 1); // Only one actuator
	// js has passive joints and 1 gap joint and 1 screw joint
	if (use_simulated_actuated_joint_ && has_simulated_passive_actuated_joint_)
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);
	}
	else if (use_simulated_actuated_joint_) 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
	}
	else 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size());
	}
	
	
	double gap_effort       = js[0]->commanded_effort_; // Newtons
	double gap_size 	= js[0]->position_; 			// Needed to calculate joint torques.
	double tendon_force 	= getTendonForceFromGripperForce(gap_effort, gap_size);
	double motor_torque 	= getMotorTorqueFromTendonForce(tendon_force);
	double motor_effort 	= getMotorEffortFromTorque(motor_torque);
	
//	ROS_INFO("PropagateEffort(): Gap Pos = %f ; Gap Effort = %f ; Tendon Force %f ; Motor Torque %f ; Motor Effort = %f", gap_size, gap_effort, tendon_force, motor_torque, motor_effort);
	
	as[0]->command_.enable_ = true;
	as[0]->command_.effort_ = motor_effort;
}

void LCGripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
	ROS_ASSERT(as.size() == 1); // Only one actuator
	// js has passive joints and 1 gap joint and 1 screw joint
	if (use_simulated_actuated_joint_ && has_simulated_passive_actuated_joint_)
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2);
	}
	else if (use_simulated_actuated_joint_) 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1);
	}
	else 
	{
		ROS_ASSERT(js.size() == 1 + passive_joints_.size());
	}
	//ROS_ASSERT(simulated_reduction_>0.0);
	
	double motor_effort 	= as[0]->command_.effort_;
	
	// gap_size is required to compute the effective distance from the tendon to the J0 joint
	double motor_pos 		= getMotorPosFromEncoderPos(as[0]->state_.position_);			  
	double tendon_length 	= getLengthFromMotorPos(motor_pos);
	double gap_size 		= getGapFromTendonLength(tendon_length);
		  
	double motor_torque 	= getMotorTorqueFromEffort(motor_effort);
	double tendon_force 	= getTendonForceFromMotorTorque(motor_torque);
	//ROS_WARN("Tendon force: %f", tendon_force);
	double gap_effort 		= getGripperForceFromTendonForce(tendon_force, gap_size);
	
	double Tj0 				= 1000.0*getTorqueJ0FromTendonForce(tendon_force, gap_size);
	double Tj1 				= 1000.0*getTorqueJ1FromTendonForce(tendon_force);
	
	//ROS_WARN("PropagateEffort(): Tj0 = %f", Tj0);
	
    // propagate fictitious joint effort backwards
    // ROS_ERROR("prop eff back eff=%f",js[0]->commanded_effort_);
    if (use_simulated_actuated_joint_)
    {
      // set screw joint effort if simulated
      js[passive_joints_.size()+1]->commanded_effort_  = gap_effort/simulated_reduction_;
      //js[0]->commanded_effort_                         = gap_effort/2.0;
      //ROS_INFO("propagateEffortBackwards(): js[0]->commanded_effort = %f", gap_effort/simulated_reduction_);
    }
    else
    {
      // an ugly hack to lessen instability due to gripper gains
      double eps=0.01;
      js[0]->commanded_effort_  = (1.0-eps)*js[0]->commanded_effort_ + eps*gap_effort/2.0; // skip slider joint effort
    }
}

double LCGripperTransmission::getMotorPosFromLength(double length)
{
	double drivetrain_pos = length * screw_reduction_; // Convert current length into a gearset position
	double motor_pos = 2.0*M_PI*drivetrain_pos * gear_reduction_; // Convert current gearset position (turns) into a motor position (radians).
	return motor_pos; // radians
}

double LCGripperTransmission::getLengthFromMotorPos(double motor_pos)
{
	double drivetrain_pos = motor_pos / (gear_reduction_*2.0*M_PI); // Convert motor position (radians) into gearset position (turns) 
	double tendon_length = drivetrain_pos / screw_reduction_;  // Convert gearset position into a length (reduction + linear motion from ball screw)
	return tendon_length;
}


double LCGripperTransmission::getGapFromTheta(double theta)
{ // NB: theta = radians
	// The gap spacing is defined by proximal joint angle theta, the width of the palm (from j0x), and the thickness of the distal link.
	double gap = 2.0 * (l1_*cos(theta) + abs(j0x_)/2.0 - thickness_)/1000.0;
	return gap;
}

double LCGripperTransmission::getThetaFromGap(double gap)
{
	// The inverse of getGapFromTheta.
	double inner_part = ((gap*1000.0)/2.0 - abs(j0x_)/2.0 + thickness_)/l1_;
	if (inner_part > 1.0) 
	{
		ROS_ERROR("GetThetaFromGap invalid - trying to get acos of %f", inner_part);
		ROS_WARN("gap: %f \tj0x_: %f \tthickness: %f \tl1: %f \tinner: %f", gap, j0x_, thickness_, l1_, inner_part);
		inner_part = 1.0;
	}
	if (inner_part < -1.0)
	{
		ROS_ERROR("GetThetaFromGap invalid - trying to get acos of %f", inner_part);
		ROS_WARN("gap: %f \tj0x_: %f \tthickness: %f \tl1: %f \tinner: %f", gap, j0x_, thickness_, l1_, inner_part);
		inner_part = -1.0;
	}
	double theta = acos(inner_part);
	return theta; // NB: radians
}

double LCGripperTransmission::getTendonLengthFromGap(double gap)
{	
	double length = 0.0;
	for (int i = 0; i <= (int)gap_to_length_coeffs_.size(); i++)
	{
		length += (gap_to_length_coeffs_[i] * pow(gap,i));
	}
	return length;
}

double LCGripperTransmission::getGapFromTendonLength(double length)
{
	double gap_pos = 0.0;
	for (int i = 0; i <= (int)gap_to_length_coeffs_.size(); i++)
	{
		gap_pos += (length_to_gap_coeffs_[i] * pow(length,i));
	}	
	return gap_pos;
}

double LCGripperTransmission::getGapVelFromTendonLengthVel(double length, double length_vel)
{	
	double gap_vel = 0.0;
	double dGap_dLength = 0.0;
	
	// dGap/dTime = dGap/dLength * dLength/dTime
	// Calculates dGap/dLength 
	for (int i = 0; i < (int)length_to_gap_coeffs_.size(); i++)
	{
		dGap_dLength += (i+1) * (length_to_gap_coeffs_[i+1] * pow(length, i));
	}
	// Calculate dGap/dTime
//	ROS_WARN("getGapVelFromTendonLengthVel: dGap_dLength: %f , length_vel %f", dGap_dLength, length);
	gap_vel = dGap_dLength * length_vel;
	
	return gap_vel;
}

double LCGripperTransmission::getTendonLengthVelFromGapVel(double gap_vel, double gap)
{
	
	double length_vel = 0.0;
	double dLength_dGap = 0.0;
	
	// dLength/dTime = dLength/dGap * dGap/dTime
	// Calculates dLength/dGap
	for (int i = 0; i < (int)gap_to_length_coeffs_.size(); i++) 
	{
		dLength_dGap += (i+1) * (gap_to_length_coeffs_[i+1] * pow(gap, i));
	}
	// Calculate dLength/dTime
	length_vel = dLength_dGap * gap_vel;
	
	return length_vel;
}

double LCGripperTransmission::getThetaVelFromGapVel(double gap_vel, double gap_size)
{
	double v = gap_vel/2.0;
	double theta = getThetaFromGap(gap_size);
	double theta_vel = v * sin(theta) / l1_;
	
	return theta_vel;
}

double LCGripperTransmission::getTendonLengthVelFromMotorVel(double motor_vel)
{
	// dLength/dTime = dLength/dMotorPos * dMotorPos/dTime
	// where Length = MotorPos / (GearRatio * ScrewReduction)
	// therefore LengthVel =  MotorVel / (GearRatio * ScrewReduction)
	double length_vel = motor_vel/(screw_reduction_ * gear_reduction_);
	return length_vel;
}

double LCGripperTransmission::getMotorVelFromTendonLengthVel(double length_vel)
{
	double motor_vel = length_vel * (gear_reduction_ * screw_reduction_);
	return motor_vel;
}

double LCGripperTransmission::getTendonForceFromMotorTorque(double motor_torque)
{
	double tendon_force = (motor_torque * 2.0 * M_PI * screw_efficiency_) / screw_lead_;
	return tendon_force;
}

double LCGripperTransmission::getGripperForceFromTendonForce(double tendon_force, double gap_size)
{
	double effective_distance = getTendonEffectiveDistanceToJ0(gap_size);
	double torque = tendon_force * (effective_distance); // Nm
	double force_j1 = torque / (l1_/1000.0); // l1_ is in mm.
	//ROS_ERROR("ED: %f \ttorque: %f \tforce_j1: %f", effective_distance, torque, force_j1);
	double theta1 = getThetaFromGap(gap_size);
	double theta2 = M_PI/2.0 - theta1; // theta2 is 90-theta1.
	double gripper_force = force_j1*cos(theta2);
	
	return gripper_force;
}

double LCGripperTransmission::getTendonForceFromGripperForce(double gripper_force, double gap_size)
{
	// Convert tendon force to gripper force. 	
	double theta1 = getThetaFromGap(gap_size);
	double theta2 = M_PI/2.0 - theta1; // theta2 is 90-theta1.
	double force_j1 = gripper_force / (cos(theta2));
	
	double torque = force_j1 * (l1_ / 1000.0); // l1 is in mm
	
	double effective_distance = getTendonEffectiveDistanceToJ0(gap_size);
	
	double tendon_force = torque / (effective_distance);
	//ROS_WARN("getTendonForceFromGripperForce()
	return tendon_force;
}

double LCGripperTransmission::getMotorTorqueFromTendonForce(double tendon_force)
{
	double motor_torque = (tendon_force * screw_lead_) / (2.0*M_PI*screw_efficiency_);
	return motor_torque;
}

double LCGripperTransmission::getMotorTorqueFromEffort(double motor_effort)
{
	// Convert from motor effort reading to actual torque. 
	// TODO: Check this - this may be handled by the controller anyway.
	double motor_torque = motor_effort;// * MAGICAL CONSTANT.
	return motor_torque;
}

double LCGripperTransmission::getMotorEffortFromTorque(double motor_torque)
{
	// Convert from actual motor torque to an "effort" value. 
	// TODO: Check this - this may be handled by the controller anyway.
	double motor_effort = motor_torque; //  * MAGICAL CONSTANT
	return motor_effort;
}

void LCGripperTransmission::initPolynomialCoefficients()
{
	// TODO: Move these magic numbers out to an xml config.
	double l2g_coeffs[] = { 1.35959902e-01, 1.10396557e+01, -7.24526160e+02, -7.95557419e+04,-2.57497099e+06  };
	double g2l_coeffs[] = { -0.01057876, 0.08412417, -0.04850085, -0.87391894, 6.51653529 };
	double g2ed_coeffs[] = { 1.33852307e-02, -1.50212267e-02, -4.00341247e-01, 4.45724019e+00, -2.30064782e+01 };
	
	//TODO: tidy this initialisation up a bit so that it works with lower order polynomials etc. 
	length_to_gap_coeffs_.clear();
	length_to_gap_coeffs_.assign(l2g_coeffs, l2g_coeffs+5); // 5 coefficients

	gap_to_length_coeffs_.clear();
	gap_to_length_coeffs_.assign(g2l_coeffs, g2l_coeffs+5); // 5 coefficients
	
	gap_to_effective_dist_coeffs_.clear();  // 5 coeffs again.
	gap_to_effective_dist_coeffs_.assign(g2ed_coeffs, g2ed_coeffs+5);
}

double LCGripperTransmission::getTendonEffectiveDistanceToJ0(double gap_size)
{
	double effective_distance = 0.0;
	for (int i = 0; i <= (int)gap_to_effective_dist_coeffs_.size(); i++)
	{
		effective_distance += (gap_to_effective_dist_coeffs_[i] * pow(gap_size,i));
	}	
	return effective_distance;
}

double LCGripperTransmission::getMotorPosFromEncoderPos(double enc_pos)
{
	double motor_pos = (enc_pos/enc_ticks_)*2.0*M_PI; 
	return motor_pos; // Radians
}

double LCGripperTransmission::getEncoderPosFromMotorPos(double motor_pos)
{
	double enc_pos = motor_pos/(2.0*M_PI)*enc_ticks_;
	return enc_pos; // Encoder pulses
}

double LCGripperTransmission::getMotorVelFromEncoderVel(double enc_vel)
{
	double motor_vel = (enc_vel/enc_ticks_)*2.0*M_PI;
	return motor_vel;
}

double LCGripperTransmission::getEncoderVelFromMotorVel(double motor_vel)
{
	double enc_vel = motor_vel/(2.0*M_PI)*enc_ticks_;
	return enc_vel;
}

double LCGripperTransmission::getTorqueJ0FromTendonForce(double tendon_force, double gap_size)
{
	double effective_distance = getTendonEffectiveDistanceToJ0(gap_size);
	double Tj0 = effective_distance * tendon_force;
	return Tj0;
}

double LCGripperTransmission::getTorqueJ1FromTendonForce(double tendon_force)
{
	double Tj1 = j1_radius_/1000.0 * tendon_force;
	return Tj1; // Nm
}

double LCGripperTransmission::getTendonForceFromTorqueJ1(double torque)
{
	double tendon_force = torque*1000.0/j1_radius_;
	return tendon_force;
}


double LCGripperTransmission::validateGapSize(double gap_size)
{
	if (gap_size > gap_open_)
	{
		gap_size = gap_open_;
	}
	
	if (gap_size < gap_closed_)
	{
		gap_size = gap_closed_;
	}
	return gap_size;
}
