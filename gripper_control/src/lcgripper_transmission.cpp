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
 *   as last_measured_effort_ should be 1to1 gap_effort of non-passive js[i]->commanded_effort
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



class LCGripperTransmission::ParamServer
{
public:
	// CONSTRUCTOR
	ParamServer(const TiXmlElement *j, std::string &gap_joint)
	{
		error_count_=0;
		j_ = j;
		setJointName();

		ros::NodeHandle nh(gap_joint);

		if (!nh.ok())
		{
			error_count_++;
			ROS_ERROR("LCG Transmission: Attempted to load parameters from server, but node handle didn't exist/is shutdown");
			return;
		}
	}

	// CONSTRUCTOR
	ParamServer(const TiXmlElement *j, Robot *robot)
	{
		error_count_=0;
		j_ = j;
		setJointName();
		if (robot)
		{
			const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name_);
			if (!joint)
			{
				error_count_++;
				ROS_ERROR("LCGripperTransmission could not find joint named \"%s\"", joint_name_);
				return;
			}
		}
	}

	// DESTRUCTOR
	~ParamServer();

	// API to retrieve joint_name
	void getJointName(const char *joint_name)
	{
		joint_name = joint_name_;
	}


	// ERROR-CHECKING PARAM GETTER WITH CUSTOM MESSAGE.
	bool getParam(const char *key, double &value)
	{
		if ( nh_!=NULL ) // GET INFO FROM PARAMETER SERVER
		{
			if ( nh_->getParam(key,value) )
			{
				return true;
			}
			else
			{
				error_count_++;
				ROS_WARN("LCG Transmission: Couldn't load \"%s\" from parameter server, joint %s.", key, joint_name_);
				return false;
			}
		}
		else            // GET INFO FROM URDF
		{
			const char *attrib = j_->Attribute(key);
			if ( attrib==NULL )
			{
				error_count_++;
				ROS_WARN("LCGripperTransmission joint \"%s\" has no attribute: %s.", joint_name_, key);
			}
			else
			{
				try
				{
					value = boost::lexical_cast<double>(attrib);
					// RETURN successfully
					return true;
				}
				catch(boost::bad_lexical_cast &e)
				{
					error_count_++;
					ROS_ERROR("%s:(%s) is not a float", key, attrib);
				}
			}
			return false;
		}
	}

	// ERROR-CHECKING PARAM GETTER WITH CUSTOM MESSAGE.
	// Version that sets a default value if param is not defined
	bool getParam(const char *key, double &value, double defaultValue)
	{
		if ( !getParam(key,value) )
		{
			ROS_WARN("LCGripperTransmission joint \"%s\", attribute \"%s\" using default value: %f.", joint_name_, key, defaultValue);
			value = defaultValue;
			return false;
		}
		else
		{
			return true;
		}
	}

	int error_count_;

private:

	const TiXmlElement *j_;
	const char *joint_name_;  //Will always point to a string that was set in an object somewhere else.
	ros::NodeHandle *nh_;

	// SET THE NAME OF THIS joint. USED WITHIN ALL OTHER METHODS.
	bool setJointName()
	{
		joint_name_ = j_->Attribute("name");
		if (joint_name_)
		{
			return true;
		}
		else
		{
			ROS_ERROR("LCGripperTransmission did not specify joint name");
			return false;
		}
	}

};


bool LCGripperTransmission::getItems(ParamServer *itemServer)
{
	// Load parameters from server that is initialized at instantiation of "itemServer" object.
	// Joints
	itemServer->getParam("joints/j0x", j0x_);
	itemServer->getParam("joints/j0y", j0y_);
	itemServer->getParam("joints/j1x", j1x_);
	itemServer->getParam("joints/j1y", j1y_);

	// Links
	itemServer->getParam("links/l0", l0_);
	itemServer->getParam("links/l1", l1_);
	itemServer->getParam("links/l2", l2_);

	// Radii
	itemServer->getParam("radii/r_c0", r_c0_);
	itemServer->getParam("radii/r_c1", r_c1_);
	itemServer->getParam("radii/r_e0", r_e0_);
	itemServer->getParam("radii/r_e1", r_e1_);
	itemServer->getParam("radii/r_f1", r_f1_);

	itemServer->getParam("p0_radius", p0_radius_);
	itemServer->getParam("j1_radius", j1_radius_);
	itemServer->getParam("thickness", thickness_);

	// Spring
	itemServer->getParam("spring/k",	spring_k_);
	itemServer->getParam("spring/x0",	spring_x0_);

	// Limits
	itemServer->getParam("limits/theta_open", 	theta_open_);
	itemServer->getParam("limits/theta_closed",	theta_closed_);
	itemServer->getParam("limits/gap_open",	gap_open_);
	itemServer->getParam("limits/gap_closed",	gap_closed_);
	itemServer->getParam("limits/max_torque",	max_torque_);

	// Actuator
	itemServer->getParam("actuator/screw_lead",		screw_lead_);
	itemServer->getParam("actuator/gear_reduction", 	gear_reduction_);
	itemServer->getParam("actuator/efficiency",		gripper_efficiency_);

	// Polynomial coefficients
	// USE tmp BECAUSE single element of vector<double> does not pass by reference.
	double tmp;
	length_to_gap_coeffs_.reserve(5);
	itemServer->getParam("polynomials/l2g_0", tmp);	length_to_gap_coeffs_[0] = tmp;
	itemServer->getParam("polynomials/l2g_1", tmp);	length_to_gap_coeffs_[1] = tmp;
	itemServer->getParam("polynomials/l2g_2", tmp);	length_to_gap_coeffs_[2] = tmp;
	itemServer->getParam("polynomials/l2g_3", tmp);	length_to_gap_coeffs_[3] = tmp;
	itemServer->getParam("polynomials/l2g_4", tmp);	length_to_gap_coeffs_[4] = tmp;

	gap_to_length_coeffs_.reserve(5);
	itemServer->getParam("polynomials/g2l_0", tmp);	gap_to_length_coeffs_[0] = tmp;
	itemServer->getParam("polynomials/g2l_1", tmp);	gap_to_length_coeffs_[1] = tmp;
	itemServer->getParam("polynomials/g2l_2", tmp);	gap_to_length_coeffs_[2] = tmp;
	itemServer->getParam("polynomials/g2l_3", tmp);	gap_to_length_coeffs_[3] = tmp;
	itemServer->getParam("polynomials/g2l_4", tmp);	gap_to_length_coeffs_[4] = tmp;

	gap_to_effective_dist_coeffs_.reserve(5);
	itemServer->getParam("polynomials/g2ed_0", tmp);	gap_to_effective_dist_coeffs_[0] = tmp;
	itemServer->getParam("polynomials/g2ed_1", tmp);	gap_to_effective_dist_coeffs_[1] = tmp;
	itemServer->getParam("polynomials/g2ed_2", tmp);	gap_to_effective_dist_coeffs_[2] = tmp;
	itemServer->getParam("polynomials/g2ed_3", tmp);	gap_to_effective_dist_coeffs_[3] = tmp;
	itemServer->getParam("polynomials/g2ed_4", tmp);	gap_to_effective_dist_coeffs_[4] = tmp;

	return !((bool) itemServer->error_count_);
}


bool LCGripperTransmission::initParametersFromServer(TiXmlElement *j)
{
	itemServer_ = new ParamServer(j,gap_joint_);
	getItems(itemServer_);

	return true;
}

bool LCGripperTransmission::initParametersFromURDF(TiXmlElement *j, Robot *robot)
{
	std::cout << "Init Parameters" << std::endl;

	itemServer_ = new ParamServer(j,robot);
	getItems(itemServer_);

	const char *joint_name;
	itemServer_->getJointName(joint_name);
	gap_joint_ = std::string(joint_name);
	joint_names_.push_back(joint_name);  // The first joint is the gap joint

	int argc = 0;
	char** argv;
	
	ros::init(argc, argv, gap_joint_);
	ros::NodeHandle nh(gap_joint_);

	lcg_state_publisher_.reset(
			new realtime_tools::RealtimePublisher<gripper_control::LCGTransmissionState>
				(nh, "state", 1));
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
		initParametersFromURDF(j, robot);		
		initParametersFromServer(j);
	}
		
	
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
		if (initParametersFromURDF(j, 0) == false)
		{
			return false;
		}
		if (initParametersFromServer(j) == false)
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
	double motor_torque 		= as[0]->state_.last_measured_effort_; // Convert current -> Nm
	
	if ( js[0]->calibrated_ )
	{
		double tendon_length 		= getLengthFromMotorPos(motor_pos);
		double tendon_vel		= getTendonLengthVelFromMotorVel(motor_vel);
		double tendon_force 		= getTendonForceFromMotorTorque(motor_torque);

		double gap_size 		= fabs(getGapFromTendonLength(tendon_length) - gap_open_); // Gap size is in mm
		//gap_size 			= validateGapSize(gap_size); // Check bounds
		double gap_vel			= getGapVelFromTendonLengthVel(tendon_length, tendon_vel);
	//	double gap_force		= getGripperForceFromTendonForce(tendon_force, gap_size);
	//	gap_force = gap_force * gripper_efficiency_; // Apply efficiency coefficient.
	
		//ROS_INFO("PropagatePosition(): ENC_POS = %f --> MOTOR_POS = %f --> TENDON_LENGTH = %f --> GAP_SIZE = %f", as[0]->state_.position_, motor_pos, tendon_length, gap_size);
	
		// Determines the state of the gap joint.
		js[0]->position_        = gap_size;
		js[0]->velocity_        = gap_vel; // each finger is moving with this velocity.
		js[0]->measured_effort_ = 0.0;//gap_force;

		// Determines the states of the passive joints.
		// we need to do this for each finger, in simulation, each finger has it's state filled out

		double joint_angle = getThetaFromGap(gap_size);
		double joint_vel = getThetaVelFromGapVel(gap_vel, gap_size);

		//ROS_INFO("PropagatePosition(): joint_angle %f", (joint_angle*180.0/M_PI) );
		for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
		{
	//		ROS_INFO("Joint %s, i %d, position %f", js[i]->joint_->name.c_str(), i, joint_angle*180.0/M_PI );
			js[i]->position_           = joint_angle;
			if(i == 3 || i == 4) // Positive joint_angle(ie theta) indicates gripper closing - distal links open during closing.
	                        js[i]->position_ = -joint_angle;

			js[i]->velocity_           = joint_vel;
			js[i]->measured_effort_    = 1.0;// TODO: Old.MT / dtheta_dMR / RAD2REV;
		}
	}
	else
	{	// WHEN CALIBRATING, THE TRANSMISSION IS UNITY, THUS USES ACTUATOR INFORMATION DIRECTLY
		js[0]->position_        = motor_pos;
		js[0]->velocity_        = motor_vel;
		js[0]->measured_effort_ = motor_torque;

		double joint_angle = 20.0*DEG2RAD;
		double joint_vel   =  0.0;
		for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
		{
			js[i]->position_           = joint_angle;
			if(i == 3 || i == 4) // Positive joint_angle(ie theta) indicates gripper closing - distal links open during closing.
	                        js[i]->position_ = -joint_angle;

			js[i]->velocity_           = joint_vel;
			js[i]->measured_effort_    = 1.0;// TODO: Old.MT / dtheta_dMR / RAD2REV;
		}
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
	
	if ( js[0]->calibrated_ )
	{
		double theta1 			= -js[2]->position_ + theta_closed_*M_PI/180.0; // Proximal joint angle, radians
		double theta1_vel 		= js[2]->velocity_;
		//	double torqueJ1 		= js[3]->commanded_effort_; // Joints 3/4 are the distal joints.
		double gap_force		= js[0]->commanded_effort_;

		double gap_size 		= getGapFromTheta(theta1);
		double tendon_length		= getTendonLengthFromGap(gap_size);
		double motor_pos 		= getMotorPosFromLength(tendon_length);
		double enc_pos 			= getEncoderPosFromMotorPos(motor_pos) + 5762.49;
		//ROS_ERROR("PropagatePositionBackwards(): Theta1: %f, GAP SIZE: %f, TENDON LENGTH: %f, MOTOR POS: %f, ENC POS: %f", theta1, gap_size, tendon_length, motor_pos, enc_pos);

		double gap_rate			= theta1_vel*cos(theta1);
		double tendon_rate		= getTendonLengthVelFromGapVel(gap_rate, gap_size);
		double motor_vel		= getMotorVelFromTendonLengthVel(tendon_rate);

		double tendon_force		= getTendonForceFromGripperForce(gap_force, gap_size);
		double motor_torque		= getMotorTorqueFromTendonForce(tendon_force);
		double motor_effort		= getMotorEffortFromTorque(motor_torque);

		as[0]->state_.position_             = enc_pos;
		as[0]->state_.velocity_             = motor_vel;
		as[0]->state_.last_measured_effort_ = motor_effort;
	}
	else
	{	/* WHEN CALIBRATING, THE TRANSMISSION IS UNITY, THUS USES JOINT INFORMATION DIRECTLY */
		// Negate output, since input is negated. TODO: Fix this at the controller level.
		as[0]->state_.position_             = -js[0]->position_;
		as[0]->state_.velocity_             =  js[0]->velocity_;
		as[0]->state_.last_measured_effort_ = -js[0]->commanded_effort_;
	}
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
	
	// NB: Positive gap effort should mean close. The controller currently gives a negative value.	
	double gap_effort       = -js[0]->commanded_effort_; // Newtons
	double gap_size 	= js[0]->position_; 			// Needed to calculate forces (varies with gap).
	
	double tendon_force 	= getTendonForceFromGripperForce(gap_effort, gap_size);
	double motor_torque 	= getMotorTorqueFromTendonForce(tendon_force);
	if (gripper_efficiency_ > 0.0 && gripper_efficiency_ <= 1.0)
		motor_torque = motor_torque / gripper_efficiency_; // Apply the efficiency coefficient.
	
	double tendon_length 	= getTendonLengthFromGap(gap_size);
	double motor_pos 	= getMotorPosFromLength(tendon_length);
	
//	ROS_INFO("PropagateEffort(): Gap Pos = %f ; Gap Effort = %f ; Tendon Force %f ; Motor Torque %f", gap_size, gap_effort, tendon_force, motor_torque);

	if (max_torque_ >= 0.0)
	{
		motor_torque = std::max(-max_torque_, std::min(motor_torque, max_torque_));
	}
	
	as[0]->command_.enable_ = true;
	as[0]->command_.effort_ = -motor_torque; // Negate output, since input is negated. TODO: Fix this at the controller level.
	
	if(loop_count_ % 10 == 0)
	{
		if(lcg_state_publisher_ && lcg_state_publisher_->trylock())
		{
			lcg_state_publisher_->msg_.header.stamp = ros::Time::now();
			lcg_state_publisher_->msg_.gap_size = gap_size;
			lcg_state_publisher_->msg_.tendon_position = tendon_length;
			lcg_state_publisher_->msg_.motor_position = motor_pos;
			lcg_state_publisher_->msg_.gap_force = gap_effort;
			lcg_state_publisher_->msg_.tendon_force = tendon_force;
			lcg_state_publisher_->msg_.motor_torque = motor_torque;
		
			lcg_state_publisher_->unlockAndPublish();
		}
	}
	loop_count_++;
	
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
	double drivetrain_pos = length / screw_lead_; // Convert current length into a gearset position
	double motor_pos = 2.0*M_PI*drivetrain_pos * gear_reduction_; // Convert current gearset position (turns) into a motor position (radians).
	return motor_pos; // radians
}

double LCGripperTransmission::getLengthFromMotorPos(double motor_pos)
{
	double drivetrain_pos = motor_pos / (gear_reduction_*2.0*M_PI); // Convert motor position (radians) into gearset position (turns) 
	double tendon_length = drivetrain_pos * screw_lead_;  // Convert gearset position into a length (reduction + linear motion from ball screw)
	return tendon_length;
}


double LCGripperTransmission::getGapFromTheta(double theta)
{ // NB: theta = radians
	// The gap spacing is defined by proximal joint angle theta, the width of the palm (from j0x), and the thickness of the distal link.
	double gap = 2.0 * (l1_*cos(theta) + fabs(j0x_)/2.0 - thickness_);
	return gap;
}

double LCGripperTransmission::getThetaFromGap(double gap)
{
	// The inverse of getGapFromTheta.
	double inner_part = (gap/2.0 - fabs(j0x_)/2.0 + thickness_)/l1_;
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
	double length_vel = motor_vel * screw_lead_ /( gear_reduction_);
	return length_vel;
}

double LCGripperTransmission::getMotorVelFromTendonLengthVel(double length_vel)
{
	double motor_vel = length_vel * gear_reduction_ / screw_lead_;
	return motor_vel;
}

double LCGripperTransmission::getTendonForceFromMotorTorque(double motor_torque)
{
	double tendon_force = (motor_torque * gear_reduction_ * 2.0 * M_PI) / screw_lead_;
	return tendon_force;
}

double LCGripperTransmission::getGripperForceFromTendonForce(double tendon_force, double gap_size)
{
	/*double effective_distance = getTendonEffectiveDistanceToJ0(gap_size);
	double torque = tendon_force * (effective_distance); // Nm
	double force_j1 = torque / l1_; // l1_ is in mm.
	//ROS_ERROR("ED: %f \ttorque: %f \tforce_j1: %f", effective_distance, torque, force_j1);
	double theta1 = getThetaFromGap(gap_size);
	double theta2 = M_PI/2.0 - theta1; // theta2 is 90-theta1.
	double gripper_force = force_j1*cos(theta2);
	
	return gripper_force;*/

	double t1 = getThetaFromGap(gap_size);	
	double Fe = getExtensorTendonForce(t1);	
	if (tendon_force <= 0.0)
		Fe = 0.0;
	double Ff = tendon_force/2.0; // Divided by 2 as the tendon force is split between the two fingers.
	r_f0_ = getFlexorMomentArm(gap_size);
	r_g0_ = l2_/2.0 + l1_*sin(t1); // Assume force applied to the middle of the distal link.
	r_g1_ = l2_/2.0;

	double Fg = (Ff * (r_f1_ - (r_c1_/r_c0_)*r_f0_) - Fe*(r_e1_ - (r_c1_/r_c0_)*r_e0_)) / (r_g1_ - (r_c1_/r_c0_)*r_g0_); 
//	ROS_INFO("getFGfromFF: input Ft: %f, Gap %f, t1 %f;  Fe: %f, Fg %f", tendon_force, gap_size, t1, Fe, Fg);
	return Fg;

}

double LCGripperTransmission::getTendonForceFromGripperForce(double gripper_force, double gap_size)
{
	// Convert tendon force to gripper force. 	
	double theta1 = getThetaFromGap(gap_size);
	double theta2 = M_PI/2.0 - theta1; // theta2 is 90-theta1.
	double force_j1 = gripper_force / (cos(theta2));

	double torque = force_j1 * l1_; // l1 is in mm
	
	double effective_distance = getFlexorMomentArm(gap_size);
	
	double tendon_force = torque / (effective_distance);
	//ROS_WARN("getTendonForceFromGripperForce()

	
	double t1 = getThetaFromGap(gap_size);
	double Fe = getExtensorTendonForce(t1);
	if (gripper_force <= 0.0) // Positive force = close gripper
		Fe = 0.0;
	double Fg = gripper_force;
	
	r_f0_ = getFlexorMomentArm(gap_size);
	r_g0_ = l2_/2.0 + l1_*sin(t1); // Assume force applied to the middle of the distal link.
	r_g1_ = l2_/2.0;

	double Ff = 2.0* (Fg*(r_g1_ - (r_c1_/r_c0_)*r_g0_) + Fe*(r_e1_ - (r_c1_/r_c0_)*r_e0_)) / (r_f1_ - (r_c1_/r_c0_)*r_f0_); 
//	ROS_INFO("getFFfromFG: Fg: %f, gap %f, t1 %f;    Fe %f, Ff %f   tendon_force %f", Fg, gap_size, t1, Fe, Ff, tendon_force);
	return Ff; // Double the result as the motor force is split between the two tendons
}

double LCGripperTransmission::getExtensorTendonForce(double theta1)
{
		double delta_theta = theta1 - (theta_open_*M_PI/180.0); // change in angle from the start pose, ie angle at fully open.
		double spring_x = fabs(delta_theta) * (r_e0_ - r_e1_) + spring_x0_; // spring extension, nominal.
		double ext_force = spring_k_ * spring_x; // extensor tendon force at the current pose.
		return ext_force;
}

double LCGripperTransmission::getMotorTorqueFromTendonForce(double tendon_force)
{
	double motor_torque = (tendon_force * screw_lead_ / (2.0*M_PI)) / gear_reduction_;
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
	
	//TODO: tidy this initialization up a bit so that it works with lower order polynomials etc.
	length_to_gap_coeffs_.clear();
	length_to_gap_coeffs_.assign(l2g_coeffs, l2g_coeffs+5); // 5 coefficients

	gap_to_length_coeffs_.clear();
	gap_to_length_coeffs_.assign(g2l_coeffs, g2l_coeffs+5); // 5 coefficients
	
	gap_to_effective_dist_coeffs_.clear();  // 5 coeffs again.
	gap_to_effective_dist_coeffs_.assign(g2ed_coeffs, g2ed_coeffs+5);
}

double LCGripperTransmission::getFlexorMomentArm(double gap_size)
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
/*
double LCGripperTransmission::getTorqueJ0FromTendonForce(double tendon_force, double gap_size)
{
	double effective_distance = getTendonEffectiveDistanceToJ0(gap_size);
	double Tj0 = effective_distance * tendon_force;
	return Tj0;
}

double LCGripperTransmission::getTorqueJ1FromTendonForce(double tendon_force)
{
	double Tj1 = j1_radius_ * tendon_force;
	return Tj1; // Nm
}

double LCGripperTransmission::getTendonForceFromTorqueJ1(double torque)
{
	double tendon_force = torque/j1_radius_;
	return tendon_force;
}*/


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
