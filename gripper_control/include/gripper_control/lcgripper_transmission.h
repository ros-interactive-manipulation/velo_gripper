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
 * <transmission type="PR2GripperTransmission" name="gripper_l_transmission">
 *   <actuator       name="l_gripper_motor" />
 *   <gap_joint      name="l_gripper_joint"              mechanical_reduction="1.0" A="0.05"  B="1.0"  C="0.0" />
 *   <passive_joint  name="l_gripper_l_finger_joint"     />
 *   <passive_joint  name="l_gripper_r_finger_joint"     />
 *   <passive_joint  name="l_gripper_r_finger_tip_joint" />
 *   <passive_joint  name="l_gripper_l_finger_tip_joint" />
 * </transmission>
 *
 * Author: John Hsu
 */

#ifndef NEW_GRIPPER_TRANSMISSION_H
#define NEW_GRIPPER_TRANSMISSION_H

#include <vector>
#include <tinyxml.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>

#include "pr2_mechanism_model/transmission.h"
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/joint_calibration_simulator.h"

#include "gripper_control/LCGTransmissionState.h"

namespace pr2_mechanism_model {

class LCGripperTransmission : public Transmission
{
public:
	LCGripperTransmission() 
	{
		use_simulated_actuated_joint_=false;
		has_simulated_passive_actuated_joint_=false;
	};
	virtual ~LCGripperTransmission() {/*myfile.close();*/}
	
	bool initXml(TiXmlElement *config, Robot *robot);
	bool initXml(TiXmlElement *config);
	
	void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>&,
						 std::vector<pr2_mechanism_model::JointState*>&);
	void propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>&,
								  std::vector<pr2_hardware_interface::Actuator*>&);
	void propagateEffort(std::vector<pr2_mechanism_model::JointState*>&,
					   std::vector<pr2_hardware_interface::Actuator*>&);
	void propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>&,
								std::vector<pr2_mechanism_model::JointState*>&);
	std::string gap_joint_;
	
	// if a screw_joint is specified, apply torque based on simulated_reduction_
	double      simulated_reduction_;
	bool        use_simulated_actuated_joint_;
	bool        has_simulated_passive_actuated_joint_;
	
	// The joint_names_ variable is inherited from Transmission.  In
	// joint_names_, the gap joint is first, followed by all the passive
	// joints.
	
	// store name for passive joints.  This matches elements 1 to N of joint_names_.
	std::vector<std::string> passive_joints_;


	void initPolynomialCoefficients();
	bool initParameters(TiXmlElement *j, Robot *robot);

	// Mapping motor states to tendon states and to gripper states (and backwards).
	double getGapFromTendonLength(double length);
	double getTendonLengthFromGap(double gap);
	double getThetaFromGap(double gap);
	double getGapFromTheta(double theta);
	
	double getMotorPosFromLength(double length);
	double getLengthFromMotorPos(double motor_pos);
	
	double getTendonLengthVelFromGapVel(double gap_vel, double gap);
	double getGapVelFromTendonLengthVel(double length, double length_vel);
	double getTendonLengthVelFromMotorVel(double motor_vel);
	double getMotorVelFromTendonLengthVel(double length_vel);
	
	double getTendonForceFromMotorTorque(double motor_torque);
	double getGripperForceFromTendonForce(double tendon_force, double gap_size);
	double getTendonForceFromGripperForce(double gripper_force, double gap_size);
	double getMotorTorqueFromTendonForce(double tendon_force);
	
	double getMotorTorqueFromEffort(double motor_effort);
	double getMotorEffortFromTorque(double motor_torque);
	
	double getMotorPosFromEncoderPos(double enc_pos);
	double getEncoderPosFromMotorPos(double motor_pos);
	double getMotorVelFromEncoderVel(double enc_vel);
	double getEncoderVelFromMotorVel(double motor_vel);
	
	double getThetaVelFromGapVel(double gap_vel, double gap);
	
	double getTendonEffectiveDistanceToJ0(double gap_size);
	
	double getTorqueJ0FromTendonForce(double tendon_force, double gap_size);
	double getTorqueJ1FromTendonForce(double tendon_force);
	double getTendonForceFromTorqueJ1(double torque);

	double validateGapSize(double gap_size);
	
	boost::shared_ptr<
			realtime_tools::RealtimePublisher<
				gripper_control::LCGTransmissionState> > lcg_state_publisher_ ;
	
	
private:	
	// Tendon routing definition. Not actually used - this is replaced by the fitted polynomials.
	double p0x_;
	double p0y_;
	double p1x_;
	double p1y_;
	double p2x_;
	double p2y_;
	double p3x_;
	double p3y_;
	// Joint positions - required for gap/theta conversions.
	double j0x_;
	double j0y_;
	double j1x_;
	double j1y_;
	
	double j1_radius_;
	double p0_radius_;
	// Link lengths. L0 is palm, L1 is proximal, L2 is distal.
	double l0_;
	double l1_;
	double l2_;
	
	double thickness_;
	
	double theta_open_;
	double theta_closed_;
	double gap_open_;
	double gap_closed_;
	double theta0_;
	
	double enc_ticks_;
	
	// FITTED POLYNOMIALS:
	std::vector<double> length_to_gap_coeffs_;
	std::vector<double> gap_to_length_coeffs_;
	std::vector<double> gap_to_effective_dist_coeffs_;

	// Drivetrain parameters
	double encoder_ticks_per_rev_; // eg 1200 pulses per motor rev.
	double gear_reduction_; // gear reduction from motor to ball screw shaft: MotorSpeed/GearReduction -> BallScrewSpeed
	double gear_efficiency_;
	double screw_reduction_;  // ball screw reduction: BallScrewSpeed/ScrewReduction -> metres of travel.
	double screw_efficiency_;
	double screw_lead_;
	
	bool use_simulated_gripper_joint;
	
	int loop_count_; // RT Publisher frequency (ie publish every X cycles).

#define RAD2MR (1.0/(2.0*M_PI)) // convert radians to motor revolutions
#define TOL 0.00001   // limit for denominators
	
	int simulated_actuator_timestamp_initialized_;
	ros::Time simulated_actuator_start_time_;
	
	JointCalibrationSimulator joint_calibration_simulator_;
	
};

} // namespace pr2_mechanism_model

#endif
