#include <iostream>
#include "gripper_control/lcgripper_transmission.h"

#define FORCE_TOL		0.1 // N
#define ED_TOL			0.0001 // mm
#define GAP_TOL 		0.0025 // mm
#define THETA_TOL		0.1 // rad
#define LENGTH_TOL		0.0005 // mm

#define ED_CLOSED		0.0133716 // mm
#define ED_MID			0.0108267 // mm
#define ED_OPEN			0.0072319 // mm

#define GAP_CLOSED		0.0021622// m
#define GAP_MID 		0.0830000 // m --> 60 deg theta
#define GAP_OPEN		0.1357631 // m


#define THETA_OPEN		20.0 * M_PI/180.0 // deg
#define THETA_MID		60.0 * M_PI/180.0 //deg
#define THETA_CLOSED	100.0 * M_PI/180.0 // deg

#define LENGTH_OPEN		0.0 	// m
#define LENGTH_MID		-0.0041172 // m
#define LENGTH_CLOSED	-0.0104045 // m

#define GF_OPEN			0.8244836 // N, given a 20N tendon load
#define GF_MID			3.1254007 // N
#define GF_CLOSED		4.3894740 // N, given a 20N tendon load




#define GRIPPER_FORCE	20.0 // N
#define TENDON_FORCE	20.0 // N


using namespace std;
using namespace pr2_mechanism_model;

int main()
{
	cout << "LCGripperTransmission tests:" << endl;
	
	LCGripperTransmission lcg_trans = LCGripperTransmission();
	
	// Check init functions
	TiXmlElement* config =  new TiXmlElement("config");
	config->SetAttribute("name", "test_config");
	
	TiXmlElement* actuator = new TiXmlElement("actuator");
	actuator->SetAttribute("name", "test_actuator");
	
	TiXmlElement* gap_joint = new TiXmlElement("gap_joint");
	gap_joint->SetAttribute("name", "lcgripper_gap");
	
	config->LinkEndChild(actuator);
	config->LinkEndChild(gap_joint);
	
	lcg_trans.initXml(config);
	
	// Check kinematic mapping functions
	
	// Gap -> Effective distance mapping
	double effective_dist = 0.0;
	cout << "GAP -> EFFECTIVE DISTANCE" << endl;
	effective_dist = lcg_trans.getFlexorMomentArm(GAP_CLOSED);
	cout << "Gap size: " << GAP_CLOSED << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_CLOSED - ED_TOL) && effective_dist < (ED_CLOSED + ED_TOL));
	effective_dist = lcg_trans.getFlexorMomentArm(GAP_MID);
	cout << "Gap size: " << GAP_MID << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_MID - ED_TOL) && effective_dist < (ED_MID + ED_TOL));
	effective_dist = lcg_trans.getFlexorMomentArm(GAP_OPEN);
	cout << "Gap size: " << GAP_OPEN << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_OPEN - ED_TOL) && effective_dist < (ED_OPEN + ED_TOL));		
			
	// Tendon -> Gripper force mapping
	double gripper_force_closed = lcg_trans.getGripperForceFromTendonForce(TENDON_FORCE, GAP_CLOSED);
	cout << "Tendon force: " << TENDON_FORCE << " , gap size: " << GAP_CLOSED << " ---> gripper force: " << gripper_force_closed << endl;
	assert (gripper_force_closed > (GF_CLOSED - FORCE_TOL) && gripper_force_closed < (GF_CLOSED + FORCE_TOL));
	double gripper_force_mid = lcg_trans.getGripperForceFromTendonForce(TENDON_FORCE, GAP_MID);
	cout << "Tendon force: " << TENDON_FORCE << " , gap size: " << GAP_MID << " ---> gripper force: " << gripper_force_mid << endl;
	assert (gripper_force_mid > (GF_MID - FORCE_TOL) && gripper_force_mid < (GF_MID + FORCE_TOL));
	double gripper_force_open = lcg_trans.getGripperForceFromTendonForce(TENDON_FORCE, GAP_OPEN);
	cout << "Tendon force: " << TENDON_FORCE << " , gap size: " << GAP_OPEN << " ---> gripper force: " << gripper_force_open << endl;
	assert (gripper_force_open > (GF_OPEN - FORCE_TOL) && gripper_force_open < (GF_OPEN + FORCE_TOL));
	
	// Gripper force -> Tendon force mapping
	double tendon_force_closed = lcg_trans.getTendonForceFromGripperForce(gripper_force_closed, GAP_CLOSED);
	cout << "Gripper force: " << gripper_force_closed << ", gap: " << GAP_CLOSED << " ---> tendon force: " << tendon_force_closed << endl; 
	assert (tendon_force_closed > (TENDON_FORCE - FORCE_TOL) && tendon_force_closed < (TENDON_FORCE + FORCE_TOL) );
	double tendon_force_mid = lcg_trans.getTendonForceFromGripperForce(gripper_force_mid, GAP_MID);
	cout << "Gripper force: " << gripper_force_mid << ", gap: " << GAP_MID << " ---> tendon force: " << tendon_force_mid << endl; 
	assert (tendon_force_mid > (TENDON_FORCE - FORCE_TOL) && tendon_force_mid < (TENDON_FORCE + FORCE_TOL) );
	double tendon_force_open = lcg_trans.getTendonForceFromGripperForce(gripper_force_open, GAP_OPEN);
	cout << "Gripper force: " << gripper_force_open << ", gap: " << GAP_OPEN << " ---> tendon force: " << tendon_force_open << endl; 
	assert (tendon_force_open > (TENDON_FORCE - FORCE_TOL) && tendon_force_open < (TENDON_FORCE + FORCE_TOL) );
	
	// Gap ---> theta
	double theta_closed = lcg_trans.getThetaFromGap(GAP_CLOSED);
	cout << "Gap: " << GAP_CLOSED << " ---> Theta " << theta_closed << endl; 
	assert (theta_closed > (THETA_CLOSED - THETA_TOL) && theta_closed < (THETA_CLOSED + THETA_TOL) );
	double theta_mid = lcg_trans.getThetaFromGap(GAP_MID);
	cout << "Gap: " << GAP_MID << " ---> Theta " << theta_mid << endl;
	assert (theta_mid > (THETA_MID - THETA_TOL) && theta_mid < (THETA_MID + THETA_TOL) );
	double theta_open = lcg_trans.getThetaFromGap(GAP_OPEN);
	cout << "Gap: " << GAP_OPEN << " ---> Theta " << theta_open << endl;
	assert (theta_open > (THETA_OPEN - THETA_TOL) && theta_open < (THETA_OPEN + THETA_TOL) );
	
	// Theta ---> gap
	double gap_closed = lcg_trans.getGapFromTheta(THETA_CLOSED);
	cout << "Theta: " << THETA_CLOSED << " ---> Gap: " << gap_closed << endl;
	assert (gap_closed > (GAP_CLOSED - GAP_TOL) && gap_closed < (GAP_CLOSED + GAP_TOL) );
	double gap_mid = lcg_trans.getGapFromTheta(THETA_MID);
	cout << "Theta: " << THETA_MID << " ---> Gap: " << gap_mid << endl;
	assert (gap_mid > (GAP_MID - GAP_TOL) && gap_mid < (GAP_MID + GAP_TOL));
	double gap_open = lcg_trans.getGapFromTheta(THETA_OPEN);
	cout << "Theta: " << THETA_OPEN << " ---> Gap: " << gap_open << endl;
	assert (gap_open > (GAP_OPEN - GAP_TOL) && gap_open < (GAP_OPEN + GAP_TOL) );
	
	// Tendon length --> Gap
	gap_closed = lcg_trans.getGapFromTendonLength(LENGTH_CLOSED);
	cout << "Length: " << LENGTH_CLOSED << " ---> Gap: " << gap_closed << endl;
	assert (gap_closed > (GAP_CLOSED - GAP_TOL) && gap_closed < (GAP_CLOSED + GAP_TOL) );
	gap_mid = lcg_trans.getGapFromTendonLength(LENGTH_MID);
	cout << "Length: " << LENGTH_MID << " ---> Gap: " << gap_mid << endl;
	assert (gap_mid > (GAP_MID - GAP_TOL) && gap_mid < (GAP_MID + GAP_TOL));
	gap_open = lcg_trans.getGapFromTendonLength(LENGTH_OPEN);
	cout << "Length: " << LENGTH_OPEN << " ---> Gap: " << gap_open << endl;
	assert (gap_open > (GAP_OPEN - GAP_TOL) && gap_open < (GAP_OPEN + GAP_TOL) );
	
	// Gap --> Tendon length
	double length_closed = lcg_trans.getTendonLengthFromGap(GAP_CLOSED);
	cout << "Gap: " << GAP_CLOSED << " ---> Length: " << length_closed << endl;
	assert (length_closed > (LENGTH_CLOSED - LENGTH_TOL) && length_closed < (LENGTH_CLOSED + LENGTH_TOL) );
	double length_mid = lcg_trans.getTendonLengthFromGap(GAP_MID);
	cout << "Gap: " << GAP_MID << " ---> Length: " << length_mid << endl;
	assert (length_mid > (LENGTH_MID - LENGTH_TOL) && length_mid < (LENGTH_MID + LENGTH_TOL) );
	double length_open = lcg_trans.getTendonLengthFromGap(GAP_OPEN);
	cout << "Gap: " << GAP_OPEN << " ---> Length: " << length_open << endl;
	assert (length_open > (LENGTH_OPEN - LENGTH_TOL) && length_open < (LENGTH_OPEN + LENGTH_TOL) );
	
	// TODO: Motor pos to tendon length
	double motor_open = lcg_trans.getMotorPosFromLength(LENGTH_CLOSED);
	double enc_open = lcg_trans.getEncoderPosFromMotorPos(motor_open);
	
	cout << "Motor_open: " << motor_open << ", enc_open: " << enc_open << endl;
	// TODO: Tendon length to motor pos
	
	// TODO: Motor torque to tendon force
	double mt_20 = lcg_trans.getMotorTorqueFromTendonForce(20.0);
	cout << "MT_20: " << mt_20 << endl;
	
	// TODO: Tendon force to motor torque
	
	cout << "Done" << endl;
	
}
