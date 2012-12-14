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
 * Authors: J Hawke & Bob Holmberg
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
#include <string>

#include <math.h>

using namespace pr2_hardware_interface;
using namespace pr2_mechanism_model;

PLUGINLIB_DECLARE_CLASS(gripper_control, LCGripperTransmission,
                         pr2_mechanism_model::LCGripperTransmission,
                         pr2_mechanism_model::Transmission)



class LCGripperTransmission::ParamFetcher
{

private:
  const TiXmlElement *j_;
  const char* joint_name_;

public:

  int error_count_;

  ros::NodeHandle *nh_;


  // CONSTRUCTOR
  ParamFetcher(const TiXmlElement *j, Robot *robot = NULL): nh_(NULL)
  {
    error_count_=0;
    j_ = j;


    // SET joint_name_ FROM XML
    joint_name_ = j_->Attribute("name");
    if (!joint_name_)
    {
      error_count_++;
      ROS_ERROR("LCGripperTransmission did not specify joint name");
      return;
    }

    // CREATE NODE HANDLE
    nh_ = new ros::NodeHandle(std::string(joint_name_));
    if (!nh_->ok())
    {
      error_count_++;
      ROS_ERROR("LCG Transmission: node handle does not exist/is shutdown");
      return;
    }

    /************************************
    if (robot)
    {
      // SET joint_name_ FROM ROBOT POINTER
      std::string jn_str;
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(jn_str);
      joint_name_ = jn_str.c_str();
      if (!joint)
      {
        error_count_++;
        ROS_ERROR("LCGripperTransmission could not find joint named \"%s\"", joint_name_);
        return;
      }
    }
    ****************************************/


  }

  // DESTRUCTOR
  ~ParamFetcher();

  // API to retrieve joint_name
  const char * getJointName()
  {
    return joint_name_;
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

};


bool LCGripperTransmission::getItems(ParamFetcher *itemFetcher)
{
  // Load parameters from server that is initialized at instantiation of "itemFetcher" object.
  // Joints

  std::cout << "Init Parameters" << std::endl;

  // itemFetcher->getParam("joints/j0x", j0x_);
  // itemFetcher->getParam("joints/j0y", j0y_);
  // itemFetcher->getParam("joints/j1x", j1x_);
  // itemFetcher->getParam("joints/j1y", j1y_);

  // Links
  itemFetcher->getParam("links/l0", l0_);
  itemFetcher->getParam("links/l1", l1_);
  itemFetcher->getParam("links/l2", l2_);
  itemFetcher->getParam("links/thickness", thickness_);

  // Radii
  itemFetcher->getParam("radii/r_c0", r_c0_);
  itemFetcher->getParam("radii/r_c1", r_c1_);
  itemFetcher->getParam("radii/r_e0", r_e0_);
  itemFetcher->getParam("radii/r_e1", r_e1_);
  itemFetcher->getParam("radii/r_f1", r_f1_);

  //itemFetcher->getParam("p0_radius", p0_radius_);
  //itemFetcher->getParam("j1_radius", j1_radius_);

  // Spring
  itemFetcher->getParam("spring/k",  spring_k_);
  itemFetcher->getParam("spring/x0",  spring_x0_);

  // Limits
  itemFetcher->getParam("limits/theta_open_deg",   theta_open_);
  theta_open_ *= DEG2RAD;    // CONVERT TO SI
  itemFetcher->getParam("limits/theta_closed_deg",  theta_closed_);
  theta_closed_ *= DEG2RAD;  // CONVERT TO SI
  itemFetcher->getParam("limits/gap_closed",  gap_closed_);
  itemFetcher->getParam("limits/max_torque",  max_torque_);
  max_torque_ = fabs(max_torque_);

  // Actuator
  itemFetcher->getParam("actuator/screw_lead",    screw_lead_);
  itemFetcher->getParam("actuator/gear_reduction",   gear_reduction_);
  itemFetcher->getParam("actuator/efficiency",    gripper_efficiency_);
  // ERROR-CHECK efficiency
  if (gripper_efficiency_ <= 0.0 || gripper_efficiency_ > 1.0)
    gripper_efficiency_ = 1.0;

  // Polynomial coefficients
  // USE tmp BECAUSE single element of vector<double> does not pass by reference.
  double tmp;
  length_to_gap_coeffs_.resize(5);
  itemFetcher->getParam("polynomials/l2g_0", tmp);  length_to_gap_coeffs_[0] = tmp;
  itemFetcher->getParam("polynomials/l2g_1", tmp);  length_to_gap_coeffs_[1] = tmp;
  itemFetcher->getParam("polynomials/l2g_2", tmp);  length_to_gap_coeffs_[2] = tmp;
  itemFetcher->getParam("polynomials/l2g_3", tmp);  length_to_gap_coeffs_[3] = tmp;
  itemFetcher->getParam("polynomials/l2g_4", tmp);  length_to_gap_coeffs_[4] = tmp;

  gap_to_length_coeffs_.resize(5);
  itemFetcher->getParam("polynomials/g2l_0", tmp);  gap_to_length_coeffs_[0] = tmp;
  itemFetcher->getParam("polynomials/g2l_1", tmp);  gap_to_length_coeffs_[1] = tmp;
  itemFetcher->getParam("polynomials/g2l_2", tmp);  gap_to_length_coeffs_[2] = tmp;
  itemFetcher->getParam("polynomials/g2l_3", tmp);  gap_to_length_coeffs_[3] = tmp;
  itemFetcher->getParam("polynomials/g2l_4", tmp);  gap_to_length_coeffs_[4] = tmp;

  // gap_to_effective_dist_coeffs_.resize(5);
  // itemFetcher->getParam("polynomials/g2ed_0", tmp);  gap_to_effective_dist_coeffs_[0] = tmp;
  // itemFetcher->getParam("polynomials/g2ed_1", tmp);  gap_to_effective_dist_coeffs_[1] = tmp;
  // itemFetcher->getParam("polynomials/g2ed_2", tmp);  gap_to_effective_dist_coeffs_[2] = tmp;
  // itemFetcher->getParam("polynomials/g2ed_3", tmp);  gap_to_effective_dist_coeffs_[3] = tmp;
  // itemFetcher->getParam("polynomials/g2ed_4", tmp);  gap_to_effective_dist_coeffs_[4] = tmp;

  // INITIALIZE MAX GAP AND CORRESPONDING TENDON POSITION, CONSISTENT WITH theta_open_
  // gap_open_ USED IN getGapFromTendonLength(), MUST COMPUTE HARD-CODED HERE TO INITIALIZE IT.
  gap_open_    = 2.0 *(l0_ + l1_*cos(theta_open_) - thickness_);
  tendon_open_ = getTendonLengthFromGap( gap_open_ );
  ROS_WARN("gap_open_=%.4lf,  theta_open_d=%.2lf, tendon_open=%.4lf",gap_open_,theta_open_*RAD2DEG,tendon_open_);

  if ( itemFetcher->error_count_ > 0 )
  {
    ROS_WARN("itemFetcher error_count = %d",itemFetcher->error_count_);
  }
  return !((bool) itemFetcher->error_count_);
}


bool LCGripperTransmission::initParametersFromServer(TiXmlElement *j)
{
  itemFetcher_ = new ParamFetcher(j);
  if ( !getItems(itemFetcher_) )
  {
    return false;
  }

  return true;
}

bool LCGripperTransmission::initParametersFromURDF(TiXmlElement *j, Robot *robot)
{
  itemFetcher_ = new ParamFetcher(j,robot);
  if ( !getItems(itemFetcher_) )
  {
    return false;
  }

  const char *joint_name = itemFetcher_->getJointName();
  gap_joint_ = std::string(joint_name);
  joint_names_.push_back(joint_name);  // The first joint is the gap joint

  int argc = 0;
  char** argv;

  ros::init(argc, argv, gap_joint_);

  lcg_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<gripper_control::LCGTransmissionState>
        (*(itemFetcher_->nh_), "state", 1));

  return true;
}

bool LCGripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
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
    if ( !(initParametersFromURDF(j, robot) || initParametersFromServer(j)) )
    {
      return false;
    }
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
    if ( !(initParametersFromServer(j)) )
    {
      return false;
    }
  }

  // Print all coefficients
  //ROS_DEBUG("LCGripper transmission parameters for %s: l0=%f, l1=%f, l2=%f, thickness=%f, theta_open=%f, theta_closed=%f, gear_reduction=%f",
  //name_.c_str(), l0_, l1_, l2_, thickness_, theta_open_, theta_closed_, gear_reduction_);

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
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2); }
  else if (use_simulated_actuated_joint_)
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1); }
  else
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size()); }

  double tendon_length  = as[0]->state_.position_ * motorGeom2TendonGeom();
  double tendon_vel     = as[0]->state_.velocity_ * motorGeom2TendonGeom();
  double motor_torque   = tqSign_ * as[0]->state_.last_measured_effort_;
  double tendon_force   = motor_torque * gripper_efficiency_ * motorTorque2TendonForce();

  if ( js[0]->calibrated_ )
  {
    double gap_size   = getGapFromTendonLength(tendon_length);
    double gap_vel    = getGapVelFromTendonLengthVel(tendon_length, tendon_vel);

    // The state of the gap joint.
    js[0]->position_        = gap_size;
    js[0]->velocity_        = gap_vel; // each finger is moving with this velocity.
    js[0]->measured_effort_ = getGripperForceFromTendonForce(tendon_force,gap_size);

    // Determines the states of the passive joints.
    // we need to do this for each finger, in simulation, each finger has it's state filled out

    double joint_angle = getThetaFromGap(gap_size);
    double joint_vel = getThetaVelFromGapVel(gap_vel, gap_size);

    for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
    {
      js[i]->position_           = joint_angle;
      if(i == 3 || i == 4)  // distal links open during closing.
        js[i]->position_ = -joint_angle;
      js[i]->velocity_           = joint_vel;
      js[i]->measured_effort_    = 1.0; // TODO: Old.MT / dtheta_dMR / RAD2REV;
    }
  }
  else
  {  // WHEN CALIBRATING, THE TRANSMISSION IS TO THE TENDON ie BALLSCREW
    js[0]->position_        = tendon_length;
    js[0]->velocity_        = tendon_vel;
    js[0]->measured_effort_ = tendon_force;

    double joint_angle = theta_open_;  // BOBH: just a placeholder for calibration
    double joint_vel   = 0.0;
    for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
    {
      js[i]->position_           = joint_angle;
      if(i == 3 || i == 4)
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
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 2); }
  else if (use_simulated_actuated_joint_)
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size() + 1); }
  else
  { ROS_ASSERT(js.size() == 1 + passive_joints_.size());  }

  // if(loop_count_ % 1650 == 0)
  // {
  //   ROS_WARN("Read js[0]: %f, js[1]: %f, js[2]: %f, js[3] %f, js[4] %f", 
  //             js[0]->position_, js[1]->position_*RAD2DEG, js[2]->position_*RAD2DEG, 
  //             js[3]->position_*RAD2DEG, js[4]->position_*RAD2DEG);
  // }

  if ( js[0]->calibrated_ )
  {
    double theta1       = -js[2]->position_ - theta_closed_; // Proximal joint angle, radians
    double theta1_vel   =  js[2]->velocity_;
    //  double torqueJ1     = js[3]->commanded_effort_; // Joints 3/4 are the distal joints.
    double gap_force    =  js[0]->commanded_effort_;

    double gap_size      = getGapFromTheta(theta1);
    double tendon_length = getTendonLengthFromGap(gap_size);
    double motor_pos     = tendon_length * tendonGeom2MotorGeom();
    //ROS_ERROR("PropagatePositionBackwards(): Theta1: %f, GAP SIZE: %f, TENDON LENGTH: %f, MOTOR POS: %f", theta1, gap_size, tendon_length, motor_pos);

    double gap_rate      = theta1_vel*cos(theta1);
    double tendon_rate   = getTendonLengthVelFromGapVel(gap_rate, gap_size);
    double motor_vel     = tendon_rate * tendonGeom2MotorGeom();

    double tendon_force    = getTendonForceFromGripperForce(gap_force, gap_size);
    double motor_torque    = tendon_force * tendonForce2MotorTorque() / gripper_efficiency_;

    as[0]->state_.position_             = motor_pos;
    as[0]->state_.velocity_             = motor_vel;
    as[0]->state_.last_measured_effort_ = tqSign_ * motor_torque;
  }
  else
  {  /* WHEN CALIBRATING, THE TRANSMISSION IS TO TENDON ie BALLSCREW */
    as[0]->state_.position_             = js[0]->position_ * tendonGeom2MotorGeom();
    as[0]->state_.velocity_             = js[0]->velocity_ * tendonGeom2MotorGeom();
    as[0]->state_.last_measured_effort_ = tqSign_ * js[0]->commanded_effort_ * tendonForce2MotorTorque();
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

  if ( js[0]->calibrated_ )
  {
    double gap_effort = tqSign_ * js[0]->commanded_effort_; // Newtons
    double gap_size   = js[0]->position_;       // Needed to calculate forces (varies with gap).

    double tendon_force   = getTendonForceFromGripperForce(gap_effort, gap_size);
    double motor_torque   = tendon_force * tendonForce2MotorTorque() / gripper_efficiency_;

    double tendon_length = getTendonLengthFromGap(gap_size);
    double motor_pos     = tendon_length * tendonGeom2MotorGeom();

    //  ROS_INFO("PropagateEffort(): Gap Pos = %f ; Gap Effort = %f ; Tendon Force %f ; Motor Torque %f", gap_size, gap_effort, tendon_force, motor_torque);

    motor_torque = std::max(-max_torque_, std::min(motor_torque, max_torque_));

    as[0]->command_.enable_ = true;
    as[0]->command_.effort_ = tqSign_ * motor_torque;

    // if(loop_count_ % 1650 == 0)
    // {
    //   ROS_INFO("js0ce=%g, tf=%g, mtq=%g, max_mtq_=%g",
    //            js[0]->commanded_effort_,
    //            tendon_force,
    //            motor_torque,
    //            max_torque_);
    // }

    if(++loop_count_ % 10 == 0 &&
       lcg_state_publisher_ &&
       lcg_state_publisher_->trylock() )
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
  else
  {
    /* WHEN CALIBRATING, THE TRANSMISSION IS TO TENDON ie BALLSCREW */
    as[0]->command_.enable_ = true;
    as[0]->command_.effort_ = tqSign_ * js[0]->commanded_effort_ * tendonForce2MotorTorque();
  }

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

  if ( js[0]->calibrated_ )
  {
    // gap_size is required to compute the effective distance from the tendon to the J0 joint
    double tendon_length = as[0]->state_.position_ * motorGeom2TendonGeom();
    double gap_size       = getGapFromTendonLength(tendon_length);

    double motor_torque   = tqSign_ * as[0]->command_.effort_;
    double tendon_force   = motor_torque * gripper_efficiency_ * motorTorque2TendonForce();
    //ROS_WARN("Tendon force: %f", tendon_force);
    double gap_effort  = getGripperForceFromTendonForce(tendon_force, gap_size);

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
  else
  {
    /* WHEN CALIBRATING, THE TRANSMISSION IS TO TENDON ie BALLSCREW */
    js[0]->commanded_effort_  = tqSign_ * as[0]->command_.effort_ * motorTorque2TendonForce();
  }
}

double LCGripperTransmission::motorGeom2TendonGeom()
{
  double tendon_qty = RAD2REV / gear_reduction_ * screw_lead_;
  return tendon_qty;
}

double LCGripperTransmission::tendonGeom2MotorGeom()
{
  return 1.0/motorGeom2TendonGeom();
}

double LCGripperTransmission::tendonForce2MotorTorque()
{
  return motorGeom2TendonGeom();
}

double LCGripperTransmission::motorTorque2TendonForce()
{
  return 1.0/motorGeom2TendonGeom();
}



double LCGripperTransmission::getGapFromTheta(double theta)
{ // The gap spacing is defined by proximal joint angle theta, 
  // the width of the palm (from l0), and thickness of the distal link.
  theta = std::max(theta,theta_open_);
  double gap = 2.0 * (l0_ + l1_*cos(theta) - thickness_);
  return gap;
}

double LCGripperTransmission::getThetaFromGap(double gap)
{
  static int count = 0;

  // IF gap IS "LARGER", THEN TENDONS ARE SLACK, BUT SET THETA --> theta_open_
  gap = std::min(gap,gap_open_);
  double x   = gap/2.0 + thickness_- l0_;
  double arg = x/l1_;

  if ( fabs(arg) > 1.0 )
  {
    if ( ++count % 1000 == 0 ) {
    ROS_ERROR("GetThetaFromGap invalid - trying to get acos of %.1g", arg);
    ROS_WARN("gap: %.3f \tl0_: %.4f \tgap_open: %.4f \tl1: %.4f \targ: %f", gap, l0_, gap_open_, l1_, arg);
    count=0;
    }
    arg = copysign(0.999999,arg);
  }
  double theta = acos(arg);
  return theta;
}

double LCGripperTransmission::getTendonLengthFromGap(double gap)
{
  double length = 0.0;
  if ( gap <= gap_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    for (int i = 0; i < (int)gap_to_length_coeffs_.size(); i++)
    {
      length += gap_to_length_coeffs_[i] * pow(gap,i);
    }
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    length = tendon_open_/gap_open_ * gap;
  } 

  return length;
}

double LCGripperTransmission::getGapFromTendonLength(double length)
{
  double gap = 0.0;
  if ( length <= tendon_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    for (int i = 0; i < (int)length_to_gap_coeffs_.size(); i++)
    {
      gap += length_to_gap_coeffs_[i] * pow(length,i);
    }
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    gap = gap_open_/tendon_open_ * length;
  }

  return gap;
}

double LCGripperTransmission::dGap_dLength(double length)
{
  double dG_dL = 0.0;

  if ( 0 < length && length <= tendon_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    // Calculate dGap/dLength
    for (int i = 1; i < (int)length_to_gap_coeffs_.size(); i++)
      dG_dL += i * (length_to_gap_coeffs_[i] * pow(length, i-1));
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    dG_dL = gap_open_/tendon_open_;
  }

  return dG_dL;
}

double LCGripperTransmission::getGapVelFromTendonLengthVel(double length, double length_vel)
{
  // dGap/dt = dGap/dLen * dLen/dt
  double gap_vel = dGap_dLength(length) * length_vel;
  return gap_vel;
}

double LCGripperTransmission::dLength_dGap(double gap)
{
  double dL_dG = 0.0;

  if ( 0 < gap && gap <= gap_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    // Calculate dLength/dGap
    for (int i = 1; i < (int)gap_to_length_coeffs_.size(); i++)
      dL_dG += i * gap_to_length_coeffs_[i] * pow(gap, i-1);
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    dL_dG = tendon_open_/gap_open_;
  } 

}

double LCGripperTransmission::getTendonLengthVelFromGapVel(double gap_vel, double gap)
{
  // dLen/dt = dLen/dGap * dGap/dt
  double length_vel = 
 * gap_vel;
  return length_vel;
}

double LCGripperTransmission::getThetaVelFromGapVel(double gap_vel, double gap_size)
{
  double v = gap_vel/2.0;
  double theta = getThetaFromGap(gap_size);
  double theta_vel = v * sin(theta) / l1_;

  return theta_vel;
}

double LCGripperTransmission::getGripperForceFromTendonForce(double tendon_force, double gap_size)
{
  // Subtract force from extensor spring
  double Fs = getExtensorTendonForce(getThetaFromGap(gap_size));
  // Extensor tension projected to grip point force
  double Fe = Fs * r_e1_/(l2_/2.0);
  // Using virtual work to compute
  // Per finger combine: dL/dG_left*Ft/2 and dG/dL=2dG_left
  double Fg = dLength_dGap(gap_size)*tendon_force - Fe;    // Per Finger
  return Fg;
}

double LCGripperTransmission::getTendonForceFromGripperForce(double gripper_force, double gap_size)
{
  // Add force from extensor spring
  double Fs = getExtensorTendonForce(getThetaFromGap(gap_size));
  // Extensor tension projected to grip point force
  double Fe = Fs * r_e1_/(l2_/2.0);
  // Using virtual work to compute
  double len = getTendonLengthFromGap(gap_size);
  // dG_dL is for double distance already, so two tendons are baked-in.
  double Ft = dGap_dLength(len) * (gripper_force + 2.0*Fe); // Two fingers
  return Ft;   
}

double LCGripperTransmission::getExtensorTendonForce(double theta1)
{
  static int count=0;
  theta1 = std::max(theta1,theta_open_);  // Don't let current theta1 go less than open_
  //if (count++%200==0) {ROS_WARN("to=%.1f,  theta1=%.1f",theta_open_*RAD2DEG,theta1*RAD2DEG);}
  double delta_theta = theta1 - theta_open_; // change in angle from the start pose, ie angle at fully open.
  double spring_x = delta_theta * (r_e0_ - r_e1_) + spring_x0_; // spring extension, nominal.
  double ext_force = spring_k_ * spring_x; // extensor tendon force at the current pose.

  ext_force=0;
  return ext_force;
}

double LCGripperTransmission::getMotorQtyFromEncoderQty(double encQty)
{
  double motorQty = encQty*REV2RAD;
  return motorQty;
}

double LCGripperTransmission::getEncoderQtyFromMotorQty(double motorQty)
{
  double encQty = motorQty*RAD2REV;
  return encQty;
}

double LCGripperTransmission::validateGapSize(double gap_size)
{
  gap_size = std::max(gap_closed_,std::min(gap_size,gap_open_));
  return gap_size;
}
