//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Matei Ciocarlie 
//
// $Id: mcGrip.h,v 1.6 2009/09/12 00:14:55 cmatei Exp $
//
//######################################################################

#include "gripper_optimization/gripperDesigns.h"

#include "worldElementFactory.h"

#include <limits>

#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"

namespace gripper_optimization {

std::string GripperDesign::getDBName(std::string graspit_name)
{
  if (graspit_name == "gripper_design_8") return "GRIPPER_DESIGN_8";
  else if (graspit_name == "gripper_design_9") return "GRIPPER_DESIGN_9";
  else if (graspit_name == "gripper_design_10") return "GRIPPER_DESIGN_10";
  else if (graspit_name == "gripper_design_11") return "GRIPPER_DESIGN_11";
  else if (graspit_name == "gripper_design_12") return "GRIPPER_DESIGN_12";
  else if (graspit_name == "gripper_design_12_cad") return "GRIPPER_DESIGN_12";
  else if (graspit_name == "gripper_design_12_flex_only") return "GRIPPER_DESIGN_12_FLEX_ONLY";
  else if (graspit_name == "gripper_design_13") return "GRIPPER_DESIGN_13";
  else if (graspit_name == "gripper_design_14") return "GRIPPER_DESIGN_14";
  else if (graspit_name == "gripper_design_15") return "GRIPPER_DESIGN_15";
  else if (graspit_name == "gripper_design_16") return "GRIPPER_DESIGN_16";
  else if (graspit_name == "gripper_design_17") return "GRIPPER_DESIGN_17";
  else if (graspit_name == "gripper_design_18") return "GRIPPER_DESIGN_18";
  else if (graspit_name == "gripper_design_19") return "GRIPPER_DESIGN_19";
  else if (graspit_name == "gripper_design_20") return "GRIPPER_DESIGN_20";
  else if (graspit_name == "gripper_design_21") return "GRIPPER_DESIGN_21";
  else if (graspit_name == "gripper_design_22") return "GRIPPER_DESIGN_22";
  else if (graspit_name == "gripper_design_23") return "GRIPPER_DESIGN_23";
  else if (graspit_name == "gripper_design_24") return "GRIPPER_DESIGN_24";
  else if (graspit_name == "gripper_design_25") return "GRIPPER_DESIGN_25";
  else if (graspit_name == "gripper_design_26") return "GRIPPER_DESIGN_26";
  else if (graspit_name == "gripper_design_27") return "GRIPPER_DESIGN_27";
  else if (graspit_name == "gripper_design_28") return "GRIPPER_DESIGN_28";
  else if (graspit_name == "gripper_design_30") return "GRIPPER_DESIGN_30";
  else if (graspit_name == "gripper_design_31") return "GRIPPER_DESIGN_31";
  else if (graspit_name == "gripper_design_32") return "GRIPPER_DESIGN_32";
  else if (graspit_name == "gripper_design_33") return "GRIPPER_DESIGN_33";
  else if (graspit_name == "gripper_design_34") return "GRIPPER_DESIGN_34";
  else if (graspit_name == "gripper_design_35") return "GRIPPER_DESIGN_35";
  else if (graspit_name == "gripper_design_36") return "GRIPPER_DESIGN_36";
  else if (graspit_name == "gripper_design_37") return "GRIPPER_DESIGN_37";
  else if (graspit_name == "gripper_design_38") return "GRIPPER_DESIGN_38";
  else if (graspit_name == "gripper_design_39") return "GRIPPER_DESIGN_39";
  else if (graspit_name == "gripper_design_40") return "GRIPPER_DESIGN_40";
  else if (graspit_name == "gripper_design_42") return "GRIPPER_DESIGN_42";
  else if (graspit_name == "gripper_design_43") return "GRIPPER_DESIGN_43";
  else if (graspit_name == "gripper_design_4bar") return "GRIPPER_DESIGN_43";
  else if (graspit_name == "gripper_design_45") return "GRIPPER_DESIGN_45";
  else if (graspit_name == "gripper_design_46") return "GRIPPER_DESIGN_46";
  else if (graspit_name == "gripper_design_47") return "GRIPPER_DESIGN_47";
  else if (graspit_name == "gripper_design_48") return "GRIPPER_DESIGN_48";
  else if (graspit_name == "gripper_design_49") return "GRIPPER_DESIGN_49";
  else if (graspit_name == "gripper_design_50") return "GRIPPER_DESIGN_50";
  else if (graspit_name == "gripper_design_51") return "GRIPPER_DESIGN_51";
  else if (graspit_name == "gripper_design_52") return "GRIPPER_DESIGN_52";
  else if (graspit_name == "gripper_design_53") return "GRIPPER_DESIGN_53";
  else if (graspit_name == "gripper_design_54") return "GRIPPER_DESIGN_54";
  else if (graspit_name == "gripper_design_55") return "GRIPPER_DESIGN_55";
  else if (graspit_name == "gripper_design_56") return "GRIPPER_DESIGN_56";
  else if (graspit_name == "gripper_design_57") return "GRIPPER_DESIGN_57";
  else if (graspit_name == "gripper_design_58") return "GRIPPER_DESIGN_58";
  else if (graspit_name == "gripper_design_59") return "GRIPPER_DESIGN_59";
  else if (graspit_name == "gripper_design_60") return "GRIPPER_DESIGN_60";
  else if (graspit_name == "gripper_design_61") return "GRIPPER_DESIGN_61";
  else {DBGA("Can not find db name for gripper name " << graspit_name); return "UNKNOWN";}
}

std::string GripperDesign::getGraspitFile(std::string db_name)
{
  if (db_name == "GRIPPER_DESIGN_8") return "/models/robots/GripperDesign/gripper_design_8.xml";
  else if (db_name == "GRIPPER_DESIGN_9") return "/models/robots/GripperDesign/gripper_design_9.xml";
  else if (db_name == "GRIPPER_DESIGN_10") return "/models/robots/GripperDesign/gripper_design_10.xml";
  else if (db_name == "GRIPPER_DESIGN_11") return "/models/robots/GripperDesign/gripper_design_11.xml";
  else if (db_name == "GRIPPER_DESIGN_12") return "/models/robots/GripperDesign/gripper_design_12.xml";
  else if (db_name == "GRIPPER_DESIGN_12_FLEX_ONLY") 
    return "/models/robots/GripperDesign/gripper_design_12_flex_only.xml";
  else if (db_name == "GRIPPER_DESIGN_13") return "/models/robots/GripperDesign/gripper_design_13.xml";
  else if (db_name == "GRIPPER_DESIGN_14") return "/models/robots/GripperDesign/gripper_design_14.xml";
  else if (db_name == "GRIPPER_DESIGN_15") return "/models/robots/GripperDesign/gripper_design_15.xml";
  else if (db_name == "GRIPPER_DESIGN_16") return "/models/robots/GripperDesign/gripper_design_16.xml";
  else if (db_name == "GRIPPER_DESIGN_17") return "/models/robots/GripperDesign/gripper_design_17.xml";
  else if (db_name == "GRIPPER_DESIGN_18") return "/models/robots/GripperDesign/gripper_design_18.xml";
  else if (db_name == "GRIPPER_DESIGN_19") return "/models/robots/GripperDesign/gripper_design_19.xml";
  else if (db_name == "GRIPPER_DESIGN_20") return "/models/robots/GripperDesign/gripper_design_20.xml";
  else if (db_name == "GRIPPER_DESIGN_21") return "/models/robots/GripperDesign/gripper_design_21.xml";
  else if (db_name == "GRIPPER_DESIGN_22") return "/models/robots/GripperDesign/gripper_design_22.xml";
  else if (db_name == "GRIPPER_DESIGN_23") return "/models/robots/GripperDesign/gripper_design_23.xml";
  else if (db_name == "GRIPPER_DESIGN_24") return "/models/robots/GripperDesign/gripper_design_24.xml";
  else if (db_name == "GRIPPER_DESIGN_25") return "/models/robots/GripperDesign/gripper_design_25.xml";
  else if (db_name == "GRIPPER_DESIGN_26") return "/models/robots/GripperDesign/gripper_design_26.xml";
  else if (db_name == "GRIPPER_DESIGN_27") return "/models/robots/GripperDesign/gripper_design_27.xml";
  else if (db_name == "GRIPPER_DESIGN_28") return "/models/robots/GripperDesign/gripper_design_28.xml";
  else if (db_name == "GRIPPER_DESIGN_30") return "/models/robots/GripperDesign/gripper_design_30.xml";
  else if (db_name == "GRIPPER_DESIGN_31") return "/models/robots/GripperDesign/gripper_design_31.xml";
  else if (db_name == "GRIPPER_DESIGN_32") return "/models/robots/GripperDesign/gripper_design_32.xml";
  else if (db_name == "GRIPPER_DESIGN_33") return "/models/robots/GripperDesign/gripper_design_33.xml";
  else if (db_name == "GRIPPER_DESIGN_34") return "/models/robots/GripperDesign/gripper_design_34.xml";
  else if (db_name == "GRIPPER_DESIGN_35") return "/models/robots/GripperDesign/gripper_design_35.xml";
  else if (db_name == "GRIPPER_DESIGN_36") return "/models/robots/GripperDesign/gripper_design_36.xml";
  else if (db_name == "GRIPPER_DESIGN_37") return "/models/robots/GripperDesign/gripper_design_37.xml";
  else if (db_name == "GRIPPER_DESIGN_38") return "/models/robots/GripperDesign/gripper_design_38.xml";
  else if (db_name == "GRIPPER_DESIGN_39") return "/models/robots/GripperDesign/gripper_design_39.xml";
  else if (db_name == "GRIPPER_DESIGN_40") return "/models/robots/GripperDesign/gripper_design_40.xml";
  else if (db_name == "GRIPPER_DESIGN_42") return "/models/robots/GripperDesign/gripper_design_42.xml";
  else if (db_name == "GRIPPER_DESIGN_43") return "/models/robots/GripperDesign/gripper_design_43.xml";
  else if (db_name == "GRIPPER_DESIGN_45") return "/models/robots/GripperDesign/gripper_design_45.xml";
  else if (db_name == "GRIPPER_DESIGN_46") return "/models/robots/GripperDesign/gripper_design_46.xml";
  else if (db_name == "GRIPPER_DESIGN_47") return "/models/robots/GripperDesign/gripper_design_47.xml";
  else if (db_name == "GRIPPER_DESIGN_48") return "/models/robots/GripperDesign/gripper_design_48.xml";
  else if (db_name == "GRIPPER_DESIGN_49") return "/models/robots/GripperDesign/gripper_design_49.xml";
  else if (db_name == "GRIPPER_DESIGN_50") return "/models/robots/GripperDesign/gripper_design_50.xml";
  else if (db_name == "GRIPPER_DESIGN_51") return "/models/robots/GripperDesign/gripper_design_51.xml";
  else if (db_name == "GRIPPER_DESIGN_52") return "/models/robots/GripperDesign/gripper_design_52.xml";
  else if (db_name == "GRIPPER_DESIGN_53") return "/models/robots/GripperDesign/gripper_design_53.xml";
  else if (db_name == "GRIPPER_DESIGN_54") return "/models/robots/GripperDesign/gripper_design_54.xml";
  else if (db_name == "GRIPPER_DESIGN_55") return "/models/robots/GripperDesign/gripper_design_55.xml";
  else if (db_name == "GRIPPER_DESIGN_56") return "/models/robots/GripperDesign/gripper_design_56.xml";
  else if (db_name == "GRIPPER_DESIGN_57") return "/models/robots/GripperDesign/gripper_design_57.xml";
  else if (db_name == "GRIPPER_DESIGN_58") return "/models/robots/GripperDesign/gripper_design_58.xml";
  else if (db_name == "GRIPPER_DESIGN_59") return "/models/robots/GripperDesign/gripper_design_59.xml";
  else if (db_name == "GRIPPER_DESIGN_60") return "/models/robots/GripperDesign/gripper_design_60.xml";
  else if (db_name == "GRIPPER_DESIGN_61") return "/models/robots/GripperDesign/gripper_design_61.xml";
  else {DBGA("Can not find Graspit file name for dbase name " << db_name); return "unknown";}
}

void GripperDesign::registerCreators()
{
  REGISTER_CREATOR("Gripper7",Gripper7);
  REGISTER_CREATOR("Gripper10",Gripper10);
  REGISTER_CREATOR("Gripper11",Gripper11);
  REGISTER_CREATOR("Gripper12",Gripper12);
  REGISTER_CREATOR("Gripper13",Gripper13);
  REGISTER_CREATOR("Gripper14",Gripper14);
  REGISTER_CREATOR("Gripper15",Gripper15);
  REGISTER_CREATOR("Gripper16",Gripper16);
  REGISTER_CREATOR("Gripper17",Gripper17);
  REGISTER_CREATOR("Gripper18",Gripper18);
  REGISTER_CREATOR("Gripper19",Gripper19);
  REGISTER_CREATOR("Gripper20",Gripper20);
  REGISTER_CREATOR("Gripper21",Gripper21);
  REGISTER_CREATOR("Gripper22",Gripper22);
  REGISTER_CREATOR("Gripper23",Gripper23);
  REGISTER_CREATOR("Gripper24",Gripper24);
  REGISTER_CREATOR("Gripper25",Gripper25);
  REGISTER_CREATOR("Gripper26",Gripper26);
  REGISTER_CREATOR("Gripper27",Gripper27);
  REGISTER_CREATOR("Gripper28",Gripper28);
  REGISTER_CREATOR("Gripper30",Gripper30);
  REGISTER_CREATOR("Gripper31",Gripper31);
  REGISTER_CREATOR("Gripper32",Gripper32);
  REGISTER_CREATOR("Gripper33",Gripper33);
  REGISTER_CREATOR("Gripper34",Gripper34);
  REGISTER_CREATOR("Gripper35",Gripper35);
  REGISTER_CREATOR("Gripper36",Gripper36);
  REGISTER_CREATOR("Gripper37",Gripper37);
  REGISTER_CREATOR("Gripper38",Gripper38);
  REGISTER_CREATOR("Gripper39",Gripper39);
  REGISTER_CREATOR("Gripper40",Gripper40);
  REGISTER_CREATOR("Gripper41",Gripper41);
  REGISTER_CREATOR("Gripper42",Gripper42);
  REGISTER_CREATOR("Gripper43",Gripper43);
  REGISTER_CREATOR("Gripper44",Gripper44);
  REGISTER_CREATOR("Gripper45",Gripper45);
  REGISTER_CREATOR("Gripper46",Gripper46);
  REGISTER_CREATOR("Gripper47",Gripper47);
  REGISTER_CREATOR("Gripper48",Gripper48);
  REGISTER_CREATOR("Gripper49",Gripper49);
  REGISTER_CREATOR("Gripper50",Gripper50);
  REGISTER_CREATOR("Gripper51",Gripper51);
  REGISTER_CREATOR("Gripper52",Gripper52);
  REGISTER_CREATOR("Gripper53",Gripper53);
  REGISTER_CREATOR("Gripper54",Gripper54);
  REGISTER_CREATOR("Gripper55",Gripper55);
  REGISTER_CREATOR("Gripper56",Gripper56);
  REGISTER_CREATOR("Gripper57",Gripper57);
  REGISTER_CREATOR("Gripper58",Gripper58);
  REGISTER_CREATOR("Gripper59",Gripper59);
}

void setInsPtY(TendonInsertionPoint *insPt, double y)
{
  vec3 pt = vec3(insPt->getAttachPoint().x(), y, insPt->getAttachPoint().z());
  insPt->setAttachPoint(pt);
}

void setInsPtXY(TendonInsertionPoint *insPt, double x, double y)
{
  vec3 pt = vec3(x, y, insPt->getAttachPoint().z());
  insPt->setAttachPoint(pt);
}

void setWrapperY(TendonWrapper *wrap, double y)
{
  vec3 loc = wrap->getLocation();
  loc.y() = y;
  wrap->setLocation(loc);
}

double norm(const std::vector<double> &vec)
{
  double n = 0;
  for (size_t i=0; i<vec.size(); i++) 
  {
    if (vec[i] == std::numeric_limits<double>::max() ) return std::numeric_limits<double>::max();
    n += vec[i]*vec[i];
  }
  return sqrt(n);
}

bool normalize(std::vector<double> &vec)
{
  double n = norm(vec);
  if (n < 1.0e-5) return false;
  for (size_t i=0; i<vec.size(); i++) vec[i] /= n;
  return true;
}

double vectorDistance(const std::vector<double> &v1, 
                      const std::vector<double> &v2)
{
  assert(v1.size() == v2.size());
  double d = 0.0;
  for (size_t i=0; i<v1.size(); i++)
  {
    d += (v1[i]-v2[i])*(v1[i]-v2[i]);
  }
  return sqrt(d);
}

double dot(const std::vector<double> &v1, const std::vector<double> &v2)
{
  assert( v1.size() == v2.size() );
  double d = 0;
  for (size_t i=0; i<v1.size(); i++)
  {
    d += v1[i] * v2[i];
  }
  return d;
}

double angularDistance(double dot)
{
  return 0.5 - 0.5 * dot;
}

template<typename T>
T rai(std::list<T> list, size_t index)
{
  typename std::list<T>::iterator it;
  size_t i=0;
  for (it=list.begin(); it!=list.end(); it++)
  {
    if (i==index) return *it;
    i++;
  }
  assert(0);
  return *(list.begin());
}

GripperDesign::GripperDesign(World *w, const char *name) : 
  HumanHand(w,name) 
{
  int numFingertipPoses = 11;
  double fingertipMin =  2.5;
  //double fingertipMin = 25.0;
  double fingertipMax = 95.0;

  double step = (fingertipMax - fingertipMin) / (numFingertipPoses - 1);
  for (int i=0; i<numFingertipPoses; i++)
  {
    double d0 = fingertipMin + i * step;
    double d1 = -d0;
    std::vector<double> pose;
    pose.push_back( d0 * M_PI / 180.0 );
    pose.push_back( d1 * M_PI / 180.0 );
    mFingertipPoses.push_back(pose);
    //DBGA( "Fingertip pose: " << d0 << " " << d1 );
  }
  assert((int)mFingertipPoses.size() == numFingertipPoses);
  
  int numEnvelopingPoses = 7;
  double envelopingMin = 30.0;
  double envelopingMax = 66.0;

  step = (envelopingMax - envelopingMin) / (numEnvelopingPoses - 1);
  for(int i=0; i<numEnvelopingPoses; i++)
  {
    double d0 = envelopingMin + i * step;
    double d1 = 80.0 - 2*d0;
    std::vector<double> pose;
    pose.push_back( d0 * M_PI / 180.0 );
    pose.push_back( d1 * M_PI / 180.0 );
    mEnvelopingPoses.push_back(pose);
    //DBGA( "Enveloping pose: " << d0 << " " << d1 );    
  }
  assert((int)mEnvelopingPoses.size() == numEnvelopingPoses);

}

int GripperDesign::loadFromXml(const TiXmlElement* root,QString rootPath)
{
  if ( HumanHand::loadFromXml(root, rootPath) != SUCCESS) return FAILURE;
  for (int c=0; c<getNumFingers(); c++)
  {
    for (int l=0; l<getFinger(c)->getNumLinks(); l++)
    {
      getFinger(c)->getLink(l)->setTransparency(0.7);
    }
  }
  getPalm()->setTransparency(0.7);


  int allPosesSampling = 10;
  double line_distance = 50;
  double step0 = (getDOF(0)->getMax() - getDOF(0)->getMin() ) / (allPosesSampling - 1);
  double step1 = (getDOF(1)->getMax() - getDOF(1)->getMin() ) / (allPosesSampling - 1);
  for (int i=0; i<allPosesSampling; i++)
  {
    double d0 = getDOF(0)->getMin() + i*step0;
    for (int j=0; j<allPosesSampling; j++)
    {
      double d1 = getDOF(1)->getMin() + j*step1;
      std::vector<double> pose;
      pose.push_back( d0 );
      pose.push_back( d1 );
      if ( (fabs(pose[0] + pose[1]) / sqrt(2)) < line_distance ) mAllPoses.push_back(pose);
      //DBGA( "All pose: " << d0 << " " << d1 );    
    }
  }
  DBGA("All poses: " << mAllPoses.size());

  return SUCCESS;
}

bool GripperDesign::computeTendonEquilibrium(double &result)
{
  std::vector<double> tendonForces;
  std::vector<double> jointResiduals;
  if (tendonEquilibrium(mActiveTendons, mPassiveTendons, true, tendonForces, jointResiduals, result))
  {
    DBGA("Tendon equilibrium analysis failed");
    return false;
  }  
  //bring down results with negative tendon forces
  if (tendonForces[0] < 1.0e5) result = std::numeric_limits<double>::max();   
  return true;  
}

bool GripperDesign::computeContactEquilibrium(bool enveloping, double &result)
{
  std::list<Contact*> contacts;
  if (enveloping) contacts = getEnvelopingContacts();
  else contacts = getFingertipContacts();
  std::vector<double> tendonForces;
  if(contactEquilibrium(contacts, mActiveTendons, tendonForces, result))
  {
    DBGA("Contact equilibrium analysis failed");
    return false;
  }
  //bring down results with negative tendon forces
  if (tendonForces[0] < 1.0e5) result = std::numeric_limits<double>::max();   
  return true;    
}

PROF_DECLARE(GD_RESET_POSE);
void GripperDesign::resetPose()
{
  PROF_TIMER_FUNC(GD_RESET_POSE);
  removeTemporaryInsertionPoints();
  std::vector<double> initial_pose(getNumDOF(), 0.0);
  forceDOFVals(&initial_pose[0]);
  setRestPosition();
}

PROF_DECLARE(GD_SET_POSE);
bool GripperDesign::setPose(const std::vector<double> &pose)
{
  PROF_TIMER_FUNC(GD_SET_POSE);
  if ((int)pose.size() != getNumDOF()) 
  {
    DBGA("Optimization pose has the wrong number of DOFs");
    return false;
  }

  std::vector<double> desired_pose = pose;
  if (getNumTendonWrappers() > 0)
  {
    resetPose();
    std::vector<double> step_by(getNumDOF(), M_PI/36.0);
    moveDOFToContacts(&desired_pose[0], &step_by[0], true);
  }
  else
  {
    forceDOFVals(&desired_pose[0]);
  }
  return true;
}

PROF_DECLARE(GRIPPER_DESIGN_JOINT_GD);
bool GripperDesign::jointPoseGradientDescent(const std::vector<double> &starting_pose,
                                             const std::vector<double> &tendon_forces,
                                             std::vector<double> &final_pose,
                                             bool restrict_above_diagonal,
                                             bool &convergence)
{
  PROF_TIMER_FUNC(GRIPPER_DESIGN_JOINT_GD);
  long int MAX_STEPS = 300;
  //letting the dynamics run forever gets us to 1.08e-5 * 1.0e6
  double CONVERGENCE_THRESHOLD = 1.0e-1 * 1.0e6;
  double STEP_THRESHOLD = 1.0e-4;
  final_pose = starting_pose;
  long int steps = 0;
  while (1)
  {
    if (steps > MAX_STEPS)
    {
      convergence = false;
      break;
    }
    if (!setPose(final_pose)) return false;
  
    double unbalanced_magnitude;
    std::vector<double> joint_residuals;
    std::vector<double> tendon_copies = tendon_forces;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendon_copies, 
                                   joint_residuals, unbalanced_magnitude);
    if (result) return false;
    double step_norm = norm(joint_residuals);
    DBGP("Step " << steps << ". Pose: " << final_pose[0] << " " << final_pose[1] << ". Magnitude norm " << step_norm);
    if (step_norm < CONVERGENCE_THRESHOLD)
    {
      convergence = true;
      break;
    }
    std::vector<double> step_direction = joint_residuals;
    normalize(step_direction);
    
    double max_norm = 10 * 1.0e6;
    double max_step = 0.1;
    step_norm = (step_norm / std::max(max_norm, step_norm)) * max_step;

    std::vector<double> previous_pose = final_pose;
    for (size_t i=0; i<final_pose.size(); i++)
    {
      final_pose[i] = final_pose[i] + step_direction[i] * step_norm;
    }
    if (restrict_above_diagonal)
    {
      if (final_pose.size() == 2) 
      {
        if (final_pose[0] < 0.0) final_pose[0] = 0.0;
        if (final_pose[1] < -final_pose[0]) final_pose[1] = -final_pose[0];
      }
      else
      {
        DBGA("Diagonal restriction hard-coded for 2 joint gripper");
      }
    }
    checkSetDOFVals(&final_pose[0]);

    //compute the size of the actual step
    for (size_t i=0; i<final_pose.size(); i++)
    {
      previous_pose[i] -= final_pose[i];
    }
    step_norm = norm(previous_pose);
    DBGP("Step threshold: " << step_norm);
    if (step_norm < STEP_THRESHOLD)
    {
      convergence = true;
      break;
    }
    steps++;
  }
  return true;
}

bool GripperDesign::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();

  std::vector< std::vector<double> > poses = getFingertipPoses();

  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( poses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  for (size_t i=0; i<poses.size(); i++)
  {
    if (!setPose(poses[i])) return false;
    double r;
    if (!computeTendonEquilibrium(r)) return false;
    results.push_back(r);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

//------------------------------------- FlexorOnlyGripper -----------------------------------------------

std::vector<double> FlexorOnlyGripper::getParameterMin()
{  

  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0};  
  std::vector<double> minPar;
  minPar.resize(8, 0.0);
  memcpy(&minPar[0], &mins[0], 8*sizeof(double));
  return minPar;
}

std::vector<double> FlexorOnlyGripper::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7};
  std::vector<double> maxPar;
  maxPar.resize(8, 0.0);
  memcpy(&maxPar[0], &maxs[0], 8*sizeof(double));
  return maxPar;
}

bool FlexorOnlyGripper::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 8) 
  {
    DBGA("Wrong number of parameters for FlexorOnlyGripper optimization");
    return false;
  }

  if (mTendonVec.size() < 1 || mTendonVec[0]->getNumPermInsPoints() != 6)
  {
    DBGA("FlexorOnlyGripper design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );  
  return true;
}


//------------------------------------- FlexorExtensorGripper -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Tendon 1, 4 insertion points, x and y (8)
    - Wrapper 0, y and radius (2)
    - Wrapper 1, y and radius (2)
    - (optional) Tendon 1 stiffness (1)
 */

std::vector<double> FlexorExtensorGripper::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   -40,  -7, 
                   -40,  -7, 
                   -10,  -7, 
                   -40,  -7,  

                   -5,   1, 
                   -5,  1,
  
                    0.25};  
  std::vector<double> minPar;
  minPar.resize(mNumParameters, 0.0);
  memcpy(&minPar[0], &mins[0], mNumParameters*sizeof(double));
  return minPar;
}

std::vector<double> FlexorExtensorGripper::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 

                   -30,   0, 
                   -30,   0, 
                     0,   0, 
                   -30,   0,   

                   0,   5,  
                   0,  5,

                   5};
  std::vector<double> maxPar;
  maxPar.resize(mNumParameters, 0.0);
  memcpy(&maxPar[0], &maxs[0], mNumParameters*sizeof(double));
  return maxPar;
}

bool FlexorExtensorGripper::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != mNumParameters) 
  {
    DBGA("Wrong number of parameters for FlexorExtensorGripper optimization");
    return false;
  }

  if (mTendonVec.size() < 2 || mTendonWrapperVec.size() < 2 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || mTendonVec[1]->getNumPermInsPoints() != 4)
  {
    DBGA("FlexorExtensorGripper design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[12], parameters[13] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[14], parameters[15] );
  
  //Tendon wrappers
  setWrapperY( mTendonWrapperVec[0], parameters[16] );
  mTendonWrapperVec[0]->setRadius( parameters[17] );
  setWrapperY( mTendonWrapperVec[1], parameters[18] );
  mTendonWrapperVec[1]->setRadius( parameters[19] );
  
  if (mOptimizeStiffness) mTendonVec[1]->setStiffness( parameters[20] * 1.0e6 );

  return true;
}

//------------------------------------- Flexor2ExtensorsNoWrappers -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Tendon 1, 4 insertion points, x and y (8)
    - Tendon 2, 1 insertion point, y (1)
    - Tendons 1 and 2 stiffnesses (2)
 */

std::vector<double> Flexor2ExtensorsNoWrappers::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 
                   -40,  -7, 
                   -40,  -7, 
                   -10,  -7, 
                   -40,  -7,  
                   -5,
                   0.25,
                   0.25};  
  std::vector<double> minPar;
  minPar.resize(19, 0.0);
  memcpy(&minPar[0], &mins[0], 19*sizeof(double));
  return minPar;
}

std::vector<double> Flexor2ExtensorsNoWrappers::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,  
                     0,  7, 
                   -30,  7, 
                   -30,   0, 
                   -30,   0,  
                     0,   0, 
                   -30,   0,
                   -1,
                   5,
                   5};
  std::vector<double> maxPar;
  maxPar.resize(19, 0.0);
  memcpy(&maxPar[0], &maxs[0], 19*sizeof(double));
  return maxPar;
}

bool Flexor2ExtensorsNoWrappers::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 19) 
  {
    DBGA("Wrong number of parameters for Flexor2ExtensorsNoWrappers optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 4 ||
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("Flexor2ExtensorsNoWrappers design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[12], parameters[13] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[14], parameters[15] );

  //Tendon 2 insertion point 1
  setInsPtY( mTendonVec[2]->getPermInsPoint(1), parameters[16] );

  //Tendon stiffness
  mTendonVec[1]->setStiffness(parameters[17] * 1.0e6);
  mTendonVec[2]->setStiffness(parameters[18] * 1.0e6);
    
  return true;
}

//------------------------------------- FlexorExtensors3Points -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Tendon 1, 6 insertion points: xy,y,xy,xy,y,xy (10)
    - Tendon 2, 3 insertion point: xy,y,xy (5)
    - Tendons 1 and 2 stiffnesses (2)
 */

std::vector<double> FlexorExtensors3Points::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   -38,  -7, 
                    -7,
                   -38,  -7, 
                   -10,  -7, 
                    -7,
                   -38,  -7,  
                   
                   -38,-7,
                   -8,
                   -38,-7,

                   0.1,
                   0.1};  
  std::vector<double> minPar;
  minPar.resize(25, 0.0);
  memcpy(&minPar[0], &mins[0], 25*sizeof(double));
  return minPar;
}

std::vector<double> FlexorExtensors3Points::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,  
                     0,  7, 
                   -30,  7, 

                   -30,   0,
                     0,
                   -30,   0,  
                    -2,   0, 
                     0,
                   -30,   0,
                   
                   -30, 0,
                   -1,
                   -30, 0,
                   
                   10,
                   10};
  std::vector<double> maxPar;
  maxPar.resize(25, 0.0);
  memcpy(&maxPar[0], &maxs[0], 25*sizeof(double));
  return maxPar;
}

bool FlexorExtensors3Points::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 25) 
  {
    DBGA("Wrong number of parameters for FlexorExtensors3Points optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 6 ||
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("FlexorExtensors3Points design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3, 4, 5
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtY(  mTendonVec[1]->getPermInsPoint(1), parameters[10] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[11], parameters[12] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[13], parameters[14] );
  setInsPtY(  mTendonVec[1]->getPermInsPoint(4), parameters[15] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(5), parameters[16], parameters[17] );

  //Tendon 2 insertion point 0,1,2
  setInsPtXY( mTendonVec[2]->getPermInsPoint(0), parameters[18], parameters[19] );
  setInsPtY(  mTendonVec[2]->getPermInsPoint(1), parameters[20] );
  setInsPtXY( mTendonVec[2]->getPermInsPoint(2), parameters[21], parameters[22] );

  //Tendon stiffness
  mTendonVec[1]->setStiffness(parameters[23] * 1.0e6);
  mTendonVec[2]->setStiffness(parameters[24] * 1.0e6);
    
  return true;
}

//------------------------------------- ExtensorsGripper -----------------------------------------------

/*! Parameters:
    - Tendon 1, 4 insertion points, x and y (8)
    - Tendon 2, 1 insertion point, y (1)
    - Tendons 1 and 2 stiffnesses (2)
 */

std::vector<double> ExtensorsGripper::getParameterMin()
{  
  double mins[] = {-40,  -7, 
                   -40,  -7, 
                   -10,  -7, 
                   -40,  -7,  
                   -5,
                   0.25,
                   0.25};  
  std::vector<double> minPar;
  minPar.resize(11, 0.0);
  memcpy(&minPar[0], &mins[0], 11*sizeof(double));
  return minPar;
}

std::vector<double> ExtensorsGripper::getParameterMax()
{
  double maxs[] = {-30,   0, 
                   -30,   0,  
                     0,   0, 
                   -30,   0,
                   -1,
                   10,
                   10};
  std::vector<double> maxPar;
  maxPar.resize(11, 0.0);
  memcpy(&maxPar[0], &maxs[0], 11*sizeof(double));
  return maxPar;
}

bool ExtensorsGripper::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 11) 
  {
    DBGA("Wrong number of parameters for ExtensorsGripper optimization");
    return false;
  }

  if (mTendonVec.size() < 3 ||
      mTendonVec[1]->getNumPermInsPoints() != 4 ||
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("ExtensorsGripper design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[6], parameters[7] );

  //Tendon 2 insertion point 1
  setInsPtY( mTendonVec[2]->getPermInsPoint(1), parameters[8] );

  //Tendon stiffness
  mTendonVec[1]->setStiffness(parameters[9] * 1.0e6);
  mTendonVec[2]->setStiffness(parameters[10] * 1.0e6);
    
  return true;
}

//------------------------------------- Gripper 7 -----------------------------------------------

std::vector<double> Gripper7::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 
                   -40,  -7, 
                   -40,  -7, 
                   -40,  -7,  
                   -5,   1, 
                   -5,   1};  
  std::vector<double> minPar;
  minPar.resize(18, 0.0);
  memcpy(&minPar[0], &mins[0], 18*sizeof(double));
  return minPar;
}

std::vector<double> Gripper7::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,  
                     0,  7, 
                   -30,  7, 
                   -30,  0, 
                   -30,  0,
                   -30,  0,
                     0,  5,
                     0,  5};
  std::vector<double> maxPar;
  maxPar.resize(18, 0.0);
  memcpy(&maxPar[0], &maxs[0], 18*sizeof(double));
  return maxPar;
}

bool Gripper7::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 18) 
  {
    DBGA("Wrong number of parameters for Gripper7 optimization");
    return false;
  }

  if (mTendonVec.size() != 2 || 
      mTendonWrapperVec.size() != 2 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 4)
  {
    DBGA("Gripper7 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  //ins point 2 gets placed on the lower edge of the wrapper
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), 
              mTendonWrapperVec[1]->getLocation().x(),
              parameters[16] - parameters[17]); 
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[12], parameters[13] );
  
  //Tendon wrappers
  setWrapperY( mTendonWrapperVec[0], parameters[14] );
  mTendonWrapperVec[0]->setRadius( parameters[15] );
  setWrapperY( mTendonWrapperVec[1], parameters[16] );
  mTendonWrapperVec[1]->setRadius( parameters[17] );
  
  return true;
}

//------------------------------------- Gripper 9 -----------------------------------------------

bool Gripper9::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 18) 
  {
    DBGA("Wrong number of parameters for Gripper9 optimization");
    return false;
  }
  if (mTendonVec.size() != 3 || mTendonWrapperVec.size() != 2 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 4 ||
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("Gripper9 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  //ins point 2 gets placed on the lower edge of the distal wrapper
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), 
              mTendonWrapperVec[1]->getLocation().x(),
              parameters[16] - parameters[17] - 0.1); 
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[12], parameters[13] );
  
  //Tendon 2 insertion point 1 placed on the lower edge of the proximal wrapper
  setInsPtXY( mTendonVec[2]->getPermInsPoint(1),
              mTendonWrapperVec[0]->getLocation().x(),
              parameters[14] - parameters[15] - 0.1);
  
  //Tendon wrappers
  setWrapperY( mTendonWrapperVec[0], parameters[14] );
  mTendonWrapperVec[0]->setRadius( parameters[15] );
  setWrapperY( mTendonWrapperVec[1], parameters[16] );
  mTendonWrapperVec[1]->setRadius( parameters[17] );
  
  return true;
}

//------------------------------------- Gripper 10 -----------------------------------------------

bool Gripper10::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    results.push_back(r);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

//------------------------------------- Gripper 11 -----------------------------------------------

bool Gripper11::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mEnvelopingPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    results.push_back(r);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

//------------------------------------- Gripper 12 -----------------------------------------------

bool Gripper12::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + mEnvelopingPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    results.push_back(r);
  }
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    results.push_back(r);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

//--------------------------------------- Gripper 13 ---------------------------------------------------

std::vector<double> Gripper13::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 
                     1};  
  std::vector<double> minPar;
  minPar.resize(9, 0.0);
  memcpy(&minPar[0], &mins[0], 9*sizeof(double));
  return minPar;
}

std::vector<double> Gripper13::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 
                     5};
  std::vector<double> maxPar;
  maxPar.resize(9, 0.0);
  memcpy(&maxPar[0], &maxs[0], 9*sizeof(double));
  return maxPar;
}

bool Gripper13::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 9) 
  {
    DBGA("Wrong number of parameters for Gripper13 optimization");
    return false;
  }

  if (mTendonVec.size() < 1 || mTendonWrapperVec.size() < 1 || mTendonVec[0]->getNumPermInsPoints() != 6 )
  {
    DBGA("Gripper13 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );
  //Tendon wrapper radius on distal joint
  mTendonWrapperVec[0]->setRadius( parameters[8] );  
  return true;
}


//------------------------------------- Gripper14 -----------------------------------------------

/*! Parameters:
    - Tendon 1, 4 insertion points, x and y (8)
    - Wrapper 1 radius (1)
 */

std::vector<double> Gripper14::getParameterMin()
{  
  double mins[] = {-40,  -7, 
                   -40,  -7, 
                   -40,  -7,  
                     1};  
  std::vector<double> minPar;
  minPar.resize(7, 0.0);
  memcpy(&minPar[0], &mins[0], 7*sizeof(double));
  return minPar;
}

std::vector<double> Gripper14::getParameterMax()
{
  double maxs[] = {-30,   0, 
                   -30,   0,  
                   -30,   0, 
                     5};
  std::vector<double> maxPar;
  maxPar.resize(7, 0.0);
  memcpy(&maxPar[0], &maxs[0], 7*sizeof(double));
  return maxPar;
}

bool Gripper14::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 7) 
  {
    DBGA("Wrong number of parameters for Gripper14 optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || mTendonWrapperVec.size() < 2 || 
      mTendonVec[1]->getNumPermInsPoints() != 4 || mTendonVec[2]->getNumPermInsPoints() != 2)
  {
    DBGA("Gripper14 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[2], parameters[3] );
  //ins point 2 get placed on the lower edge of the wrapper
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), 
              mTendonWrapperVec[0]->getLocation().x(), -mTendonWrapperVec[0]->getRadius() );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[4], parameters[5] );
  
  //Tendon wrapper
  mTendonWrapperVec[1]->setRadius( parameters[6] );
  
  return true;
}

//------------------------------------- Gripper15 -----------------------------------------------

bool Gripper15::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  std::vector<double> equilibriumPoint(2, 0.0);
  equilibriumPoint[0] = 0.55;
  equilibriumPoint[1] = -0.55;

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (!setPose(mAllPoses[i])) return false;
    //compute equilibrium for 1N force
    double unbalanced_magnitude;
    std::vector<double> tendonForces(1, 1.0 * 1.0e6);
    std::vector<double> jointResiduals;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    //ask for joint residuals gradient to point to equilibrium point
    if (!normalize(jointResiduals))
    {
      DBGA("Zero norm for joint residual");
      return false;
    }
    std::vector<double> pose = mAllPoses[i];
    pose[0] = equilibriumPoint[0] - pose[0];
    pose[1] = equilibriumPoint[1] - pose[1];
    normalize(pose);

    double dot = pose[0]*jointResiduals[0] + pose[1]*jointResiduals[1];
    //map it to 0..1 range with lower being better
    dot = (1.0 - dot)/2.0;
    results.push_back(dot);
    DBGP("Dot: " << dot);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper15::getGDGate()
{
  return 1.25;
}

double Gripper15::getGDEps()
{
  return 1.0e-5;
}

double Gripper15::getSaveThreshold()
{
  return 1.5;
}

//------------------------------------- Gripper16 -----------------------------------------------

bool Gripper16::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  std::vector<double> equilibriumPoint1(2, 0.0);
  equilibriumPoint1[0] = 0.55;
  equilibriumPoint1[1] = -0.55;

  std::vector<double> equilibriumPoint2(2, 0.0);
  equilibriumPoint2[0] = 1.25;
  equilibriumPoint2[1] = -1.25;

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (!setPose(mAllPoses[i])) return false;
    //compute equilibrium for 1N force
    double unbalanced_magnitude;
    std::vector<double> tendonForces(1, 1.0 * 1.0e6);
    std::vector<double> jointResiduals;

    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    std::vector<double> pose = mAllPoses[i];
    pose[0] = equilibriumPoint1[0] - pose[0];
    pose[1] = equilibriumPoint1[1] - pose[1];
    normalize(pose);
    double dot = pose[0]*jointResiduals[0] + pose[1]*jointResiduals[1];
    dot = (1.0 - dot)/2.0;
    results.push_back(dot);

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    pose = mAllPoses[i];
    pose[0] = equilibriumPoint2[0] - pose[0];
    pose[1] = equilibriumPoint2[1] - pose[1];
    normalize(pose);
    dot = pose[0]*jointResiduals[0] + pose[1]*jointResiduals[1];
    dot = (1.0 - dot)/2.0;
    results.push_back(dot);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper16::getGDGate()
{
  return 2.0;
}

double Gripper16::getGDEps()
{
  return 1.0e-5;
}

double Gripper16::getSaveThreshold()
{
  return 1.5;
}

//------------------------------------- Gripper17 -----------------------------------------------
std::vector<double> Gripper17::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 
                   -40,  -7, 
                   -40,  -7, 
                   -40,  -7,  
                   -5,   1, 
                   -5,   1,
                   0.25, 
                   0.25};  
  std::vector<double> minPar;
  minPar.resize(20, 0.0);
  memcpy(&minPar[0], &mins[0], 20*sizeof(double));
  return minPar;
}

std::vector<double> Gripper17::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,  
                     0,  7, 
                   -30,  7, 
                   -30,  0, 
                   -30,  0,
                   -30,  0,
                     0,  5,
                   0,  5,
                   5.0,
                   5.0};
  std::vector<double> maxPar;
  maxPar.resize(20, 0.0);
  memcpy(&maxPar[0], &maxs[0], 20*sizeof(double));
  return maxPar;
}


bool Gripper17::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 20) 
  {
    DBGA("Wrong number of parameters for Gripper17 optimization");
    return false;
  }
  if (mTendonVec.size() != 3 || mTendonWrapperVec.size() != 2 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 4 ||
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("Gripper9 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8], parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  //ins point 2 gets placed on the lower edge of the distal wrapper
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), 
              mTendonWrapperVec[1]->getLocation().x(),
              parameters[16] - parameters[17] - 0.1); 
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[12], parameters[13] );
  
  //Tendon 2 insertion point 1 placed on the lower edge of the proximal wrapper
  setInsPtXY( mTendonVec[2]->getPermInsPoint(1),
              mTendonWrapperVec[0]->getLocation().x(),
              parameters[14] - parameters[15] - 0.1);
  
  //Tendon wrappers
  setWrapperY( mTendonWrapperVec[0], parameters[14] );
  mTendonWrapperVec[0]->setRadius( parameters[15] );
  setWrapperY( mTendonWrapperVec[1], parameters[16] );
  mTendonWrapperVec[1]->setRadius( parameters[17] );
  
  //Tendon stiffness
  mTendonVec[1]->setStiffness(parameters[18] * 1.0e6);
  mTendonVec[2]->setStiffness(parameters[19] * 1.0e6);

  return true;
}


double Gripper17::getGDGate()
{
  return 3.0;
}

double Gripper17::getGDEps()
{
  return 1.0e-5;
}

double Gripper17::getSaveThreshold()
{
  return 1.5;
}

//------------------------------------- Gripper18 -----------------------------------------------

double Gripper18::weightFunction(double d)
{
  return std::max(0.0, (2.0 - d)) * 0.5;
}

bool Gripper18::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  std::vector<double> equilibriumPoint1(2, 0.0);
  equilibriumPoint1[0] = 0.55;
  equilibriumPoint1[1] = -0.55;

  std::vector<double> equilibriumPoint2(2, 0.0);
  equilibriumPoint2[0] = 1.25;
  equilibriumPoint2[1] = -1.25;

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (!setPose(mAllPoses[i])) return false;
    //compute equilibrium for 1N force
    double unbalanced_magnitude;
    std::vector<double> tendonForces(1, 1.0 * 1.0e6);
    std::vector<double> jointResiduals;

    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    std::vector<double> pose = mAllPoses[i];
    pose[0] = equilibriumPoint1[0] - pose[0];
    pose[1] = equilibriumPoint1[1] - pose[1];
    normalize(pose);
    double dot = pose[0]*jointResiduals[0] + pose[1]*jointResiduals[1];
    dot = (1.0 - dot)/2.0;
    double w = weightFunction( vectorDistance(mAllPoses[i], equilibriumPoint1) );
    results.push_back(w * dot);

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    pose = mAllPoses[i];
    pose[0] = equilibriumPoint2[0] - pose[0];
    pose[1] = equilibriumPoint2[1] - pose[1];
    normalize(pose);
    dot = pose[0]*jointResiduals[0] + pose[1]*jointResiduals[1];
    dot = (1.0 - dot)/2.0;
    w = weightFunction( vectorDistance(mAllPoses[i], equilibriumPoint2) );
    results.push_back(dot);
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper18::getGDGate()
{
  return 6.0;
}

double Gripper18::getGDEps()
{
  return 1.0e-5;
}

double Gripper18::getSaveThreshold()
{
  return 3.0;
}

//------------------------------------- Gripper19 -----------------------------------------------

double Gripper19::weightFunction(double d)
{
  double w = std::max(0.0, (2.0 - d)) * 0.5;
  return w*w;
}

//------------------------------------- Gripper20 -----------------------------------------------

double Gripper20::getDotValue(const std::vector<double> &jointResiduals, size_t pose_index)
{
  double dot;
  if (pose_index == 0)
  {
    //first pose must point down the line
    std::vector<double> desiredDirection(2,0.0);
    desiredDirection[0] =  1.0;
    desiredDirection[1] = -1.0;
    normalize(desiredDirection);
    dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
    dot = (1.0 - dot)/2.0;
    //first and last pose get twice the weight
    dot *= 2;
  }
  else if (pose_index == mFingertipPoses.size()-1)
  {
    //last pose must point up the line
    std::vector<double> desiredDirection(2,0.0);
    desiredDirection[0] = -1.0;
    desiredDirection[1] =  1.0;
    normalize(desiredDirection);
    dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
    dot = (1.0 - dot)/2.0;
    //first and last pose get twice the weight
    dot *= 2;
  }
  else
  {
    //all other poses must just point along the line, either up or down
    std::vector<double> desiredDirection(2,0.0);
    desiredDirection[0] =  1.0;
    desiredDirection[1] = -1.0;
    normalize(desiredDirection);
    dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
    dot = 1.0 - fabs(dot);
  }
  DBGP("Dot: " << dot);
  return dot;
}

bool Gripper20::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    DBGP("Pose: " << mFingertipPoses[i][0] << " " << mFingertipPoses[i][1]);
    if (!setPose(mFingertipPoses[i])) return false;

    //compute equilibrium for 1N force
    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back(getDotValue(jointResiduals, i));

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back(getDotValue(jointResiduals, i));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper20::getGDGate()
{
  return 1.75;
}

double Gripper20::getGDEps()
{
  return 1.0e-5;
}

double Gripper20::getSaveThreshold()
{
  return 1.75;
}

//------------------------------------- Gripper21 -----------------------------------------------

double Gripper21::getDotValue(const std::vector<double> &jointResiduals, size_t pose_index)
{
  double dot;
  DBGP("Dot: " << dot);

  std::vector<double> desiredDirection(2,0.0);
  if ( mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = -1.0;
    desiredDirection[1] = -1.0;
  }
  else if (mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] =  1.0;
    desiredDirection[1] =  1.0;
  }
  else
  {
    //points on the line are not accounted for
    return 0;
  }
  normalize(desiredDirection);
  dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
  dot = (1.0 - dot)/2.0;
  return dot;
}

bool Gripper21::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    DBGP("Pose: " << mAllPoses[i][0] << " " << mAllPoses[i][1]);
    if (!setPose(mAllPoses[i])) return false;

    //compute equilibrium for 1N force
    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    DBGP("Residuals for 1N: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    DBGP("Normalized residuals for 1N: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(getDotValue(jointResiduals, i));

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Residuals for 2N: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    DBGP("Normalized residuals for 2N: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(getDotValue(jointResiduals, i));
    DBGP("");
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper21::getGDGate()
{
  return 0.75;
}

double Gripper21::getGDEps()
{
  return 1.0e-5;
}

double Gripper21::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper22 -----------------------------------------------
PROF_DECLARE(GRIPPER22_OPTIMIZATION);
bool Gripper22::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  PROF_RESET_ALL;
  PROF_START_TIMER(GRIPPER22_OPTIMIZATION);

  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    DBGP("Pose: " << mAllPoses[i][0] << " " << mAllPoses[i][1]);
    std::vector<double> tendonForces(1, 0.0);
    std::vector<double> finalPose(2, 0.0);
    bool convergence;
    double distance;

    tendonForces[0] = 1.0 * 1.0e6;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (convergence) distance = fabs(finalPose[0] + finalPose[1]) / sqrt(2);
    else {results.push_back(std::numeric_limits<double>::max()); break;}
    results.push_back(distance);

    tendonForces[0] = 2.0 * 1.0e6;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (convergence) distance = fabs(finalPose[0] + finalPose[1]) / sqrt(2);
    else {results.push_back(std::numeric_limits<double>::max()); break;}
    results.push_back(distance);
  }
  results.insert(results.begin(), norm(results));

  PROF_STOP_TIMER(GRIPPER22_OPTIMIZATION);
  PROF_PRINT_ALL;

  return true;  
}

double Gripper22::getGDGate()
{
  return 0.5;
}

double Gripper22::getGDEps()
{
  return 1.0e-5;
}

double Gripper22::getSaveThreshold()
{
  return 0.5;
}
  
//------------------------------------- Gripper23 -----------------------------------------------

/*! Parameters:
    - Tendon 1, 3 insertion points, x and y (6)
    - Wrapper 1 radius (1)
    - stiffness of both extensor tendons (2)
 */

std::vector<double> Gripper23::getParameterMin()
{  
  double mins[] = {-40,  -7, 
                   -40,  -7, 
                   -40,  -7,  
                   1,
                   0.25,
                   0.25};  
  std::vector<double> minPar;
  minPar.resize(9, 0.0);
  memcpy(&minPar[0], &mins[0], 9*sizeof(double));
  return minPar;
}

std::vector<double> Gripper23::getParameterMax()
{
  double maxs[] = {-30,   0, 
                   -30,   0,  
                   -30,   0, 
                   5,
                   5.0,
                   5.0};
  std::vector<double> maxPar;
  maxPar.resize(9, 0.0);
  memcpy(&maxPar[0], &maxs[0], 9*sizeof(double));
  return maxPar;
}

bool Gripper23::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != 9) 
  {
    DBGA("Wrong number of parameters for Gripper23 optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || mTendonWrapperVec.size() < 2 || 
      mTendonVec[1]->getNumPermInsPoints() != 4 || mTendonVec[2]->getNumPermInsPoints() != 2)
  {
    DBGA("Gripper23 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 1 insertion points 0, 1, 2, 3
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[2], parameters[3] );
  //ins point 2 get placed on the lower edge of the wrapper
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), 
              mTendonWrapperVec[0]->getLocation().x(), -mTendonWrapperVec[0]->getRadius() );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(3), parameters[4], parameters[5] );
  
  //Tendon wrapper
  mTendonWrapperVec[1]->setRadius( parameters[6] );

  //Tendon stiffness
  mTendonVec[1]->setStiffness(parameters[7] * 1.0e6);
  mTendonVec[2]->setStiffness(parameters[8] * 1.0e6);

  return true;
}

PROF_DECLARE(GRIPPER23_OPTIMIZATION);
bool Gripper23::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  PROF_RESET_ALL;
  PROF_START_TIMER(GRIPPER23_OPTIMIZATION);

  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    DBGP("Pose: " << mAllPoses[i][0] << " " << mAllPoses[i][1]);
    std::vector<double> tendonForces(1, 0.0);
    std::vector<double> finalPose(2, 0.0);
    std::vector<double> desiredConvergence(2, 0.0);
    bool convergence;

    tendonForces[0] = 1.0 * 1.0e6;
    desiredConvergence[0] = desiredConvergence[1] = 0.25;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );

    tendonForces[0] = 2.0 * 1.0e6;
    desiredConvergence[0] = desiredConvergence[1] = 0.75;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );
  }
  results.insert(results.begin(), norm(results));

  PROF_STOP_TIMER(GRIPPER23_OPTIMIZATION);
  PROF_PRINT_ALL;

  return true;  
}

double Gripper23::getGDGate()
{
  return 9.0;
}

double Gripper23::getGDEps()
{
  return 1.0e-5;
}

double Gripper23::getSaveThreshold()
{
  return 9.0;
}

//------------------------------------- Gripper24 -----------------------------------------------

PROF_DECLARE(GRIPPER24_OPTIMIZATION);
bool Gripper24::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  PROF_RESET_ALL;
  PROF_START_TIMER(GRIPPER24_OPTIMIZATION);

  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();

  //free space motion
  std::vector<double> free_space_results;
  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    DBGP("Pose: " << mAllPoses[i][0] << " " << mAllPoses[i][1]);
    std::vector<double> tendonForces(1, 0.0);
    std::vector<double> finalPose(2, 0.0);
    std::vector<double> desiredConvergence(2, 0.0);
    bool convergence;

    tendonForces[0] = 1.0 * 1.0e6;
    desiredConvergence[0] = desiredConvergence[1] = 0.25;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (!convergence) free_space_results.push_back(2.0);
    else free_space_results.push_back( vectorDistance( finalPose, desiredConvergence ) );

    tendonForces[0] = 2.0 * 1.0e6;
    desiredConvergence[0] = desiredConvergence[1] = 0.75;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, false, convergence)) return false;
    if (!convergence) free_space_results.push_back(2.0);
    else free_space_results.push_back( vectorDistance( finalPose, desiredConvergence ) );
  }

  //enveloping grasps
  //std::cerr << "Envel individual results: ";
  std::vector<double> enveloping_results;
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    enveloping_results.push_back(r);
    //std::cerr << r << " ";
  }
  //std::cerr << "\n";

  //fingertip grasps
  //std::cerr << "Fingertip individual results: ";
  std::vector<double> fingertip_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    fingertip_results.push_back(r);
    //std::cerr << r << " ";
  }
  //std::cerr << "\n";

  DBGP( "Free: " << norm(free_space_results) << 
        " Envel: " << 1.0e-8 * norm(enveloping_results)  << 
        " Tip: " << 1.0e-8 * norm(fingertip_results) );

  //decide relative weighting of results, which are in different units
  results.push_back( norm(free_space_results) );
  results.push_back( 1.0e-8 * norm(enveloping_results) );
  results.push_back( 1.0e-8 * norm(fingertip_results) );

  results.insert(results.begin(), norm(results));
  PROF_STOP_TIMER(GRIPPER24_OPTIMIZATION);
  PROF_PRINT_ALL;

  return true;  
}

double Gripper24::getGDGate()
{
  return 12.0;
}

double Gripper24::getGDEps()
{
  return 1.0e-5;
}

double Gripper24::getSaveThreshold()
{
  return 12.0;
}

//------------------------------------- Gripper25 -----------------------------------------------

double Gripper25::getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index)
{
  double dot;
  std::vector<double> desiredDirection(2,0.0);
  if ( mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] = -1.0;
  }
  else if (mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] =  1.0;
  }
  else
  {
    //points on the line are not accounted for
    return 0;
  }
  normalize(desiredDirection);
  dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
  dot = (1.0 - dot)/2.0;
  return dot;
}

PROF_DECLARE(GRIPPER25_OPTIMIZATION);
bool Gripper25::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  PROF_RESET_ALL;
  PROF_START_TIMER(GRIPPER25_OPTIMIZATION);

  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    DBGP("Pose: " << mFingertipPoses[i][0] << " " << mFingertipPoses[i][1]);
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    std::vector<double> desiredDirection(2,0.0);
    //all fingertip poses must point down the line
    desiredDirection[0] =  1.0;
    desiredDirection[1] = -1.0;
    normalize(desiredDirection);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirection) );

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirection) );
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (mAllPoses[i][0] == mAllPoses[i][1]) continue;
    if (!setPose(mAllPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back(getDotValueToLine(jointResiduals, i));

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    results.push_back(getDotValueToLine(jointResiduals, i));
  }

  results.insert(results.begin(), norm(results));

  PROF_STOP_TIMER(GRIPPER25_OPTIMIZATION);
  PROF_PRINT_ALL;

  return true;  
}

double Gripper25::getGDGate()
{
  return 4.75;
}

double Gripper25::getGDEps()
{
  return 1.0e-5;
}

double Gripper25::getSaveThreshold()
{
  return 4.75;
}

//------------------------------------- Gripper26 -----------------------------------------------

double Gripper26::getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index)
{
  double dot;
  std::vector<double> desiredDirection(2,0.0);
  if ( mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] = -1.0;
  }
  else if (mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] =  1.0;
  }
  else
  {
    //points on the line are not accounted for
    return 0;
  }
  normalize(desiredDirection);
  dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
  dot = (1.0 - dot)/2.0;
  return dot;
}

PROF_DECLARE(GRIPPER24_OPTIMIZATION);
bool Gripper26::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  PROF_RESET_ALL;
  PROF_START_TIMER(GRIPPER24_OPTIMIZATION);

  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();

  //free space motion
  std::vector<double> free_space_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    DBGP("Pose: " << mFingertipPoses[i][0] << " " << mFingertipPoses[i][1]);
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    std::vector<double> desiredDirection(2,0.0);
    //all fingertip poses must point down the line
    desiredDirection[0] =  1.0;
    desiredDirection[1] = -1.0;
    normalize(desiredDirection);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirection) );

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirection) );
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (mAllPoses[i][0] == mAllPoses[i][1]) continue;
    if (!setPose(mAllPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back(getDotValueToLine(jointResiduals, i));

    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back(getDotValueToLine(jointResiduals, i));
  }


  //enveloping grasps
  //std::cerr << "Envel individual results: ";
  std::vector<double> enveloping_results;
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    enveloping_results.push_back(r);
    //std::cerr << r << " ";
  }
  //std::cerr << "\n";

  //fingertip grasps
  //std::cerr << "Fingertip individual results: ";
  std::vector<double> fingertip_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    fingertip_results.push_back(r);
    //std::cerr << r << " ";
  }
  //std::cerr << "\n";

  DBGP( "Free: " << norm(free_space_results) << 
        " Envel: " << 1.0e-8 * norm(enveloping_results)  << 
        " Tip: " << 1.0e-8 * norm(fingertip_results) );

  //decide relative weighting of results, which are in different units
  results.push_back( norm(free_space_results) );
  results.push_back( 1.0e-8 * norm(enveloping_results) );
  results.push_back( 1.0e-8 * norm(fingertip_results) );

  results.insert(results.begin(), norm(results));
  PROF_STOP_TIMER(GRIPPER24_OPTIMIZATION);
  PROF_PRINT_ALL;

  return true;  
}

double Gripper26::getGDGate()
{
  return 4.5;
}

double Gripper26::getGDEps()
{
  return 1.0e-5;
}

double Gripper26::getSaveThreshold()
{
  return 4.5;
}

//------------------------------------- Gripper27 -----------------------------------------------

double Gripper27::getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index)
{
  double dot;
  std::vector<double> desiredDirection(2,0.0);
  if ( mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] = -1.0;
  }
  else if (mAllPoses[pose_index][1] > -mAllPoses[pose_index][0] )
  {
    desiredDirection[0] = desiredDirection[1] =  1.0;
  }
  else
  {
    //points on the line are not accounted for
    return 0;
  }
  normalize(desiredDirection);
  dot = jointResiduals[0] * desiredDirection[0] + jointResiduals[1] * desiredDirection[1];
  dot = (1.0 - dot)/2.0;
  return dot;
}

bool Gripper27::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();

  //free space motion
  std::vector<double> free_space_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    DBGP("Pose: " << mFingertipPoses[i][0] << " " << mFingertipPoses[i][1]);
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    std::vector<double> desiredDirection(2,0.0);
    //all fingertip poses must point down the line
    desiredDirection[0] =  1.0;
    desiredDirection[1] = -1.0;
    normalize(desiredDirection);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirection) );
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (mAllPoses[i][0] == mAllPoses[i][1]) continue;
    if (!setPose(mAllPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 1.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 1.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back(getDotValueToLine(jointResiduals, i));
  }


  //enveloping grasps
  std::vector<double> enveloping_results;
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    enveloping_results.push_back(r);
  }

  //fingertip grasps
  std::vector<double> fingertip_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    fingertip_results.push_back(r);
  }

  DBGP( "Free: " << norm(free_space_results) << 
        " Envel: " << 1.0e-8 * norm(enveloping_results)  << 
        " Tip: " << 1.0e-8 * norm(fingertip_results) );

  //decide relative weighting of results, which are in different units
  results.push_back( norm(free_space_results) );
  results.push_back( 1.0e-8 * norm(enveloping_results) );
  results.push_back( 1.0e-8 * norm(fingertip_results) );

  results.insert(results.begin(), norm(results));

  return true;  
}

double Gripper27::getGDGate()
{
  return 2.0;
}

double Gripper27::getGDEps()
{
  return 1.0e-5;
}

double Gripper27::getSaveThreshold()
{
  return 2.0;
}

//------------------------------------- Gripper28 -----------------------------------------------

bool Gripper28::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();

  //free space motion
  std::vector<double> free_space_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    DBGP("Pose: " << mFingertipPoses[i][0] << " " << mFingertipPoses[i][1]);
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    std::vector<double> desiredDirectionDown(2,0.0);
    desiredDirectionDown[0] =  1.0;
    desiredDirectionDown[1] = -1.0;
    normalize(desiredDirectionDown);
    std::vector<double> desiredDirectionUp(2,0.0);
    desiredDirectionUp[0] =  1.0;
    desiredDirectionUp[1] = -1.0;
    normalize(desiredDirectionUp);

    tendonForces[0] = 2.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirectionDown) );

    tendonForces[0] = 0.25 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.25N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back( 0.5 - 0.5 * dot(jointResiduals, desiredDirectionUp) );
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if (mAllPoses[i][0] == mAllPoses[i][1]) continue;
    if (!setPose(mAllPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);

    tendonForces[0] = 2.0 * 1.0e6;
    int result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back(getDotValueToLine(jointResiduals, i));

    tendonForces[0] = 0.25 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.25N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}    
    free_space_results.push_back(getDotValueToLine(jointResiduals, i));
  }

  //enveloping grasps
  std::vector<double> enveloping_results;
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(true, r)) return false;
    enveloping_results.push_back(r);
  }

  //fingertip grasps
  std::vector<double> fingertip_results;
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    double r;
    if (!computeContactEquilibrium(false, r)) return false;
    fingertip_results.push_back(r);
  }

  DBGP( "Free: " << norm(free_space_results) << 
        " Envel: " << 1.0e-8 * norm(enveloping_results)  << 
        " Tip: " << 1.0e-8 * norm(fingertip_results) );

  //decide relative weighting of results, which are in different units
  results.push_back( norm(free_space_results) );
  results.push_back( 1.0e-8 * norm(enveloping_results) );
  results.push_back( 1.0e-8 * norm(fingertip_results) );

  results.insert(results.begin(), norm(results));

  return true;  
}

double Gripper28::getGDGate()
{
  return 4.0;
}

double Gripper28::getGDEps()
{
  return 1.0e-5;
}

double Gripper28::getSaveThreshold()
{
  return 4.0;
}

//------------------------------------- Gripper30 -----------------------------------------------

double coneDistance( const std::vector<double> &v,
                     double d0x, double d0y, 
                     double d1x, double d1y )
{
  std::vector<double> c0(2), c1(2);
  c0[0] = d0x; c0[1] = d0y; normalize(c0);
  c1[0] = d1x; c1[1] = d1y; normalize(c1);

  std::vector<double> sum(2);
  sum[0] = d0x + d1x;
  sum[1] = d0y + d1y;
  normalize(sum);
  
  std::vector<double> vnorm = v;
  normalize(vnorm);

  double dmid = dot(vnorm, sum);
  double dcone = dot(c0, sum);
  if (dmid > dcone) return 0.0;
  return std::min( angularDistance(dot(vnorm, c0)), angularDistance(dot(vnorm, c1)) );
}

double coneDistance( const std::vector<double> &v,
                     std::vector<double> d0,
                     std::vector<double> d1 )
{
  assert(d0.size() == 2 && d1.size() == 2);
  return coneDistance(v, d0[0], d0[1], d1[0], d1[1]);
}

bool Gripper30::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    //if ( mEnvelopingPoses[i].at(1) < -mEnvelopingPoses[i].at(0) ) continue;
    //if ( (fabs(mEnvelopingPoses[i].at(0) + mEnvelopingPoses[i].at(1)) / sqrt(2)) < 0.1  ) continue;

    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  -1, -1));

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  1, 0.4,  1, 1));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGA("Joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    DBGA("Normalized joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(multiplier * coneDistance(jointResiduals,  -0.4, -1,  1, -1));
    DBGA("Cone distance " << coneDistance(jointResiduals,  -0.4, -1,  1, -1));
    DBGA("");

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals,  -1, 1,  -1, -0.4));

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, 1,  1, 0.4));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper30::getGDGate()
{
  return 1.0;
}

double Gripper30::getGDEps()
{
  return 1.0e-5;
}

double Gripper30::getSaveThreshold()
{
  return 1.0;
}

void Gripper30::updateJointValuesFromDynamics()
{
  HumanHand::updateJointValuesFromDynamics();
    
  assert( getNumChains() > 0 && getChain(0)->getNumJoints() > 1 && getNumDOF() > 1);
  getChain(0)->getJoint(1)->setMin( -getChain(0)->getJoint(0)->getVal() - 0.01 );
  getDOF(1)->updateMinMax();
  
  
}

//------------------------------------- Gripper31 -----------------------------------------------


//! Clamps a number to be between -1 and 1
double clamp(double v)
{
  return std::max( std::min( v, 1.0), -1.0 );
}

double coneDistanceScaled( const std::vector<double> &v,
                           double d0x, double d0y, 
                           double d1x, double d1y,
                           double scale)
{
  std::vector<double> c0(2), c1(2);
  c0[0] = d0x; c0[1] = d0y; normalize(c0);
  c1[0] = d1x; c1[1] = d1y; normalize(c1);

  std::vector<double> sum(2);
  sum[0] = d0x + d1x;
  sum[1] = d0y + d1y;
  normalize(sum);
  
  std::vector<double> vnorm = v;
  normalize(vnorm);

  double dmid = dot(vnorm, sum);
  double dcone = dot(c0, sum);
  if (dmid > dcone) 
  {
    //DBGA("Inside cone. Value: " << angularDistance( clamp( norm(v) / scale ) ) );
    return 0.1 - 0.1 * (std::min(1.0, norm(v) / scale )) ;
  }
  //DBGA("Outside cone. Value: " << std::min( angularDistance( clamp( dot(v, c0)/scale ) ), 
  //                                          angularDistance( clamp( dot(v, c1)/scale ) ) ) );
  return 0.1 + std::min( angularDistance(dot(vnorm, c0)), angularDistance(dot(vnorm, c1)) );

  //return std::min( angularDistance( clamp( dot(v, c0)/scale ) ), 
  //                 angularDistance( clamp( dot(v, c1)/scale ) ) );
}

bool Gripper31::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }
  
  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if ( mEnvelopingPoses[i].at(1) < -mEnvelopingPoses[i].at(0) ) continue;
    // if ( (fabs(mEnvelopingPoses[i].at(0) + mEnvelopingPoses[i].at(1)) / sqrt(2)) < 0.1  ) continue;
    //if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Norm for mixed: " << norm(jointResiduals));
    results.push_back(coneDistanceScaled(jointResiduals,  -1, -1,  0, -1,  2.0e6));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    DBGP("Norm for passive: " << norm(jointResiduals));
    results.push_back(coneDistanceScaled(jointResiduals,  -1, 0,  -1, -1,  2.0e6));

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    DBGP("Norm for active: " << norm(jointResiduals));
    results.push_back(coneDistanceScaled(jointResiduals,  1, 0.4,  1, 1,  2.0e6));
  }
  
  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Norm for mixed: " << norm(jointResiduals));
    results.push_back(multiplier * coneDistanceScaled(jointResiduals,  -0.4, -1,  1, -1,  2.0e6));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    DBGP("Norm for passive: " << norm(jointResiduals));
    results.push_back(multiplier * coneDistanceScaled(jointResiduals,  -1, 1,  -1, -0.4,  2.0e6));
   
    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    DBGP("Norm for active: " << norm(jointResiduals));
    results.push_back(multiplier * coneDistanceScaled(jointResiduals,  0, 1,  1, 0.4,  2.0e6));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper31::getGDGate()
{
  return 2;
}

double Gripper31::getGDEps()
{
  return 1.0e-5;
}

double Gripper31::getSaveThreshold()
{
  return 2;
}

//------------------------------------- Gripper32 -----------------------------------------------

bool Gripper32::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  -1, -1));

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  1, 0.6,  1, 1));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGA("Joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    DBGA("Normalized joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGA("Cone distance " << coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGA("");

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals,  -1, 1,  -1, -0.6));

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, 1,  1, 0.6));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper32::getGDGate()
{
  return 0.75;
}

double Gripper32::getGDEps()
{
  return 1.0e-5;
}

double Gripper32::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper33 -----------------------------------------------

bool Gripper33::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces dominate
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  1, 0.6,  0.6, 1));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    DBGP("Normalized joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("Cone distance " << coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("");

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals,  -1, 0, 0, -1));

    //active forces dominate
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, 1,  1, 0.6));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper33::getGDGate()
{
  return 0.75;
}

double Gripper33::getGDEps()
{
  return 1.0e-5;
}

double Gripper33::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper34 -----------------------------------------------

std::vector<double> Gripper34::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   -40,  -10, 
                   -40,  -10, 
                   -20,  -10, 
                   -50,  -10,  

                   -5,   1, 
                   -5,  1,
  
                    0.25};  
  std::vector<double> minPar;
  minPar.resize(mNumParameters, 0.0);
  memcpy(&minPar[0], &mins[0], mNumParameters*sizeof(double));
  return minPar;
}

std::vector<double> Gripper34::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 

                   -30,   0, 
                   -30,   0, 
                     0,   0, 
                   -30,   0,   

                   0,   5,  
                   0,  5,

                   5};
  std::vector<double> maxPar;
  maxPar.resize(mNumParameters, 0.0);
  memcpy(&maxPar[0], &maxs[0], mNumParameters*sizeof(double));
  return maxPar;
}

//------------------------------------- Gripper35 -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Tendon 1, 3 insertion points, x and y (6)
    - Tendon 2, 3 insertion points, x and y (6)
    - Tendon 1 stiffness (1)
    - Tendon 2 rest length (1)
    - Tendon 2 stiffness (1)
 */

std::vector<double> Gripper35::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   -40,  -7, 
                   -45,  -7,
                   -40,  -7, 

                   -10,  -7, 
                    -5,  -7,
                   -40,  -7,  

                   0.1, 

                    10,  
                   0.1};  
  std::vector<double> minPar;
  minPar.resize(mNumParameters, 0.0);
  memcpy(&minPar[0], &mins[0], mNumParameters*sizeof(double));
  return minPar;
}

std::vector<double> Gripper35::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 


                   -30,   0,
                   -35,   0,
                   -30,   0,  

                     0,   0, 
                     5,   0,
                   -30,   0,

                   5.0,  

                    30,
                   5.0};
  std::vector<double> maxPar;
  maxPar.resize(mNumParameters, 0.0);
  memcpy(&maxPar[0], &maxs[0], mNumParameters*sizeof(double));
  return maxPar;
}

bool Gripper35::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != mNumParameters) 
  {
    DBGA("Wrong number of parameters for Gripper35 optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 3 || 
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("Gripper35 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8],  parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[12], parameters[13] );

  //Tendon 2 insertion point 0, 1, 2
  setInsPtXY( mTendonVec[2]->getPermInsPoint(0), parameters[14], parameters[15] );
  setInsPtXY( mTendonVec[2]->getPermInsPoint(1), parameters[16], parameters[17] );
  setInsPtXY( mTendonVec[2]->getPermInsPoint(2), parameters[18], parameters[19] );

  //Tendon rest lengths and stiffness
  mTendonVec[1]->setStiffness (parameters[20] * 1.0e6);
  mTendonVec[2]->setDefaultRestLength(parameters[21]);
  mTendonVec[2]->setStiffness (parameters[22] * 1.0e6);

  return true;
}

bool Gripper35::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if ( mAllPoses[i].at(1) < -mAllPoses[i].at(0) ) continue;
    std::vector<double> tendonForces(1, 0.0);
    std::vector<double> finalPose(2, 0.0);
    std::vector<double> desiredConvergence(2, 0.0);
    bool convergence;
    
    tendonForces[0] = 2.0 * 1.0e6;
    desiredConvergence[0] =  0.75;
    desiredConvergence[1] = -0.75;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, true, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );

    tendonForces[0] = 5.0 * 1.0e6;
    desiredConvergence[0] =  1.74533;
    desiredConvergence[1] = -1.74533;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, true, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper35::getGDGate()
{
  return 7.0;
}

double Gripper35::getGDEps()
{
  return 1.0e-5;
}

double Gripper35::getSaveThreshold()
{
  return 7.0;
}

void Gripper35::updateJointValuesFromDynamics()
{
  HumanHand::updateJointValuesFromDynamics();
    
  assert( getNumChains() > 0 && getChain(0)->getNumJoints() > 1 && getNumDOF() > 1);
  getChain(0)->getJoint(1)->setMin( -getChain(0)->getJoint(0)->getVal() - 0.01 );
  getDOF(1)->updateMinMax();  
}

//------------------------------------- Gripper36 -----------------------------------------------

bool Gripper36::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mAllPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mAllPoses.size(); i++)
  {
    if ( mAllPoses[i].at(1) < -mAllPoses[i].at(0) ) continue;
    std::vector<double> tendonForces(1, 0.0);
    std::vector<double> finalPose(2, 0.0);
    std::vector<double> desiredConvergence(2, 0.0);
    bool convergence;

    tendonForces[0] = 0.0 * 1.0e6;
    desiredConvergence[0] =  0.0;
    desiredConvergence[1] = -0.0;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, true, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );
    
    tendonForces[0] = 2.0 * 1.0e6;
    desiredConvergence[0] =  0.75;
    desiredConvergence[1] = -0.75;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, true, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );

    tendonForces[0] = 5.0 * 1.0e6;
    desiredConvergence[0] =  1.74533;
    desiredConvergence[1] = -1.74533;
    if (!jointPoseGradientDescent(mAllPoses[i], tendonForces, finalPose, true, convergence)) return false;
    if (!convergence) results.push_back(1.0);
    else results.push_back( vectorDistance( finalPose, desiredConvergence ) );
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper36::getGDGate()
{
  return 11.0;
}

double Gripper36::getGDEps()
{
  return 1.0e-5;
}

double Gripper36::getSaveThreshold()
{
  return 11.0;
}

//------------------------------------- Gripper37 -----------------------------------------------

bool Gripper37::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces dominate
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  1, 0.6,  0.6, 1));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    DBGP("Normalized joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("Cone distance " << coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("");

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals,  -1, 0, 0, -1));

    //active forces dominate
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, 1,  1, 0.6));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper37::getGDGate()
{
  return 0.75;
}

double Gripper37::getGDEps()
{
  return 1.0e-5;
}

double Gripper37::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper38 -----------------------------------------------

bool Gripper38::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces dominate
    //they must be close to ratio needed for stable grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    //compute tendon forces and compare them to desired ratio
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(angularDistance(dot(jointResiduals, jointContactTorques)));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    DBGP("Joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    DBGP("Normalized joint residuals: " << jointResiduals[0] << " " << jointResiduals[1]);
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("Cone distance " << coneDistance(jointResiduals,  0, -1,  1, -1));
    DBGP("");

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals,  -1, 0, 0, -1));

    //active forces dominate
    //they must be below threshold for ejection
    //for the range in which we want transitions to enveloping grasps, must also be above horizontal
    //upper limit for cone is threshold for ejection
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }
    //lower limit for cone depends on whether we need enveloping grasps or not
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = 1; ll[1] = -1;}
    //compute tendon forces and compare them to desired range
    tendonForces[0] = 10.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper38::getGDGate()
{
  return 0.75;
}

double Gripper38::getGDEps()
{
  return 1.0e-5;
}

double Gripper38::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper39 -----------------------------------------------

bool Gripper39::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces only
    //they must be close to ratio needed for stable grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    //compute tendon forces and compare them to desired ratio
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(angularDistance(dot(jointResiduals, jointContactTorques)));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;

    //first compute joint torques for fingertip grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }

    //active forces must below grasp ratio, and positive if transition to enveloping grasp is required
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = 1; ll[1] = -1;}

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));

    //active forces dominate
    //the lower the value here, the better
    tendonForces[0] = mActiveForcesWeight;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10N forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));

    //only passive forces, must be as close as possible to the opposite of active forces
    //this ensures quick transitions and powerful extensions
    std::vector<double> negJointContactTorques(2);
    negJointContactTorques[0] = -jointContactTorques[0];
    negJointContactTorques[1] = -jointContactTorques[1];
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    //else results.push_back(multiplier * angularDistance(dot(jointResiduals, negJointContactTorques)));
    //try a more restrictive cone here
    //the more restrictive the cone, the more powerful active forces
    else results.push_back(multiplier * coneDistance(jointResiduals, 
                                                     -1, 0, 
                                                     mPassiveConeLimit, -1));
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper39::getGDGate()
{
  return 0.75;
}

double Gripper39::getGDEps()
{
  return 1.0e-5;
}

double Gripper39::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper41 -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Tendon 1, 3 insertion points, x and y (6)
    - Tendon 2, 3 insertion points, x and y (6)
    - Tendon 1 pre-tensioning (1)
    - Tendon 1 stiffness (1)
    - Tendon 2 pre-tensioning (1)
    - Tendon 2 stiffness (1)
 */

std::vector<double> Gripper41::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   -40,  -7, 
                   -45,  -7,
                   -40,  -7, 

                   -10,  -7, 
                    -5,  -7,
                   -40,  -7,  

                     0,
                   0.1, 

                     0,  
                   0.1};  
  std::vector<double> minPar;
  minPar.resize(mNumParameters, 0.0);
  memcpy(&minPar[0], &mins[0], mNumParameters*sizeof(double));
  return minPar;
}

std::vector<double> Gripper41::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 


                   -30,   0,
                   -35,   0,
                   -30,   0,  

                     0,   0, 
                     5,   0,
                   -30,   0,

                    20,
                   5.0,  

                    20,
                   5.0};
  std::vector<double> maxPar;
  maxPar.resize(mNumParameters, 0.0);
  memcpy(&maxPar[0], &maxs[0], mNumParameters*sizeof(double));
  return maxPar;
}

bool Gripper41::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != mNumParameters) 
  {
    DBGA("Wrong number of parameters for Gripper41 optimization");
    return false;
  }

  if (mTendonVec.size() < 3 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 || 
      mTendonVec[1]->getNumPermInsPoints() != 3 || 
      mTendonVec[2]->getNumPermInsPoints() != 3)
  {
    DBGA("Gripper41 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Tendon 1 insertion points 0, 1, 2
  setInsPtXY( mTendonVec[1]->getPermInsPoint(0), parameters[8],  parameters[9] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(1), parameters[10], parameters[11] );
  setInsPtXY( mTendonVec[1]->getPermInsPoint(2), parameters[12], parameters[13] );

  //Tendon 2 insertion point 0, 1, 2
  setInsPtXY( mTendonVec[2]->getPermInsPoint(0), parameters[14], parameters[15] );
  setInsPtXY( mTendonVec[2]->getPermInsPoint(1), parameters[16], parameters[17] );
  setInsPtXY( mTendonVec[2]->getPermInsPoint(2), parameters[18], parameters[19] );

  //Tendon pre-tensioning and stiffness
  mTendonVec[1]->setPreTensionLength(parameters[20]);
  mTendonVec[1]->setStiffness (parameters[21] * 1.0e6);
  mTendonVec[2]->setPreTensionLength(parameters[22]);
  mTendonVec[2]->setStiffness (parameters[23] * 1.0e6);

  return true;
}

void Gripper41::updateJointValuesFromDynamics()
{
  HumanHand::updateJointValuesFromDynamics();
    
  assert( getNumChains() > 0 && getChain(0)->getNumJoints() > 1 && getNumDOF() > 1);
  getChain(0)->getJoint(1)->setMin( -getChain(0)->getJoint(0)->getVal() - 0.01 );
  getDOF(1)->updateMinMax();  
}

//------------------------------------- Gripper42 -----------------------------------------------

bool Gripper42::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces only
    //they must be close to ratio needed for stable grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    //compute tendon forces and compare them to desired ratio
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(angularDistance(dot(jointResiduals, jointContactTorques)));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;

    //first compute joint torques for fingertip grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }

    //active forces must below grasp ratio, and positive if transition to enveloping grasp is required
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = 1; ll[1] = -1;}

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));

    //active forces dominate
    //the lower the value here, the better
    tendonForces[0] = mActiveForcesWeight;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10N forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
    
    //both active and passive forces
    tendonForces[0] = 2.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  0, -1,  1, -1));

    //only passive forces, must be as close as possible to the opposite of active forces
    //this ensures quick transitions and powerful extensions
    std::vector<double> negJointContactTorques(2);
    negJointContactTorques[0] = -jointContactTorques[0];
    negJointContactTorques[1] = -jointContactTorques[1];
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    //else results.push_back(multiplier * angularDistance(dot(jointResiduals, negJointContactTorques)));
    //try a more restrictive cone here
    //the more restrictive the cone, the more powerful active forces
    else results.push_back(multiplier * coneDistance(jointResiduals, 
                                                     -1, 0, 
                                                     mPassiveConeLimit, -1));
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper42::getGDGate()
{
  return 0.75;
}

double Gripper42::getGDEps()
{
  return 1.0e-5;
}

double Gripper42::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper44 -----------------------------------------------

/*! Parameters:
    - Tendon 0, 4 insertion points, x and y (8)
    - Joint 1 spring pre-tensioning (1)
    - Joint 1 spring stiffness (1)
    - Joint 2 spring pre-tensioning (1)
    - Joint 2 spring stiffness (1)
 */

std::vector<double> Gripper44::getParameterMin()
{  
  double mins[] = {-40,  0, 
                   -40,  0, 
                   -10,  0, 
                   -40,  0, 

                   190.0 * M_PI/180.0,
                   3.6, 

                     0,  
                    1.0};  
  std::vector<double> minPar;
  minPar.resize(mNumParameters, 0.0);
  memcpy(&minPar[0], &mins[0], mNumParameters*sizeof(double));
  return minPar;
}

std::vector<double> Gripper44::getParameterMax()
{
  double maxs[] = {-30,  7, 
                   -30,  7,   
                     0,  7, 
                   -30,  7, 

                   190.0 * M_PI/180.0,
                   3.6,  

                    M_PI,
                   10.0};
  std::vector<double> maxPar;
  maxPar.resize(mNumParameters, 0.0);
  memcpy(&maxPar[0], &maxs[0], mNumParameters*sizeof(double));
  return maxPar;
}

bool Gripper44::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (parameters.size() != mNumParameters) 
  {
    DBGA("Wrong number of parameters for Gripper44 optimization");
    return false;
  }

  if (mTendonVec.size() < 1 || 
      mTendonVec[0]->getNumPermInsPoints() != 6 ||
      getChain(0)->getNumJoints() < 2)
  {
    DBGA("Gripper44 design has changed; optimization no longer applies");
    return false;
  }

  //Tendon 0 insertion points 2, 3, 4, 5
  setInsPtXY( mTendonVec[0]->getPermInsPoint(2), parameters[0], parameters[1] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(3), parameters[2], parameters[3] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(4), parameters[4], parameters[5] );
  setInsPtXY( mTendonVec[0]->getPermInsPoint(5), parameters[6], parameters[7] );

  //Joint spring pre-tensioning and stiffness
  getChain(0)->getJoint(0)->setRestValue( getChain(0)->getJoint(0)->getMin() - parameters[8] );
  getChain(0)->getJoint(0)->setSpringStiffness( parameters[9] * 1.0e6 );
  getChain(0)->getJoint(1)->setRestValue( getChain(0)->getJoint(1)->getMin() - parameters[10] );
  getChain(0)->getJoint(1)->setSpringStiffness( parameters[11] * 1.0e6 );

  return true;
}

void Gripper44::updateJointValuesFromDynamics()
{
  HumanHand::updateJointValuesFromDynamics();
    
  assert( getNumChains() > 0 && getChain(0)->getNumJoints() > 1 && getNumDOF() > 1);
  getChain(0)->getJoint(1)->setMin( -getChain(0)->getJoint(0)->getVal() - 0.01 );
  getDOF(1)->updateMinMax();  
}

//------------------------------------- Gripper45 -----------------------------------------------

bool Gripper45::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces; closing regime
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces
    tendonForces[0] = 0.0;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces only
    //they must be close to ratio needed for stable grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    //compute tendon forces and compare them to desired ratio
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(angularDistance(dot(jointResiduals, jointContactTorques)));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;

    //first compute joint torques for fingertip grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }

    //active forces must below grasp ratio, and positive if transition to enveloping grasp is required
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = 1; ll[1] = -1;}

    //only active forces
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));

    //active forces dominate, regime for grasping
    tendonForces[0] = mActiveForceGrasp;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10N forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
    
    //both active and passive forces, regime for closing
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  mConeLimitClose, -1,  1, -1));

    //only passive forces, must be as close as possible to the opposite of active forces
    //this ensures quick transitions and powerful extensions
    std::vector<double> negJointContactTorques(2);
    negJointContactTorques[0] = -jointContactTorques[0];
    negJointContactTorques[1] = -jointContactTorques[1];
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    //else results.push_back(multiplier * angularDistance(dot(jointResiduals, negJointContactTorques)));
    //try a more restrictive cone here
    //the more restrictive the cone, the more powerful active forces
    else results.push_back(multiplier * coneDistance(jointResiduals, 
                                                     -1, 0, 
                                                     mConeLimitExtend, -1));
  }
  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper45::getGDGate()
{
  return 0.75;
}

double Gripper45::getGDEps()
{
  return 1.0e-5;
}

double Gripper45::getSaveThreshold()
{
  return 0.75;
}

//------------------------------------- Gripper48 -----------------------------------------------

bool Gripper48::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    if ( minInsPointDistance() < mMinInsPointDistance ) 
    {
      results.insert(results.begin(), std::numeric_limits<double>::max()); 
      return true;
    }

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces; closing regime
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  0.4, -1));

    //only passive forces; opening regime
    tendonForces[0] = 0.0;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces only; squeezing regime
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(angularDistance(dot(jointResiduals, jointContactTorques)));
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    if ( minInsPointDistance() < mMinInsPointDistance ) 
    {
      results.insert(results.begin(), std::numeric_limits<double>::max()); 
      return true;
    }

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;

    //first compute joint torques for fingertip grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }

    //active forces must below grasp ratio, and positive if transition to enveloping grasp is required
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = 1; ll[1] = -1;}

    //only active forces, squeezing regime
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));

    //active forces dominate, enveloping regime
    tendonForces[0] = mActiveForceGrasp;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10N forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
    
    //both active and passive forces, closing regime
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  mConeLimitClose, -1,  1, -1));

    //only passive forces, opening regime
    //this ensures quick transitions and powerful extensions
    std::vector<double> negJointContactTorques(2);
    negJointContactTorques[0] = -jointContactTorques[0];
    negJointContactTorques[1] = -jointContactTorques[1];
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals, -1, 0, mConeLimitExtend, -1));
  }

  if ( mUseSquareRoot)
  {
    for (size_t i=0; i<results.size(); i++)
    {
      results[i] = sqrt( results[i] );
    }
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper48::getGDGate()
{
  if (mUseSquareRoot) return sqrt(0.75);
  return 0.75;
}

double Gripper48::getGDEps()
{
  if (mUseSquareRoot) return sqrt(1.0e-5);
  return 1.0e-5;
}

double Gripper48::getSaveThreshold()
{
  if (mUseSquareRoot) return sqrt(0.75);
  return 0.75;
}

//------------------------------------- Gripper51 -----------------------------------------------

bool Gripper51::performOptimization(const std::vector<double> &parameters, std::vector<double> &results)
{
  if (!parameters.empty() && !setOptimizationParameters(parameters)) return false;
  resetPose();
  results.clear();
  if (insPointInsideWrapper())
  {
    results.resize( mFingertipPoses.size() + 1, std::numeric_limits<double>::max());
    return true;
  }

  for (size_t i=0; i<mEnvelopingPoses.size(); i++)
  {
    if (!setPose(mEnvelopingPoses[i])) return false;
    if ( minInsPointDistance() < mMinInsPointDistance ) 
    {
      results.insert(results.begin(), std::numeric_limits<double>::max()); 
      return true;
    }

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;

    //both active and passive forces; closing regime
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(coneDistance(jointResiduals,  -1, -1,  mConeLimitOffaxisClose, -1));

    //only passive forces; opening regime
    tendonForces[0] = 0.0;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(1.0);
    else results.push_back(coneDistance(jointResiduals,  -1, 0,  0, -1));

    //active forces only; squeezing regime
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive tendons failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back( std::max(0.0, mActiveWeight * ( angularDistance(dot(jointResiduals, 
                                                                           jointContactTorques)) - mActiveLeeway) ) );
  }

  for (size_t i=0; i<mFingertipPoses.size(); i++)
  {
    if (!setPose(mFingertipPoses[i])) return false;
    if ( minInsPointDistance() < mMinInsPointDistance ) 
    {
      results.insert(results.begin(), std::numeric_limits<double>::max()); 
      return true;
    }

    double unbalanced_magnitude;
    std::vector<double> jointResiduals;
    std::vector<double> tendonForces(1, 0.0);
    int result;
    double multiplier = 1.0;

    //first compute joint torques for fingertip grasps
    std::list<Contact*> contacts;
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    std::vector<double> jointContactTorques;
    result = contactTorques(contacts, jointContactTorques);
    if (result){DBGA("Joint torque computation failed");return false;}
    if (!normalize(jointContactTorques)){DBGA("Zero norm for joint contact torques"); return false;}
    if ( jointContactTorques[0] < 0.0 || jointContactTorques[1] < 0.0 )
    {
      DBGA("Negative joint contact torque encountered");
      return false;
    }

    //active forces must below grasp ratio, and positive if transition to enveloping grasp is required
    std::vector<double> ll(2);
    if (mFingertipPoses[i].at(0) < 1.30) {ll[0] = 1; ll[1] = 0;}
    else {ll[0] = mConeLimitClose; ll[1] = -1;}

    //only active forces, squeezing regime
    tendonForces[0] = 2.0 * 1.0e6;
    std::set<size_t> noPassiveTendons;
    result = tendonEquilibrium(mActiveTendons, noPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude, false);
    if (result) {DBGA("Tendon equilibrium with 2.0N force and no passive forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));

    //active forces dominate, enveloping regime
    tendonForces[0] = mActiveForceGrasp;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 10N forces failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals, ll,  jointContactTorques));
    
    //both active and passive forces, closing regime
    tendonForces[0] = mActiveForceClose;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 2.0N force failed");return false;}
    if (!normalize(jointResiduals)) { DBGA("Zero norm for joint residual"); return false;}
    results.push_back(multiplier * coneDistance(jointResiduals,  mConeLimitClose, -1,  1, -1));

    //only passive forces, opening regime
    //this ensures quick transitions and powerful extensions
    std::vector<double> negJointContactTorques(2);
    negJointContactTorques[0] = -jointContactTorques[0];
    negJointContactTorques[1] = -jointContactTorques[1];
    tendonForces[0] = 0.0 * 1.0e6;
    result = tendonEquilibrium(mActiveTendons, mPassiveTendons, false, tendonForces, 
                               jointResiduals, unbalanced_magnitude);
    if (result) {DBGA("Tendon equilibrium with 0.0N force failed");return false;}
    if (!normalize(jointResiduals)) results.push_back(multiplier * 1.0);
    else results.push_back(multiplier * coneDistance(jointResiduals, -1, 0, mConeLimitExtend, -1));
  }

  if ( mUseSquareRoot)
  {
    for (size_t i=0; i<results.size(); i++)
    {
      results[i] = sqrt( results[i] );
    }
  }

  results.insert(results.begin(), norm(results));
  return true;  
}

double Gripper51::getGDGate()
{
  if (mUseSquareRoot) return sqrt(0.75);
  return 0.75;
}

double Gripper51::getGDEps()
{
  return 1.0e-8;
}

double Gripper51::getSaveThreshold()
{
  if (mUseSquareRoot) return sqrt(0.1);
  return 0.1;
}

bool Gripper55::setOptimizationParameters(const std::vector<double> &parameters)
{
  if (!Gripper51::setOptimizationParameters(parameters)) return false;
  //getChain(0)->getJoint(1)->setSpringStiffness( 3.6 * 1.0e6 );
  return true;
}

std::vector<double> Gripper56::getParameterMin()
{ 
  std::vector<double> params = Gripper51::getParameterMin();
  params.at(11) = 3.6;
  return params;
}

std::vector<double> Gripper56::getParameterMax()
{
  std::vector<double> params = Gripper51::getParameterMax();
  params.at(11) = 3.6;
  return params;
}

std::vector<double> Gripper58::getParameterMin()
{ 
  std::vector<double> params = Gripper51::getParameterMin();
  params.at(10) = 2.79;
  params.at(11) = 3.59;
  return params;
}

std::vector<double> Gripper58::getParameterMax()
{
  std::vector<double> params = Gripper51::getParameterMax();
  params.at(10) = 2.79;
  params.at(11) = 3.59;
  return params;
}

std::vector<double> Gripper59::getParameterMin()
{ 
  std::vector<double> params = Gripper51::getParameterMin();
  params.at(10) = 1.22;
  params.at(11) = 3.6;
  return params;
}

std::vector<double> Gripper59::getParameterMax()
{
  std::vector<double> params = Gripper51::getParameterMax();
  params.at(10) = 1.22;
  params.at(11) = 3.6;
  return params;
}

std::vector<double> Gripper60::getParameterMin()
{ 
  std::vector<double> params = Gripper51::getParameterMin();
  params.at(10) = 2.79;
  params.at(11) = 2.70;
  return params;
}

std::vector<double> Gripper60::getParameterMax()
{
  std::vector<double> params = Gripper51::getParameterMax();
  params.at(10) = 2.79;
  params.at(11) = 2.70;
  return params;
}

std::vector<double> Gripper61::getParameterMin()
{ 
  std::vector<double> params = Gripper51::getParameterMin();
  params.at(10) = 2.79;
  params.at(11) = 2.25;
  return params;
}

std::vector<double> Gripper61::getParameterMax()
{
  std::vector<double> params = Gripper51::getParameterMax();
  params.at(10) = 2.79;
  params.at(11) = 2.25;
  return params;
}

}
