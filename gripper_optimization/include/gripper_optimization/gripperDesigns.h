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

#ifndef _GRIPPER_DESIGNS_H_
#define _GRIPPER_DESIGNS_H_

#include <vector>

#include "humanHand.h"

namespace gripper_optimization {

class GripperDesign : public HumanHand
{
protected:
  std::vector< std::vector<double> > mFingertipPoses;
  std::vector< std::vector<double> > mEnvelopingPoses;
  std::vector< std::vector<double> > mAllPoses;
  std::set<size_t> mActiveTendons;
  std::set<size_t> mPassiveTendons;
  size_t mNumParameters;

  //! Computes tendon equilibrium in the current pose with the current parameters
  virtual bool computeTendonEquilibrium(double &result);

  //! Computes contact equilibrium in the current pose with the current parameters
  virtual bool computeContactEquilibrium(bool enveloping, double &result);

  virtual std::list<Contact*> getEnvelopingContacts()
  {
    std::list<Contact*> contacts; 
    contacts.push_back(getChain(0)->getLink(0)->getVirtualContacts().front());
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    return contacts;
  }

  virtual std::list<Contact*> getFingertipContacts()
  {
    std::list<Contact*> contacts; 
    contacts.push_back(getChain(0)->getLink(1)->getVirtualContacts().front());
    return contacts;
  }
  
public:
  GripperDesign(World *w, const char *name);
  virtual ~GripperDesign(){}
  
  //! Makes all links transparent
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

  bool setPose(const std::vector<double> &pose);

  void resetPose();

  //! Set particular values for the parameters we are optimizing
  virtual bool setOptimizationParameters(const std::vector<double>&) {return false;}

  virtual std::vector<double> getParameterMin() = 0;
  virtual std::vector<double> getParameterMax() = 0;

  std::vector< std::vector<double> > getFingertipPoses() {return mFingertipPoses;}
  std::vector< std::vector<double> > getEnvelopingPoses() {return mEnvelopingPoses;}

  //! Computes full optimization on with the given parameters
  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  static std::string getDBName(std::string graspit_name);
  static std::string getGraspitFile(std::string db_name);

  //! Follows the gradient of the joint residuals until the hand settles in a new pose
  bool jointPoseGradientDescent(const std::vector<double> &starting_pose,
                                const std::vector<double> &tendon_forces,
                                std::vector<double> &final_pose,
                                bool restrict_above_diagonal,
                                bool &convergence);


  //! This is the value for the optimization result above which we don't even start gradient descent
  virtual double getGDGate(){return 20.0 * 1.0e6;}
  //! This is the minimum improvement for GD to continue
  virtual double getGDEps(){return 1.0e-4 * 1.0e6;}
  //! This is the value below which we save the result in the database
  virtual double getSaveThreshold(){return 5.0 * 1.0e6;}

  //! Registers the world element creators with GraspIt's World Element Factory
  static void registerCreators();
};

// Only optimizes the flexor tendon
class FlexorOnlyGripper : public GripperDesign
{
protected:
public:
  FlexorOnlyGripper(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

// Optimizes both flexor and extensor
class FlexorExtensorGripper : public GripperDesign
{
protected:
  bool mOptimizeStiffness;
public:
  FlexorExtensorGripper(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mOptimizeStiffness = false;
    mNumParameters = 20;
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

//! Two extensors, no wrappers, also optimizes relative extensor stiffness
class Flexor2ExtensorsNoWrappers : public GripperDesign
{
protected:
public:
  Flexor2ExtensorsNoWrappers(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

//! Same as Flexor2ExtensorsNoWrappers, but three insertion points for each tendon at each join, fully optimized
class FlexorExtensors3Points : public GripperDesign
{
public:
  FlexorExtensors3Points(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

//! Two extensors only are optimized, plus relative stiffness, no wrappers, but assumes a flexor tendon as well
class ExtensorsGripper : public GripperDesign
{
public:
  ExtensorsGripper(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

// Places insertion point of extensor on lower edge of wrapper
class Gripper7 : public FlexorExtensorGripper
{
public:
  Gripper7(World *w, const char *name) : FlexorExtensorGripper(w, name) {}
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

// Dedicated joint 0 extensor with insertion point on lower edge of wrapper
class Gripper9 : public Gripper7
{
public:
  Gripper9(World *w, const char *name) : Gripper7(w, name) 
  {
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
};

// Does contact equilibrium on fingertip poses
class Gripper10 : public FlexorOnlyGripper
{
public:
  Gripper10(World *w, const char *name) : FlexorOnlyGripper(w, name) {}
  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);
};

// Does contact equilibrium on enveloping poses
class Gripper11 : public FlexorOnlyGripper
{
public:
  Gripper11(World *w, const char *name) : FlexorOnlyGripper(w, name) {}
  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);
};

// Does contact equilibrium on both fingertip and enveloping poses
class Gripper12 : public FlexorOnlyGripper
{
public:
  Gripper12(World *w, const char *name) : FlexorOnlyGripper(w, name) {}
  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  //! This is the value for the optimization result above which we don't even start gradient descent
  virtual double getGDGate(){return 40.0 * 1.0e6;}
  //! This is the minimum improvement for GD to continue
  virtual double getGDEps(){return 1.0e-5 * 1.0e6;}
  //! This is the value below which we save the result in the database
  virtual double getSaveThreshold(){return 10.0 * 1.0e6;}
};

// Also uses the wrapper on the distal joint
class Gripper13 : public Gripper12
{
public:
  Gripper13(World *w, const char *name) : Gripper12(w, name) {}

  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};

// Optimizes extensors only plus radius of proximal pulley, based on results from Gripper13
class Gripper14 : public FlexorExtensorGripper
{
public:
  Gripper14(World *w, const char *name) : FlexorExtensorGripper(w, name) 
  {
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  virtual double getGDGate(){return 1.0 * 1.0e6;}
  virtual double getGDEps(){return 1.0e-6 * 1.0e6;}
  virtual double getSaveThreshold(){return 0.5 * 1.0e6;}
};

//! Optimizes gradient of a single unbalanced force for a set force value
class Gripper15 : public Gripper9
{
public:
  Gripper15(World *w, const char *name) : Gripper9(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Optimizes gradient of multiple unbalanced forces
class Gripper16 : public Gripper9
{
public:
  Gripper16(World *w, const char *name) : Gripper9(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Optimizes gradient of multiple unbalanced forces and also extensor stiffness
class Gripper17 : public Gripper16
{
public:
  Gripper17(World *w, const char *name) : Gripper16(w, name) {}

  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();

};

//! Weighs samples closer to desired equilibrium point differently
class Gripper18 : public Gripper17
{
protected:
  virtual double weightFunction(double d);

public:
  Gripper18(World *w, const char *name) : Gripper17(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Used squared weighting and more samples compared to Gripper18
class Gripper19 : public Gripper18
{
protected:
  virtual double weightFunction(double d);

public:
  Gripper19(World *w, const char *name) : Gripper18(w, name) {}
};

//! Optimizes points along the fingertip line to point in the right direction
class Gripper20 : public Gripper17
{
protected:
  virtual double getDotValue(const std::vector<double> &jointResiduals, size_t pose_index);
public:
  Gripper20(World *w, const char *name) : Gripper17(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Optimizes points everywhere to point TO the right line
class Gripper21 : public Gripper17
{
protected:
  virtual double getDotValue(const std::vector<double> &jointResiduals, size_t pose_index);
public:
  Gripper21(World *w, const char *name) : Gripper17(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Gradient descent from everywhere to the right line
class Gripper22 : public Gripper17
{
public:
  Gripper22(World *w, const char *name) : Gripper17(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Optimizes with gradient descent to specific points on the line using results from Gripper13 for flexor
class Gripper23 : public FlexorExtensorGripper
{
public:
  Gripper23(World *w, const char *name) : FlexorExtensorGripper(w, name)
  {
    mPassiveTendons.insert(2);
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Complete optimization of free poses (GD to specific points), enveloping and tip grasps no wrappers
class Gripper24 : public Flexor2ExtensorsNoWrappers
{
public:
  Gripper24(World *w, const char *name) : Flexor2ExtensorsNoWrappers(w, name){}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! All poses point to line, fingertip poses point down the line, 
//! no wrappers, based on results from Gripper12FlexOnly
class Gripper25 : public ExtensorsGripper
{
protected:
  double getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index);

public:
  Gripper25(World *w, const char *name) : ExtensorsGripper(w, name){}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Complete optimization of free poses, enveloping and tip grasps no wrappers
class Gripper26 : public Flexor2ExtensorsNoWrappers
{
protected:
  double getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index);

public:
  Gripper26(World *w, const char *name) : Flexor2ExtensorsNoWrappers(w, name){}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Complete optimization of free poses, enveloping and tip grasps no wrappers, but only 1N force
class Gripper27 : public FlexorExtensors3Points
{
protected:
  double getDotValueToLine(const std::vector<double> &jointResiduals, size_t pose_index);

public:
  Gripper27(World *w, const char *name) : FlexorExtensors3Points(w, name){}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Also requests that small forces bring gripper back up the line
class Gripper28 : public Gripper27
{
public:
  Gripper28(World *w, const char *name) : Gripper27(w, name){}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

class Gripper30 : public FlexorExtensorGripper
{
public:
  Gripper30(World *w, const char *name) : FlexorExtensorGripper(w, name)
  {
    mOptimizeStiffness = true;
    mNumParameters = 21;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();

  //! Does not allow the distal joint to extend more than the negative of the proximal joint
  //! Essentially simulates a four-bar linkage
  virtual void updateJointValuesFromDynamics();
};

class Gripper31 : public Gripper30
{
public:
  Gripper31(World *w, const char *name) : Gripper30(w, name)
  {
    mOptimizeStiffness = true;
    mNumParameters = 21;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different limits and optimization weights than Gripper30
class Gripper32 : public Gripper30
{
public:
  Gripper32(World *w, const char *name) : Gripper30(w, name)
  {
    mOptimizeStiffness = true;
    mNumParameters = 21;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! More permissive enveloping poses and dominant active forces (not alone)
//! Updated to new gradients of movement along constraint
class Gripper33 : public Gripper30
{
public:
  Gripper33(World *w, const char *name) : Gripper30(w, name)
  {
    mOptimizeStiffness = true;
    mNumParameters = 21;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Wider range of parameters than Gripper33. Also optimizes with fewer fingertip poses
class Gripper34 : public Gripper33
{
public:
  Gripper34(World *w, const char *name) : Gripper33(w, name)
  {
    mOptimizeStiffness = true;
    mNumParameters = 21;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();
};


//! Two individual extensors with 3 insertion points, no wrappers
//! Optimizes 2N to gradient descent to specific point on the line and 5N to bottom right corner
class Gripper35 : public GripperDesign
{
protected:
public:
  Gripper35(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mPassiveTendons.insert(2);
    mNumParameters = 23;
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();

  //! Does not allow the distal joint to extend more than the negative of the proximal joint
  //! Essentially simulates a four-bar linkage
  virtual void updateJointValuesFromDynamics();

};

//! Also optimizes 0N to gradient descent to upper left corner
class Gripper36 : public Gripper35
{
protected:
public:
  Gripper36(World *w, const char *name) : Gripper35(w, name) {}
  
  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
  
};

//! Optimizes directions of arrows at fingertip and enveloping poses as opposed to full GD
class Gripper37 : public Gripper35
{
protected:
public:
  Gripper37(World *w, const char *name) : Gripper35(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Optimizes directions of arrows at fingertip and enveloping poses also taking into account
//! directions needed for grasps
class Gripper38 : public Gripper35
{
protected:
public:
  Gripper38(World *w, const char *name) : Gripper35(w, name) {}

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different optimizations, still taking into accoung grasping ratios, but with arrows in 
//! different directions
class Gripper39 : public Gripper35
{
protected:
  double mPassiveConeLimit;
  double mActiveForcesWeight;
public:
  Gripper39(World *w, const char *name) : Gripper35(w, name) 
  {
    mPassiveConeLimit = -0.4;
    mActiveForcesWeight = 10.0 * 1.0e6;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different cone limit for passive forces and weight for active forces
class Gripper40 : public Gripper39
{
public:
  Gripper40(World *w, const char *name) : Gripper39(w, name) 
  {
    mPassiveConeLimit = -0.2;
    mActiveForcesWeight = 10.0 * 1.0e6;
  }
};

//! Two individual extensors with 3 insertion points, no wrappers
//! Use pre-tensioning on both extensor tendons
class Gripper41 : public GripperDesign
{
protected:
public:
  Gripper41(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mPassiveTendons.insert(1);
    mPassiveTendons.insert(2);
    mNumParameters = 24;
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  //! Does not allow the distal joint to extend more than the negative of the proximal joint
  //! Essentially simulates a four-bar linkage
  virtual void updateJointValuesFromDynamics();

};

//! Same optimization as Gripper39, but on the base of Gripper41
class Gripper42 : public Gripper41
{
protected:
  double mPassiveConeLimit;
  double mActiveForcesWeight;
public:
  Gripper42(World *w, const char *name) : Gripper41(w, name) 
  {
    mPassiveConeLimit = -0.4;
    mActiveForcesWeight = 10.0 * 1.0e6;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different cone limit and active force weight
class Gripper43 : public Gripper42
{
public:
  Gripper43(World *w, const char *name) : Gripper42(w, name) 
  {
    mPassiveConeLimit = -0.6;
    mActiveForcesWeight = 8.0 * 1.0e6;
  }
};

//! Linear springs with pre-tensioning instead of extensor tendons
class Gripper44 : public GripperDesign
{
protected:
public:
  Gripper44(World *w, const char *name) : GripperDesign(w, name) 
  {
    mActiveTendons.insert(0);
    mNumParameters = 12;
  }
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);
  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

  //! Does not allow the distal joint to extend more than the negative of the proximal joint
  //! Essentially simulates a four-bar linkage
  virtual void updateJointValuesFromDynamics();
};

//! Same optimization as Gripper43, but on the base of Gripper44
class Gripper45 : public Gripper44
{
protected:
  double mConeLimitClose;
  double mConeLimitExtend;

  double mActiveForceClose;
  double mActiveForceGrasp;
public:
  Gripper45(World *w, const char *name) : Gripper44(w, name) 
  {
    mConeLimitClose = 0.0;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 8.0 * 1.0e6;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different parameters than Gripper45
class Gripper46 : public Gripper45
{
public:
  Gripper46(World *w, const char *name) : Gripper45(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 6.0 * 1.0e6;
  }
};

//! Different parameters than Gripper45
class Gripper47 : public Gripper45
{
public:
  Gripper47(World *w, const char *name) : Gripper45(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 1.0 * 1.0e6;
    mActiveForceGrasp = 3.0 * 1.0e6;
  }
};

//! Similar to Gripper45, but does not allow insertion point to be too close to each other in any pose
//! and uses sqrt(dot) to penalize small angular errors
class Gripper48 : public Gripper44
{
protected:
  double mMinInsPointDistance;

  double mConeLimitClose;
  double mConeLimitExtend;

  double mActiveForceClose;
  double mActiveForceGrasp;

  bool mUseSquareRoot;
public:
  Gripper48(World *w, const char *name) : Gripper44(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

//! Different parameters than Gripper48, identical to Gripper46 but with distance between insertion points
class Gripper49 : public Gripper48
{
public:
  Gripper49(World *w, const char *name) : Gripper48(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 6.0 * 1.0e6;

    mMinInsPointDistance = 2.0;
    mUseSquareRoot = false;
  }
};

//! Identical to Gripper49 but using sqrt
class Gripper50 : public Gripper48
{
public:
  Gripper50(World *w, const char *name) : Gripper48(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 6.0 * 1.0e6;

    mMinInsPointDistance = 2.0;
    mUseSquareRoot = true;
  }
};


//! Similar to Gripper48, but more relaxed for active forces in fingertip grasps where no transition is needed
//! Also more relaxed for enveloping grasps, with the goal of getting a perfect 0 result
class Gripper51 : public Gripper44
{
protected:
  double mMinInsPointDistance;

  double mConeLimitClose;
  double mConeLimitExtend;

  double mConeLimitOffaxisClose;

  double mActiveForceClose;
  double mActiveForceGrasp;

  bool mUseSquareRoot;

  double mActiveLeeway;
  double mActiveWeight;
public:
  Gripper51(World *w, const char *name) : Gripper44(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mConeLimitOffaxisClose = 0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.0005;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual bool performOptimization(const std::vector<double> &parameters, std::vector<double> &results);

  virtual double getGDGate();
  virtual double getGDEps();
  virtual double getSaveThreshold();
};

class Gripper52 : public Gripper51
{
public:
  Gripper52(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.6;

    mConeLimitOffaxisClose = 0.6;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 6.0 * 1.0e6;

    mActiveLeeway = 0.0005;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }
};

//less strict coneLimitClose, but smaller force for enveloping
class Gripper53 : public Gripper51
{
public:
  Gripper53(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.5;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.0005;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }
};

//More active leeway
class Gripper54 : public Gripper51
{
public:
  Gripper54(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.5;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }
};

//less strict coneLimitExtend
class Gripper55 : public Gripper51
{
public:
  Gripper55(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  //used temporarily to force spring to actual catalog value
  virtual bool setOptimizationParameters(const std::vector<double> &parameters);

};

//like Gripper55, but with both springs at catalog values
class Gripper56 : public Gripper51
{
public:
  Gripper56(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

};

//like Gripper56, but with more space between insertion points
class Gripper57 : public Gripper56
{
public:
  Gripper57(World *w, const char *name) : Gripper56(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 4.0;
    mUseSquareRoot = true;
  }
};

//like Gripper56, but with both springs and rest positions at catalog values
class Gripper58 : public Gripper51
{
public:
  Gripper58(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

};

//like Gripper58, but with a different spring at the distal joint
class Gripper59 : public Gripper51
{
public:
  Gripper59(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

};

//like Gripper58, but with a different spring at the distal joint
class Gripper60 : public Gripper51
{
public:
  Gripper60(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

};

//like Gripper58, but with a different spring at the distal joint
class Gripper61 : public Gripper51
{
public:
  Gripper61(World *w, const char *name) : Gripper51(w, name) 
  {
    mConeLimitClose = 0.2;
    mConeLimitExtend = -0.4;

    mConeLimitOffaxisClose = 0.8;

    mActiveForceClose = 2.0 * 1.0e6;
    mActiveForceGrasp = 5.0 * 1.0e6;

    mActiveLeeway = 0.00075;
    mActiveWeight = 1.0;

    mMinInsPointDistance = 3.0;
    mUseSquareRoot = true;
  }

  virtual std::vector<double> getParameterMin();
  virtual std::vector<double> getParameterMax();

};

}

#endif
