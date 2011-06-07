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
// Author(s): Matei T. Ciocarlie
//
// $Id: gfoDlg.cpp,v 1.6 2009/09/19 00:35:12 cmatei Exp $
//
//######################################################################

#include "gfoDlg.h"

#include <QInputDialog>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "grasp.h"
#include "body.h"
#include "matrix.h"
#include "mainWindow.h"

// for hand-specific optimizations, might be temporary
#include "mcGrip.h"
#include "humanHand.h"
#include "gripperDesigns.h"

#include "debug.h"

#define PROF_ENABLED
#include "profiling.h"

GFODlg::GFODlg(MainWindow *mw, Hand *h, QWidget *parent) : mMainWindow(mw), mHand(h), QDialog(parent)
{
	setupUi(this);
	statusLabel->setText("Status: optimization off");
	if (mHand->inherits("HumanHand")) {
		optimizationTypeBox->insertItem("Tendon equilibrium");
		optimizationTypeBox->insertItem("Contact equilibrium");
		optimizationTypeBox->insertItem("Equilibrium plot");
		optimizationTypeBox->insertItem("Joint gradient descent");
		optimizationTypeBox->insertItem("Optimization test");
		optimizationTypeBox->insertItem("Contact forces from tendons");
        }
	optimizationTypeBox->insertItem("Contact force existence");
	optimizationTypeBox->insertItem("Contact force optimization");
	optimizationTypeBox->insertItem("Grasp force existence");
	optimizationTypeBox->insertItem("Grasp force optimization");
	optimizationTypeBox->insertItem("Compliant joint equilibrium");
	optimizationTypeBox->insertItem("DOF force equilibrium");
	if (mHand->isA("McGrip")) {
		optimizationTypeBox->insertItem("McGrip tendon route");
		optimizationTypeBox->insertItem("McGrip joint equilibrium");
	}
	QObject::connect(exitButton, SIGNAL(clicked()), 
					 this, SLOT(exitButtonClicked()));
	QObject::connect(mHand, SIGNAL(configurationChanged()), 
					 this, SLOT(handConfigurationChanged()));
	QObject::connect(optimizationOnBox, SIGNAL(clicked()), 
					 this, SLOT(optimizationOnBoxClicked()));
}

GFODlg::~GFODlg()
{
	mMainWindow->clearContactsList();
}

void 
GFODlg::optimizationOnBoxClicked()
{
	if (!optimizationOnBox->isChecked()) {
		statusLabel->setText("Status: optimization off");
	} else {
		runOptimization();
	}
}

void
GFODlg::handConfigurationChanged()
{
	if (!optimizationOnBox->isChecked()) {
		return;
	}
	runOptimization();
}

void
GFODlg::runOptimization()
{
	mHand->getGrasp()->update();
	if (mHand->getGrasp()->getObject()) {
		mHand->getGrasp()->getObject()->resetExtWrenchAcc();	
	}

	if (optimizationTypeBox->currentText()=="Grasp force existence") {
          graspForceOptimization(Grasp::GRASP_FORCE_EXISTENCE);
	} else if (optimizationTypeBox->currentText()=="Grasp force optimization") {
          graspForceOptimization(Grasp::GRASP_FORCE_OPTIMIZATION);
	} else if (optimizationTypeBox->currentText()=="Contact force existence") {
          graspForceOptimization(Grasp::CONTACT_FORCE_EXISTENCE);
	} else if (optimizationTypeBox->currentText()=="Contact force optimization") {
          graspForceOptimization(Grasp::CONTACT_FORCE_OPTIMIZATION);
	} else if (optimizationTypeBox->currentText()=="Compliant joint equilibrium") {
		compliantEquilibriumOptimization(false);
	} else if (optimizationTypeBox->currentText()=="DOF force equilibrium") {
		compliantEquilibriumOptimization(true);
	} else if (optimizationTypeBox->currentText()=="McGrip tendon route") {
		tendonRouteOptimization();
	} else if (optimizationTypeBox->currentText()=="McGrip joint equilibrium") {
		mcgripEquilibrium();
	} else if (optimizationTypeBox->currentText()=="Tendon equilibrium") {
		tendonEquilibrium();
	} else if (optimizationTypeBox->currentText()=="Contact equilibrium") {
		contactEquilibrium();
	} else if (optimizationTypeBox->currentText()=="Equilibrium plot") {
		equilibriumPlot();
                optimizationOnBox->setChecked(false);
	} else if (optimizationTypeBox->currentText()=="Joint gradient descent") {
		jointGradientDescent();
                optimizationOnBox->setChecked(false);
	} else if (optimizationTypeBox->currentText()=="Optimization test") {
		optimizationTest();
                optimizationOnBox->setChecked(false);	
	} else if (optimizationTypeBox->currentText()=="Contact forces from tendons") {
		contactForcesFromTendons();
        } else {
		DBGA("Unkown option selected in optimization box");
	}
}

void 
GFODlg::displayResults(int result)
{
	if (result < 0) {
		statusLabel->setText("Status: optimization error");
		mMainWindow->clearContactsList();
	} else if (result > 0) {
		mMainWindow->clearContactsList();
		statusLabel->setText("Status: problem unfeasible");
	} else {
		statusLabel->setText("Status: optimization successful");
		graspItGUI->getIVmgr()->drawDynamicForces();
		//keep in mind that World::updateGrasps() will overwrite this and use the wrenches
		//to draw the worst case disturbance instead
		graspItGUI->getIVmgr()->drawUnbalancedForces();
		mMainWindow->updateContactsList();
	}
}

void
GFODlg::mcgripEquilibrium()
{
	if (!mHand->isA("McGrip")) {
		DBGA("Hand is not a McGrip!");
		return;
	}
	int result = static_cast<McGrip*>(mHand)->jointTorqueEquilibrium();
	displayResults(result);
}

void
GFODlg::tendonRouteOptimization()
{
	if (!mHand->isA("McGrip")) {
		DBGA("Hand is not a McGrip!");
		return;
	}
	/*
	//tendon route optimization
	Matrix l(6,1);
	int result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonRouteOptimization(&l);
	DBGA("l matrix:\n" << l);
	*/
	
	//tendon and construction optimization with new formulation
	Matrix p(8,1);
	double obj;
	int result = static_cast<McGripGrasp*>(mHand->getGrasp())->tendonAndHandOptimization(&p, obj);
	DBGA("p matrix:\n" << p);
	displayResults(result);
	
	/*
	Matrix *a, *B;
	static_cast<McGrip*>(mHand)->getRoutingMatrices(&B, &a);
	delete a;
	delete B;
	*/
}

//temporary; defined in gripperDesigns.cpp
bool normalize(std::vector<double> &vec);

PROF_DECLARE(GFO_GRADIENT_DESCENT);
void
GFODlg::jointGradientDescent()
{
  if (!mHand->inherits("GripperDesign")) {
    DBGA("Hand is not a Gripper Design!");
    return;
  }
  GripperDesign *hand = static_cast<GripperDesign*>(mHand);
  //PROF_RESET_ALL;
  //hand->resetPose();
  //PROF_PRINT_ALL;
  
  std::vector<double> currentPose(hand->getNumDOF(), 0.0);
  std::vector<double> finalPose(hand->getNumDOF(), 0.0);
  hand->getDOFVals(&currentPose[0]);
  std::vector<double> tendonForces(1, 0.0);

  tendonForces[0] = hand->getTendon(0)->getActiveForce();
  bool convergence;
  PROF_RESET_ALL;
  PROF_START_TIMER(GFO_GRADIENT_DESCENT);
  if (!hand->jointPoseGradientDescent(currentPose, tendonForces, finalPose, true, convergence))
  {
    DBGA("Error in calculation");
    return;
  }
  PROF_STOP_TIMER(GFO_GRADIENT_DESCENT);
  PROF_PRINT_ALL;

  if (convergence) {DBGA("Convergence; pose: " << finalPose[0] << " " << finalPose[1]);}
  else {DBGA("No convergence; pose: " << finalPose[0] << " " << finalPose[1]);}
  
}

void
GFODlg::tendonEquilibrium()
{
  if (!mHand->inherits("HumanHand")) {
    DBGA("Hand is not a Human Hand!");
    return;
  }

  HumanHand *hand = static_cast<HumanHand*>(mHand);
  if (hand->getNumTendons() < 1) 
  {
    DBGA("Hand not suited for hard-coded analysis");
    return;
  }  

  std::cerr << "DOFs: " << hand->getDOF(0)->getVal() << " " << hand->getDOF(1)->getVal() << "\n";

  std::cerr << "Joints: " << hand->getChain(0)->getJoint(0)->getVal() << " " 
            << hand->getChain(0)->getJoint(1)->getVal() << "\n";
  std::cerr << "Min ins pt distance: " << hand->minInsPointDistance() << "\n";

  std::set<size_t> activeTendons;
  activeTendons.insert(0);
  std::set<size_t> passiveTendons;
  passiveTendons.insert(1);
  passiveTendons.insert(2);

  double unbalanced_magnitude;
  std::vector<double> tendonForces;
  std::vector<double> jointResiduals;

  
  int result = hand->tendonEquilibrium(activeTendons, passiveTendons, true, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);  

  std::cerr << "Tendon forces:";
  for (size_t i=0; i<tendonForces.size(); i++) std::cerr << " " << tendonForces[i]*1.0e-6 << "N";
  std::cerr << "\n";

  std::cerr << "Joint residuals:";
  for (size_t i=0; i<jointResiduals.size(); i++) std::cerr << " " << jointResiduals[i]*1.0e-6 << "Nmm";
  std::cerr << "\n";

  std::cerr << "Unbalanced magnitude: " << unbalanced_magnitude*1.0e-6 << "Nmm \n\n";


  tendonForces[0] = 2.0 * 1.0e6;
  result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);

  std::cerr << "Tendon forces:";
  for (size_t i=0; i<tendonForces.size(); i++) std::cerr << " " << tendonForces[i]*1.0e-6 << "N";
  std::cerr << "\n";

  std::cerr << "Joint residuals:";
  for (size_t i=0; i<jointResiduals.size(); i++) std::cerr << " " << jointResiduals[i]*1.0e-6 << "Nmm";
  std::cerr << "\n";

  std::cerr << "Unbalanced magnitude: " << unbalanced_magnitude*1.0e-6 << "Nmm \n\n";


  tendonForces[0] = hand->getTendon(0)->getActiveForce();
  result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                   jointResiduals, unbalanced_magnitude);

  std::cerr << "Tendon forces:";
  for (size_t i=0; i<tendonForces.size(); i++) std::cerr << " " << tendonForces[i]*1.0e-6 << "N";
  std::cerr << "\n";

  std::cerr << "Joint residuals:";
  for (size_t i=0; i<jointResiduals.size(); i++) std::cerr << " " << jointResiduals[i]*1.0e-6 << "Nmm";
  std::cerr << "\n";

  std::cerr << "Unbalanced magnitude: " << unbalanced_magnitude*1.0e-6 << "Nmm \n\n";
  
  std::cerr << "---\n\n";

  displayResults(result);  
}

PROF_DECLARE(EQUILIBRIUM_PLOT);
void GFODlg::equilibriumPlot()
{
  if (!mHand->inherits("HumanHand")) {
    DBGA("Hand is not a Human Hand!");
    return;
  }

  HumanHand *hand = static_cast<HumanHand*>(mHand);
  if (hand->getNumDOF() != 2) 
  {
    DBGA("Hand not suited for hard-coded analysis");
    return;
  }  
  
  bool ok;
  QString filename = QInputDialog::getText(this, tr("QInputDialog::getText()"), tr("Results file:"), 
                                           QLineEdit::Normal, "equilibrium.csv", &ok);
  if (!ok) {DBGA("Canceled by user"); return;}

  DBGA("Starting equilibrium plot");
  FILE* file = fopen(filename.toLatin1().data(), "w");
  if (!file) 
  {
    DBGA("Failed to open file for results");
    return;
  }

  std::set<size_t> activeTendons;
  activeTendons.insert(0);
  std::set<size_t> passiveTendons;
  passiveTendons.insert(1);
  passiveTendons.insert(2);

  std::vector<double> dofs(2, 0.0);
  dofs[0] = hand->getDOF(0)->getMin();
  double resolution = 1.0 * 3.14159 / 180.0;
  int outer_count = 0;
  PROF_RESET_ALL;
  PROF_START_TIMER(EQUILIBRIUM_PLOT);
  while ( dofs[0] <= hand->getDOF(0)->getMax() )
  {
    int inner_count = 0;
    dofs[1] = hand->getDOF(1)->getMin();  
    while (dofs[1] <= hand->getDOF(1)->getMax())
    {
      static_cast<GripperDesign*>(mHand)->setPose(dofs);
      /*
      std::list<Contact*> contacts;
      contacts.push_back(hand->getChain(0)->getLink(1)->getVirtualContacts().front());      
      std::vector<double> jointContactTorques;
      if (hand->contactTorques(contacts, jointContactTorques))
      {
        DBGA("Joint torque computation failed");
        continue;
      }
      fprintf(file,"%f,%f,0,0,%f,%f", dofs[0], dofs[1], jointContactTorques[0],jointContactTorques[1]);
      contacts.push_back(hand->getChain(0)->getLink(0)->getVirtualContacts().front());
      if (hand->contactTorques(contacts, jointContactTorques))
      {
        DBGA("Joint torque computation failed");
        continue;
      }
      fprintf(file,",0,0,%f,%f",jointContactTorques[0],jointContactTorques[1]);
      */
      
      double unbalanced_magnitude;
      std::vector<double> tendonForces;
      std::vector<double> jointResiduals;
      int result = hand->tendonEquilibrium(activeTendons, passiveTendons, true, tendonForces, 
                                           jointResiduals, unbalanced_magnitude);
      if (result) 
      {
        DBGA("Error in optimization at dof values " << dofs[0] << " " << dofs[1]);
        continue;
      }
      fprintf(file,"%f,%f,%f,%f,%f,%f", dofs[0], dofs[1], tendonForces[0], unbalanced_magnitude,
              jointResiduals[0], jointResiduals[1]);

      tendonForces.resize(1,0.0);

      tendonForces[0]=1.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=3.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=5.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=2.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=0.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=2.0 * 1.0e6;
      std::set<size_t> noTendons;
      result = hand->tendonEquilibrium(activeTendons, noTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude, false);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);

      tendonForces[0]=10.0 * 1.0e6;
      result = hand->tendonEquilibrium(activeTendons, passiveTendons, false, tendonForces, 
                                       jointResiduals, unbalanced_magnitude);
      if (result) {DBGA("Error in opt. at dofs " << dofs[0] << " " << dofs[1]); continue;}
      fprintf(file,",%f,%f,%f,%f", tendonForces[0], unbalanced_magnitude, jointResiduals[0], jointResiduals[1]);
      
      fprintf(file,"\n");
      dofs[1] += resolution;
      inner_count++;
    }
    dofs[0] += resolution;
    DBGA( 100* (dofs[0] - hand->getDOF(0)->getMin()) / (hand->getDOF(0)->getMax() - hand->getDOF(0)->getMin()) 
          << "\% done; inner count: " << inner_count);
    outer_count++;
  }

  fclose(file);
  DBGA("Equilibrium plot done; outer_count: " << outer_count);

  PROF_STOP_TIMER(EQUILIBRIUM_PLOT);
  PROF_PRINT_ALL;
}

void
GFODlg::contactForcesFromTendons()
{
  if (!mHand->inherits("HumanHand")) {
    DBGA("Hand is not a Human Hand!");
    return;
  }

  HumanHand *hand = static_cast<HumanHand*>(mHand);
  if (hand->getNumTendons() < 1 || 
      hand->getNumChains() < 1 ||
      hand->getChain(0)->getNumLinks() < 2 ||
      hand->getChain(0)->getLink(0)->getNumVirtualContacts() < 1 || 
      hand->getChain(0)->getLink(1)->getNumVirtualContacts() < 1 ) 
  {
    DBGA("Hand not suited for hard-coded analysis");
    return;
  }  

  std::set<size_t> activeTendons;
  activeTendons.insert(0);
  std::list<Contact*> contacts;
  contacts.push_back(hand->getChain(0)->getLink(0)->getVirtualContacts().front());
  contacts.push_back(hand->getChain(0)->getLink(1)->getVirtualContacts().front());

  std::vector<double> tendonForces(1);
  tendonForces[0] = hand->getTendon(0)->getActiveForce();
  std::vector<double> contactForces;

  int result = hand->contactForcesFromTendonForces(contacts, contactForces, activeTendons, tendonForces);
  if (!result)
  {
    std::cerr << "Contacts forces:";
    for (size_t i=0; i<contactForces.size(); i++) std::cerr << " " << contactForces[i]*1.0e-6 << "N";
    std::cerr << "\n";
  }
  else
  {
    DBGA("Computation failed");
  }
  displayResults(result);
}

void
GFODlg::contactEquilibrium()
{
  if (!mHand->inherits("HumanHand")) {
    DBGA("Hand is not a Human Hand!");
    return;
  }

  HumanHand *hand = static_cast<HumanHand*>(mHand);
  if (hand->getNumTendons() < 1 || 
      hand->getNumChains() < 1 ||
      hand->getChain(0)->getNumLinks() < 2 ||
      hand->getChain(0)->getLink(0)->getNumVirtualContacts() < 1 || 
      hand->getChain(0)->getLink(1)->getNumVirtualContacts() < 1 ) 
  {
    DBGA("Hand not suited for hard-coded analysis");
    return;
  }  

  std::set<size_t> activeTendons;
  activeTendons.insert(0);
  std::list<Contact*> contacts;
  contacts.push_back(hand->getChain(0)->getLink(0)->getVirtualContacts().front());
  contacts.push_back(hand->getChain(0)->getLink(1)->getVirtualContacts().front());

  std::vector<double> jointContactTorques;
  if (hand->contactTorques(contacts, jointContactTorques))
  {
    DBGA("Joint torque computation failed");
  }
  else
  {
    DBGA("Joint torques from contacts: " << jointContactTorques[0] << " " << jointContactTorques[1]);
  }

  double unbalanced_magnitude;
  std::vector<double> tendonForces;
  int result = hand->contactEquilibrium(contacts, activeTendons, tendonForces, unbalanced_magnitude);
  if (!result)
  {
    std::cerr << "Tendon forces:";
    for (size_t i=0; i<tendonForces.size(); i++) std::cerr << " " << tendonForces[i]*1.0e-6 << "N";
    std::cerr << "\n";
    std::cerr << "Unbalanced magnitude: " << unbalanced_magnitude*1.0e-6 << "Nmm \n\n";
  }
  displayResults(result);  
  //std::cerr << "DOF 0: " << hand->getDOF(0)->getVal() * 180.0 / 3.14159 << "\n";
  //std::cerr << "DOF 1: " << hand->getDOF(1)->getVal() * 180.0 / 3.14159 << "\n";
}

void GFODlg::optimizationTest()
{
  if (!mHand->inherits("GripperDesign")) {
    DBGA("Hand is not a Gripper Design!");
    return;
  }
  GripperDesign *hand = static_cast<GripperDesign*>(mHand);
  std::vector<double> results, params;
  if (hand->performOptimization(params, results))
  {
    std::cerr << "Optimization results:";
    for(size_t i=0; i<results.size(); i++) 
    {
      std::cerr << " " << results[i];
    }
    std::cerr << "\n";
  }  
  else
  {
    DBGA("Optimization failed");
  }
}

void
GFODlg::compliantEquilibriumOptimization(bool useDynamicDofForce)
{
	Matrix tau(mHand->staticJointTorques(useDynamicDofForce));
	int result = mHand->getGrasp()->computeQuasistaticForces(tau);
	displayResults(result);
}

void
GFODlg::graspForceOptimization(int computation)
{
	Matrix tau(Matrix::ZEROES<Matrix>(mHand->getNumJoints(),1));
	int result = mHand->getGrasp()->computeQuasistaticForcesAndTorques(&tau, computation);
	if (!result) {
		DBGA("Optimal joint torques:\n" << tau);
	}
	displayResults(result);
}
