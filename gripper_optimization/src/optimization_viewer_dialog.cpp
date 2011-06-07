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
// $Id: gfoDlg.h,v 1.3 2009/08/17 22:17:32 cmatei Exp $
//
//######################################################################

#include "gripper_optimization/optimization_viewer_dialog.h"

#include <algorithm>

#include <household_objects_database/objects_database.h>
#include <household_objects_database/database_task.h>

#include "mainWindow.h"
#include "graspitGUI.h"
#include "world.h"
#include "debug.h"

#include "gripper_optimization/gripperDesigns.h"

using household_objects_database::DatabaseOptimizationResult;

namespace gripper_optimization {

OptimizationViewerDialog::OptimizationViewerDialog(QWidget *parent) :
  QDialog(parent),
  mCurrentResult(0)
{
  setupUi(this);

  QObject::connect(mNextButton, SIGNAL(clicked()), this, SLOT(next()));
  QObject::connect(mPrevButton, SIGNAL(clicked()), this, SLOT(prev()));
  QObject::connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
  QObject::connect(mLoadButton, SIGNAL(clicked()), this, SLOT(load()));
  QObject::connect(mOptimizationButton, SIGNAL(clicked()), this, SLOT(optimization()));

  mDatabase = new household_objects_database::ObjectsDatabase("wgs36", "5432", "willow", 
                                                              "willow", "household_objects");
  if (!mDatabase->isConnected())
  {
    DBGA("Database failed to connect");
    delete mDatabase; mDatabase = NULL;
  }
}

OptimizationViewerDialog::~OptimizationViewerDialog()
{
  delete mDatabase;
}

void OptimizationViewerDialog::done()
{
  QDialog::accept();
}

bool compareResults(const boost::shared_ptr<DatabaseOptimizationResult> &r1, 
                    const boost::shared_ptr<DatabaseOptimizationResult> &r2)
{
  if (r1->results_.data().at(0) < r2->results_.data().at(0)) return true;
  return false;
}

void OptimizationViewerDialog::load()
{
  World *world = graspItGUI->getMainWindow()->getMainWorld();
  if (!world->getCurrentHand()) 
  {
    DBGA("No hand selected");
    return;
  }
  if (!mDatabase)
  {
    DBGA("Database is not connected");
    return;
  }

  std::vector< boost::shared_ptr<DatabaseOptimizationResult> > newResults;
  std::string db_name = GripperDesign::getDBName( world->getCurrentHand()->getName().toStdString() );
  std::string whereClause("hand_name='" + db_name + "'");
  if (!mDatabase->getList<DatabaseOptimizationResult>(newResults, whereClause))
  {
    DBGA("Failed to load optimization results");
    return;
  }
  DBGA(newResults.size() << " optimization results loaded");
  mResults = newResults;
  std::sort(mResults.begin(), mResults.end(), compareResults);
  mCurrentResult = 0;
  showResult();
}

void OptimizationViewerDialog::showResult()
{
  if (mResults.empty()) 
  {
    mRankLabel->setText("Rank: 0/0");
    return;
  }
  QString currentNum, totalNum;
  currentNum.setNum((int)mCurrentResult + 1);
  totalNum.setNum((int)mResults.size());
  mRankLabel->setText("Rank: " + currentNum + "/" + totalNum);
  if ( mCurrentResult >= mResults.size())
  {
    DBGA("Wrong result requested, exceeding number of loaded results");
    return;
  }
  World *world = graspItGUI->getMainWindow()->getMainWorld();
  if (!world->getCurrentHand()) 
  {
    DBGA("No hand selected");
    return;
  }
  if (!world->getCurrentHand()->isA("GripperDesign"))
  {
    DBGA("Wrong hand loaded");
    return;
  }
  GripperDesign *hand = static_cast<GripperDesign*>(world->getCurrentHand());
  if (!hand->setOptimizationParameters(mResults.at(mCurrentResult)->parameters_.data()))
  {
    DBGA("Failed to set results on hand");
    return;
  }
  
  //get current hand DOFs
  std::vector<double> current_pose(hand->getNumDOF(), 0.0);
  hand->getDOFVals(&current_pose[0]);
  hand->resetPose();
  //move hand back in old pose
  std::vector<double> step_by(hand->getNumDOF(), M_PI/36.0);
  hand->moveDOFToContacts(&current_pose[0], &step_by[0], true);
  
  QString magn;
  double m = mResults.at(mCurrentResult)->results_.data().at(0);
  if (m > 1.0e4)
  {
    magn.setNum(m * 1.0e-6,'f',2);
    mMagnitudeLabel->setText("Magnitude: " + magn + " * 1.0e6");
  }
  else
  {
    magn.setNum(m,'f',4);
    mMagnitudeLabel->setText("Magnitude: " + magn);
  }

  if (hand->insPointInsideWrapper()){DBGA("Insertion point inside wrapper");}
  else DBGA("No insertion point inside wrapper");
}

void OptimizationViewerDialog::next()
{
  if (mResults.empty()) mCurrentResult = 0;
  else mCurrentResult = std::min(mCurrentResult+1, mResults.size() - 1);
  showResult();
}

void OptimizationViewerDialog::prev()
{
  if (mCurrentResult > 0) mCurrentResult--;
  showResult();
}

void OptimizationViewerDialog::optimization()
{
  if (mResults.empty()) return;
  if ( mCurrentResult >= mResults.size())
  {
    DBGA("Wrong result requested, exceeding number of loaded results");
    return;
  }
  World *world = graspItGUI->getMainWindow()->getMainWorld();
  if (!world->getCurrentHand()) 
  {
    DBGA("No hand selected");
    return;
  }
  if (!world->getCurrentHand()->isA("GripperDesign"))
  {
    DBGA("Wrong hand loaded");
    return;
  }
  GripperDesign *hand = static_cast<GripperDesign*>(world->getCurrentHand());
  std::vector<double> results;
  if (!hand->performOptimization(mResults.at(mCurrentResult)->parameters_.data(), results))
  {
    DBGA("Optimization failed");
    return;
  }
  DBGA("Optimization result: " << results[0]);
}

}
