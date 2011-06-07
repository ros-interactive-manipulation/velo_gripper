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
// Author(s):  Matei T. Ciocarlie
//
// $Id: taskDispatcher.h,v 1.3 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

#include "gripper_optimization/tendonOptimizationTask.h"

#include "gripper_optimization/gripperDesigns.h"

#include <time.h>
#include <limits>

#include "graspitGUI.h"

#include "DBPlanner/db_manager.h"

#include "ivmgr.h"
#include "world.h"
#include "debug.h"

namespace gripper_optimization {

TendonOptimizationTask::TendonOptimizationTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                               db_planner::DatabaseManager *mgr, 
                                               db_planner::TaskRecord rec) 
  : graspit_dbase_tasks::DBTask(disp, mgr, rec) 
{
}

void TendonOptimizationTask::loadHand()
{
  World *world = graspItGUI->getIVmgr()->getWorld();

  //check if the currently selected hand is the same as the one we need
  //if not, load the hand we need
  if (world->getCurrentHand()) 
  {
    mGripper = dynamic_cast<GripperDesign*>(world->getCurrentHand());
    if (!mGripper) 
    {
      DBGA("Wrong hand is loaded");
      mStatus = ERROR;
      return;
    }    
  } 
  else 
  {
    QString handPath(GripperDesign::getGraspitFile(mOptimizationTask.hand_name).c_str());
    handPath = QString(getenv("GRASPIT")) + handPath;
    mGripper = dynamic_cast<GripperDesign*>(world->importRobot(handPath));
    if ( !mGripper ) 
    {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return;
    }
    //hide all tendons to save time during optimizations
    for (int i=0; i<mGripper->getNumTendons(); i++)
    {
      mGripper->getTendon(i)->setVisible(false);
    }
  }
}

void SingleTendonOptimizationTask::start()
{
  mStatus = RUNNING;

  //read the actual task details from the database
  if (!mDBMgr->GetOptimizationTaskRecord(mRecord.taskId, &mOptimizationTask))
  {
    mStatus = ERROR;
    return;
  }

  loadHand();
  if (mStatus==ERROR) return;
  
  //call the optimization over all poses
  std::vector<double> results;
  if (!mGripper->performOptimization(mOptimizationTask.parameters, results))
  {
    mStatus = ERROR;
    return;
  }

  //save the results to the database
  if (!mDBMgr->SaveOptimizationResults(mOptimizationTask, mOptimizationTask.parameters, results))
  {
    DBGA("Failed to save optimization results to database");
    mStatus = ERROR;
    return;
  }
  mStatus = DONE;
}


RandomTendonOptimizationTask::RandomTendonOptimizationTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                                           db_planner::DatabaseManager *mgr, 
                                                           db_planner::TaskRecord rec) : 
  TendonOptimizationTask(disp, mgr, rec) 
{
  srand( (unsigned)time(NULL) );
}

double randomVariable(double min, double max)
{
  double r = ((double)rand()) / RAND_MAX;
  return min + r * (max - min);
}

std::vector<double> RandomTendonOptimizationTask::randomParameters()
{
  std::vector<double> params;
  for (size_t i=0; i<mParameterMin.size(); i++)
  {
    params.push_back( randomVariable(mParameterMin[i], mParameterMax[i]) );
  }
  /*
  std::cout << "Random parameters: ";
  for (size_t i=0; i<params.size(); i++) std::cout << params[i] << " ";
  std::cout << "\n";   
  */
  return params;
}

void RandomTendonOptimizationTask::start()
{
  mStatus = RUNNING;
  //read the actual task details from the database
  if (!mDBMgr->GetOptimizationTaskRecord(mRecord.taskId, &mOptimizationTask))
  {
    mStatus = ERROR;
    return;
  }
  loadHand();
  if (mStatus==ERROR) return;
  mParameterMin = mGripper->getParameterMin();
  mParameterMax = mGripper->getParameterMax();
  if (mParameterMin.empty() ||
      mParameterMin.size() != mParameterMax.size())
  {
    DBGA("Illegal parameter bounds from hand");
    mStatus = ERROR;
  }
}

void RandomTendonOptimizationTask::mainLoop()
{  
  std::vector<double> parameters = randomParameters();
  std::vector<double> results;

  /*
  //call the optimization over all poses
  if (!mGripper->performOptimization(parameters, results))
  {
    mStatus = ERROR;
    return;
  }
  */

  //call gradient descent
  if (!gradientDescent(parameters, results))
  {
    DBGA("Gradient descent failed");
    mStatus = ERROR;
    return;
  }

  //save the results to the database
  if ( results[0] < mGripper->getSaveThreshold() && 
       !mDBMgr->SaveOptimizationResults(mOptimizationTask, parameters, results) )
  {
    DBGA("Failed to save random optimization results to database");
    mStatus = ERROR;
  }     
}

bool RandomTendonOptimizationTask::gradientDescentStep(const std::vector<double> &seed,
                                                       std::vector<double> &parameters,
                                                       std::vector<double> &results)
{
  double jump = 0.01;
  results.clear();
  results.push_back( std::numeric_limits<double>::max() );
  assert (seed.size() == mParameterMin.size() && seed.size() == mParameterMax.size());

  for (size_t i=0; i<seed.size(); i++)
  {
    std::vector<double> new_results;
    if (seed[i] > mParameterMin[i])
    {
      std::vector<double> perturbation = seed;
      perturbation[i] = std::max( mParameterMin[i],
                                  perturbation[i] - jump * (mParameterMax[i] - mParameterMin[i]) );
      if (!mGripper->performOptimization(perturbation, new_results)) return false;
      if (new_results[0] < results[0])
      {
        results = new_results;
        parameters = perturbation;
      }
    }
    if (seed[i] < mParameterMax[i])
    {
      std::vector<double> perturbation = seed;    
      perturbation[i] = std::min( mParameterMax[i],
                                  perturbation[i] + jump * (mParameterMax[i] - mParameterMin[i]) );
      if (!mGripper->performOptimization(perturbation, new_results)) return false;
      if (new_results[0] < results[0])
      {
        results = new_results;
        parameters = perturbation;
      }
    }
  }
  return true;
}

bool RandomTendonOptimizationTask::gradientDescent(std::vector<double> &seed,
                                                   std::vector<double> &results)
{
  double GATE = mGripper->getGDGate();
  double EPS = mGripper->getGDEps();
  //compute the quality of the seed
  if (!mGripper->performOptimization(seed, results)) return false;
  if (results[0] > GATE)
  {
    if (results[0] < std::numeric_limits<double>::max())
    {
      DBGA("Bad seed for GD: " << results[0] << "; not starting");
    }
    return true;
  }
  double initial_result_norm = results[0];
  int num_steps = 0;
  DBGA("GD starting at " << results[0]);
  while(1)
  {
    std::vector<double> new_parameters;
    std::vector<double> new_results;
    if (!gradientDescentStep(seed, new_parameters, new_results)) return false;
    if ( results[0] - new_results[0] > EPS)
    {
      DBGA("  GD improving from " << results[0] << " to " << new_results[0]);
      results = new_results;
      seed = new_parameters;
      num_steps++;
    }
    else
    {
      break;
    }
  }
  DBGA("Gradient descent from " << initial_result_norm << " to " << results[0] << " in " << num_steps << " steps.");
  return true;
}

}
