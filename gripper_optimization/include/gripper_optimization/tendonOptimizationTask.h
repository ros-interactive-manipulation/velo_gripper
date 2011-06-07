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

#ifndef _TENDON_OPTIMIZATION_TASK_H_
#define _TENDON_OPTIMIZATION_TASK_H_

#include "DBPlanner/task.h"

#include <graspit_dbase_tasks/dbTask.h>

namespace gripper_optimization {

class GripperDesign;

class TendonOptimizationTask : public graspit_dbase_tasks::DBTask
{
protected:
  //! Holds the details of the optimization task itself
  db_planner::OptimizationTaskRecord mOptimizationTask;

  //! The hand used for all the optimizations
  GripperDesign *mGripper;

  //! Makes sure the currently loaded hand is the right one
  void loadHand();

public:
  TendonOptimizationTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                         db_planner::DatabaseManager *mgr, 
                         db_planner::TaskRecord rec); 

  virtual ~TendonOptimizationTask(){}

};

/*! Will run a single optimization, using the parameters already specified in the database. */
class SingleTendonOptimizationTask : public TendonOptimizationTask
{
public:
  SingleTendonOptimizationTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                               db_planner::DatabaseManager *mgr, 
                               db_planner::TaskRecord rec) : 
    TendonOptimizationTask(disp, mgr, rec) {}

  virtual ~SingleTendonOptimizationTask(){}

  virtual void start();

  virtual void mainLoop() {TendonOptimizationTask::mainLoop();}
};

class RandomTendonOptimizationTask : public TendonOptimizationTask
{
protected:

  std::vector<double> mParameterMin;

  std::vector<double> mParameterMax;
 
  std::vector<double> randomParameters();

  bool gradientDescentStep(const std::vector<double> &seed,
                           std::vector<double> &parameters,
                           std::vector<double> &results);

  bool gradientDescent(std::vector<double> &seed,
                       std::vector<double> &results);
public:
  RandomTendonOptimizationTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                               db_planner::DatabaseManager *mgr, 
                               db_planner::TaskRecord rec);

  virtual ~RandomTendonOptimizationTask(){}

  virtual void start();

  virtual void mainLoop();
};


} //namespace

#endif
