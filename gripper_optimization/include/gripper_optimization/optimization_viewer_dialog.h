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

#ifndef _optimization_viewer_dialog_h_
#define _optimization_viewer_dialog_h_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <QDialog>

#include "gripper_optimization/ui_optimization_viewer_dialog.h"

namespace household_objects_database {
class ObjectsDatabase;
class DatabaseOptimizationResult;
}

namespace gripper_optimization {

class OptimizationViewerDialog : public QDialog, public Ui::OptimizationViewerDialogBase
{
  Q_OBJECT
private:
  size_t mCurrentResult;
  std::vector< std::vector<double> > mParams;
  household_objects_database::ObjectsDatabase *mDatabase;
  std::vector< boost::shared_ptr<household_objects_database::DatabaseOptimizationResult> > mResults;

  void showResult();
public:
  OptimizationViewerDialog(QWidget *parent=0);
  ~OptimizationViewerDialog();

public slots:
  void load();
  void next();
  void prev();
  void done();
  void optimization();

};

}
#endif
