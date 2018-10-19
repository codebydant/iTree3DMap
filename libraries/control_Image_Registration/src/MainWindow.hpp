// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QtGui>
#include <QTreeView>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QMainWindow>
#include <QSplitter>
#include <QStatusBar>
#include <QWidget>

#include <iostream>
#include <string>

#include "document.hpp"

class MainWindow : public QMainWindow{
  Q_OBJECT

private:
  QTabWidget * m_tabWidget;
  QWidget * m_tab_1;
  QLineEdit * dpLineEdit;
  QLineEdit * minDistLineEdit;
  QLineEdit * param1LineEdit;
  QLineEdit * param2LineEdit;
  QLineEdit * minRadiusLineEdit;
  QLineEdit * maxRadiusLineEdit;
  std::string dp;
  std::string minDist;
  std::string param1;
  std::string param2;
  std::string minRadius;
  std::string maxRadius;
  QWidget * houghParameters;
  QTreeView * m_treeView_Images;   // Image list of the project
  QLabel * myLabel;
  QPushButton* button ;
  QPushButton* buttonOkParams;

  QStatusBar *m_statusbar;         // Status bar

  // -- DOCUMENT
  Document m_doc;
  std::string m_sfm_data_filename;
  // -- END DOCUMENT

  private slots:

  void doubleClickImageList();
  void openProject(char** argv);
  void setParameters(void);
  void getDefaultParameters(std::string& inputPath);

public:

  /// Constructor
  MainWindow(int argc,char** argv,QWidget * parent = 0);

  /* Create the Window layout (shape panels & co.)
  |____________________________
  |Tab1|Tab2|                  |
  |---------|                  |
  |         |     Drawing      |
  | Img     |      Widget      |
  | list    |                  |
  |         |                  |
  |_OK Butt_|__________________|
*/
  void createPanel(char** argv);

  /// Establish action-event connections
  void createConnections();
};

#endif /* MAINWINDOW_HPP */
