// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "MainWindow.hpp"

#include <QFileDialog>
#include <QGridLayout>
#include <QMenuBar>
#include <QDebug>
#include <QMenu>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QtGui>
#include <QWidget>
#include <iostream>
#include <fstream>

#include <algorithm>
#include <clocale>

#include "ControlPoint2DNode.hpp"
#include "ControlPointTableView.hpp"

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"
#include <openMVG/geometry/Similarity3.hpp>
//#include "openMVG/sfm/sfm_data_BA_ceres.hpp>
#include <openMVG/sfm/sfm_data_transform.hpp>
#include <openMVG/sfm/sfm_data_BA_ceres.hpp>

#include "openMVG/stl/stl.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

std::string output_dir;

void MainWindow::help(){
  /*
   Usage: ./control_point_Reg <sfm data bin> <output dir>
   */
}

void MainWindow::doubleClickImageList(){
  const QModelIndex modelIndex = m_treeView_Images->currentIndex();
  const QModelIndex res = modelIndex.sibling ( modelIndex.row(), 0 );
  const QString current_string_clicked = res.data().toString();

  std::istringstream is(current_string_clicked.toStdString());
  size_t id;
  is >> id;
  const View * view = m_doc._sfm_data.GetViews().at(id).get();
  const std::string sView = stlplus::create_filespec(m_doc._sfm_data.s_root_path, view->s_Img_path);
 // m_widget->addImage(QString::fromStdString(sView), 0.0, 0.0, false);
 // m_widget->setCurrentViewId(view->id_view);

  QImage myImage;
  myImage.load(sView.c_str());

  myImage = myImage.scaledToWidth(myLabel->width());
  myLabel->setPixmap(QPixmap::fromImage(myImage));

  //myLabel->setFixedHeight(640);
  //myLabel->setFixedWidth(480);
  //myLabel->setScaledContents(true);

  std::string output_path = output_dir;
  output_path += "/image_selected.txt";

  std::ofstream ofs(output_path.c_str());

  ofs << sView << std::endl;
  ofs << view->id_view << std::endl;
  //ofs << view->s_Img_path << std::endl;
  ofs.close();

  button->setEnabled(true);

}

void MainWindow::openProject(char** argv){
  const QString sfm_data_fileName = argv[1];
      //QFileDialog::getOpenFileName(this, tr("Choose a sfm_data project file"),
    //QString::null, tr("sfm_data (*.json *.xml *.bin)"));
  if(sfm_data_fileName.isEmpty())
    return;

  m_sfm_data_filename = sfm_data_fileName.toStdString();

  if(m_doc.loadData(sfm_data_fileName.toStdString())){
    //Add image names in the QT tree view
    {
      QStandardItemModel * model = new QStandardItemModel(0,1, this);
      model->setHeaderData(0, Qt::Horizontal, QObject::tr("Views"));
      m_treeView_Images->setModel(model);

      std::vector<IndexT> view_ids;
      view_ids.reserve(m_doc._sfm_data.GetViews().size());
      std::transform(m_doc._sfm_data.GetViews().begin(), m_doc._sfm_data.GetViews().end(),
                     std::back_inserter(view_ids), stl::RetrieveKey());
      std::sort(view_ids.begin(), view_ids.end());

      // Add view in reverse order to have them ordered by ID
      for (std::vector<IndexT>::const_reverse_iterator iter = view_ids.rbegin();
        iter != view_ids.rend(); ++iter){
        Views::const_iterator iterV = m_doc._sfm_data.GetViews().find(*iter);
        const View * view = iterV->second.get();
        if (m_doc._sfm_data.IsPoseAndIntrinsicDefined(view)) {
          std::ostringstream os;
          os << view->id_view << " " << view->s_Img_path;
          model->insertRow(0);
          model->setData(model->index(0, 0), QString::fromStdString(os.str()));
        }
      }
    }
  }else{
    QMessageBox msgBox;
    msgBox.setText("Cannot open the provided sfm_data file.");
    msgBox.exec();
  }
}

MainWindow::MainWindow(int argc,char** argv,QWidget * parent): QMainWindow(){

   output_dir = argv[2];
   createPanel(argv);
   createConnections();

  setWindowTitle(tr("Control_point_editor"));

  QMainWindow::statusBar()->showMessage("Choose an image pattern reference and then press [OK].");

}

void MainWindow::createPanel(char** argv){

  QSplitter *splitter = new QSplitter;  
  //-- Create left panel
  m_tabWidget = new QTabWidget;
  //-- Create right panel
  m_widget = new control_point_GUI::GraphicsView(m_doc, this);

  button = new QPushButton();
  QString text("OK");
  button->setText(text);

  myLabel = new QLabel(this);

  splitter->addWidget(m_tabWidget);
  splitter->addWidget(myLabel);

  //splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  setCentralWidget(splitter);

  //-- Add tab inside the m_tabWidget
  m_tab_1 = new QWidget;
  m_tab_1->setObjectName(QString::fromUtf8("m_tab_1"));
  m_tabWidget->addTab(m_tab_1, QString());
  m_tabWidget->setTabText(m_tabWidget->indexOf(m_tab_1), "ImageList");

  //-- Configure tab widgets
  m_treeView_Images = new QTreeView(m_tab_1);
  m_treeView_Images->setRootIsDecorated(false);
  m_treeView_Images->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_treeView_Images->setObjectName(QString::fromUtf8("m_treeView_Images"));
  m_treeView_Images->setSortingEnabled(true);

  openProject(argv);

  button->setEnabled(false);

  QGridLayout * gridLayout1 = new QGridLayout(m_tab_1);
  gridLayout1->addWidget(m_treeView_Images, 0, 0, 1, 1);
  gridLayout1->addWidget(button);


}

void MainWindow::createConnections(){
  connect (m_treeView_Images,SIGNAL(activated(const QModelIndex &)),this,SLOT(doubleClickImageList()));

    connect(button, SIGNAL(clicked()), this, SLOT(close()));

}
