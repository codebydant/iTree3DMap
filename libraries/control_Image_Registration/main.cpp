// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <QApplication>
#include "src/MainWindow.hpp"
#include <QMessageBox>

int main(int argc, char ** argv){

  if(argc < 4 or argc > 4){
     std::cerr << "Enter: <sfm data bin> <output dir> <hough paramaters>" << std::endl;
     return -1;
   }


  QApplication app(argc, argv);

  const QString sfm_data_fileName = argv[1];
  Document temp;
      //QFileDialog::getOpenFileName(this, tr("Choose a sfm_data project file"),
    //QString::null, tr("sfm_data (*.json *.xml *.bin)"));
  if(sfm_data_fileName.isEmpty() or !temp.loadData(sfm_data_fileName.toStdString())){
      QMessageBox msgBox;
      msgBox.setText("Cannot open the provided sfm_data file.");
      msgBox.exec();
      return -1;
}

  MainWindow * mainWindow = new MainWindow(argc,argv);
  mainWindow->setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint);
  mainWindow->move(0,0);
  mainWindow->setFixedSize(640,480);
  mainWindow->show();

  return app.exec();
}
