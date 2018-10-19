// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "MainWindow.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

std::string output_dir;

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

void MainWindow::getDefaultParameters(std::string& inputPath){

  std::ifstream file(inputPath.c_str());
  if(!file.is_open()){
      std::cout << "Error: Hough file not found." << std::endl;
      return std::exit(-1);
    }

  std::string dp;
  std::string minDist;
  std::string param1;
  std::string param2;
  std::string minRadius;
  std::string maxRadius;


  while(file >> dp >> minDist >> param1 >> param2 >> minRadius >> maxRadius){
  }

  dpLineEdit->setText(dp.c_str());
  minDistLineEdit->setText(minDist.c_str());
  param1LineEdit->setText(param1.c_str());
  param2LineEdit->setText(param2.c_str());
  minRadiusLineEdit->setText(minRadius.c_str());
  maxRadiusLineEdit->setText(maxRadius.c_str());
}

void MainWindow::setParameters(void){

  dp = dpLineEdit->text().toStdString();
  minDist = minDistLineEdit->text().toStdString();
  param1 = param1LineEdit->text().toStdString();
  param2 = param2LineEdit->text().toStdString();
  minRadius = minRadiusLineEdit->text().toStdString();
  maxRadius = maxRadiusLineEdit->text().toStdString();

  std::string out = output_dir;
  out += "/hough_parameters.txt";

  std::ofstream ofs(out.c_str());
  ofs << dp << std::endl
      << minDist << std::endl
      << param1 << std::endl
      << param2 << std::endl
      << minRadius << std::endl
      << maxRadius << std::endl;
  ofs.close();

  this->close();

}

void MainWindow::createPanel(char** argv){

  QSplitter *splitter = new QSplitter;  
  //-- Create left panel
  m_tabWidget = new QTabWidget;
  //-- Create right panel
  //m_widget = new control_point_GUI::GraphicsView(m_doc, this);

  button = new QPushButton();
  buttonOkParams = new QPushButton();
  QString text("OK");
  button->setText(text);
  buttonOkParams->setText("Set Params");

  myLabel = new QLabel(this);

  splitter->addWidget(m_tabWidget);
  splitter->addWidget(myLabel);

  //splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  setCentralWidget(splitter);

  //-- Add tab inside the m_tabWidget
  m_tab_1 = new QWidget;
  houghParameters = new QWidget;
  m_tab_1->setObjectName(QString::fromUtf8("m_tab_1"));
  m_tabWidget->addTab(m_tab_1, QString());
  m_tabWidget->setTabText(m_tabWidget->indexOf(m_tab_1), "ImageList");
  m_tabWidget->addTab(houghParameters, "Parameters");

  dpLineEdit = new QLineEdit;
  minDistLineEdit = new QLineEdit;
  param1LineEdit = new QLineEdit;
  param2LineEdit = new QLineEdit;
  minRadiusLineEdit = new QLineEdit;
  maxRadiusLineEdit = new QLineEdit;

  dpLineEdit->setFixedWidth(50);
  minDistLineEdit->setFixedWidth(50);
  param1LineEdit->setFixedWidth(50);
  param2LineEdit->setFixedWidth(50);
  minRadiusLineEdit->setFixedWidth(50);
  maxRadiusLineEdit->setFixedWidth(50);

  dpLineEdit->setValidator(new QDoubleValidator(0,900,2,this));
  minDistLineEdit->setValidator(new QDoubleValidator(0,900,2,this));
  param1LineEdit->setValidator(new QDoubleValidator(0,900,2,this));
  param2LineEdit->setValidator(new QDoubleValidator(0,900,2,this));
  minRadiusLineEdit->setValidator( new QIntValidator(0, 900, this));
  maxRadiusLineEdit->setValidator( new QIntValidator(0, 900, this));

  //-- Configure tab widgets
  m_treeView_Images = new QTreeView(m_tab_1);
  m_treeView_Images->setRootIsDecorated(false);
  m_treeView_Images->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_treeView_Images->setObjectName(QString::fromUtf8("m_treeView_Images"));
  m_treeView_Images->setSortingEnabled(true);

  QLabel * dpLabel = new QLabel;
  QLabel * minDistLabel = new QLabel;
  QLabel * param1Label = new QLabel;
  QLabel * param2Label = new QLabel;
  QLabel * minRadiusLabel = new QLabel;
  QLabel * maxRadiusLabel = new QLabel;

  dpLabel->setText("DP");
  minDistLabel->setText("Min distance");
  param1Label->setText("Param 1");
  param2Label->setText("Param 2");
  minRadiusLabel->setText("Min radius");
  maxRadiusLabel->setText("Max radius");

  openProject(argv);
  std::string houghDefaultParameters = argv[3];
  getDefaultParameters(houghDefaultParameters);

  button->setEnabled(false);

  QGridLayout * gridLayout1 = new QGridLayout(m_tab_1);
  gridLayout1->addWidget(m_treeView_Images, 0, 0, 1, 1);
  gridLayout1->addWidget(button);

  QGridLayout * gridLayout2 = new QGridLayout(houghParameters);
  gridLayout2->addWidget(dpLineEdit,0,1);
  gridLayout2->addWidget(minDistLineEdit,1,1);
  gridLayout2->addWidget(param1LineEdit,2,1);
  gridLayout2->addWidget(param2LineEdit,3,1);
  gridLayout2->addWidget(minRadiusLineEdit,4,1);
  gridLayout2->addWidget(maxRadiusLineEdit,5,1);

  gridLayout2->addWidget(dpLabel,0,0);
  gridLayout2->addWidget(minDistLabel,1,0);
  gridLayout2->addWidget(param1Label,2,0);
  gridLayout2->addWidget(param2Label,3,0);
  gridLayout2->addWidget(minRadiusLabel,4,0);
  gridLayout2->addWidget(maxRadiusLabel,5,0,1,1);
  gridLayout2->setAlignment(Qt::AlignHCenter);
  gridLayout2->setAlignment(Qt::AlignTop);
  buttonOkParams->setFixedWidth(200);
  gridLayout2->addWidget(buttonOkParams,6,0,1,1,Qt::AlignCenter);

}

void MainWindow::createConnections(){
  connect (m_treeView_Images,SIGNAL(activated(const QModelIndex &)),this,SLOT(doubleClickImageList()));
  connect(button, SIGNAL(clicked()), this, SLOT(close()));
  connect(buttonOkParams, SIGNAL(clicked()), this, SLOT(setParameters()));
}
