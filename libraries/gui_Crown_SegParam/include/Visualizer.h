#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QWidget>
#include <string>
#include <iostream>
#include <fstream>

namespace Ui {
  class Visualizer;
}

class Visualizer : public QWidget{
  Q_OBJECT

public:
  explicit Visualizer(int argc,char** argv,QWidget *parent = 0);
  ~Visualizer();


private:
  Ui::Visualizer *ui;

  std::string eps;
  std::string minPtsAux;
  std::string minPts;
  std::string octreeResolution;

  std::string output_dir;

private slots:

  void init();
  void on_okButton_clicked();
  void getDefaultParameters(std::string& inputPath);
};

#endif // VISUALIZER_HPP
