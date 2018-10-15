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

  std::string kSearch;

  std::string distanceWeight;
  std::string maxIterations;
  std::string distanceThreshold;

  std::string distanceWeightCylinder;
  std::string maxIterationsCylinder;
  std::string distanceThresholdCylinder;
  std::string minRadius;
  std::string maxRadius;

  std::string output_dir;

private slots:

  void init();
  void on_okButton_clicked();
  void on_clearButton_clicked();
  void getDefaultParameters(std::string& inputPath);
};

#endif // VISUALIZER_HPP
