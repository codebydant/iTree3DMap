#include "include/Visualizer.h"

#include "ui_Visualizer.h"

Visualizer::Visualizer(int argc, char** argv, QWidget* parent) : QWidget(parent), ui(new Ui::Visualizer) {
  ui->setupUi(this);

  std::string trunkDefaultParameters = argv[1];
  output_dir = argv[2];
  getDefaultParameters(trunkDefaultParameters);

  init();
}

Visualizer::~Visualizer() { delete ui; }

void Visualizer::init() {
  ui->kSearchLine->setValidator(new QIntValidator(0, 900, this));

  ui->distanceWeightLine->setValidator(new QDoubleValidator(0, 900, 2, this));
  ui->maxIterationsLine->setValidator(new QIntValidator(0, 900, this));
  ui->distanceThresHoldLine->setValidator(new QDoubleValidator(0, 900, 2, this));

  ui->distanceWeightCylinderLine->setValidator(new QDoubleValidator(0, 900, 2, this));
  ui->maxIterationsCylinderLine->setValidator(new QIntValidator(0, 900, this));
  ui->distanceThresholdCylinderLine->setValidator(new QDoubleValidator(0, 900, 2, this));
  ui->minRadiusCylinderLine->setValidator(new QDoubleValidator(0, 900, 2, this));
  ui->maxRadiusCylinderLine->setValidator(new QDoubleValidator(0, 900, 2, this));
}

void Visualizer::on_okButton_clicked() {
  kSearch = ui->kSearchLine->text().toStdString();

  distanceWeight = ui->distanceWeightLine->text().toStdString();
  maxIterations = ui->maxIterationsLine->text().toStdString();
  distanceThreshold = ui->distanceThresHoldLine->text().toStdString();

  distanceWeightCylinder = ui->distanceWeightCylinderLine->text().toStdString();
  maxIterationsCylinder = ui->maxIterationsCylinderLine->text().toStdString();
  distanceThresholdCylinder = ui->distanceThresholdCylinderLine->text().toStdString();
  minRadius = ui->minRadiusCylinderLine->text().toStdString();
  maxRadius = ui->maxRadiusCylinderLine->text().toStdString();

  std::string out = output_dir;
  out += "/trunk_parameters.txt";

  std::ofstream ofs(out.c_str());
  ofs << kSearch << std::endl

      << distanceWeight << std::endl
      << maxIterations << std::endl
      << distanceThreshold << std::endl

      << distanceWeightCylinder << std::endl
      << maxIterationsCylinder << std::endl
      << distanceThresholdCylinder << std::endl
      << minRadius << std::endl
      << maxRadius << std::endl;

  ofs.close();

  close();
}

void Visualizer::on_clearButton_clicked() {
  ui->kSearchLine->clear();

  ui->distanceWeightLine->clear();
  ui->maxIterationsLine->clear();
  ui->distanceThresHoldLine->clear();

  ui->distanceWeightCylinderLine->clear();
  ui->maxIterationsCylinderLine->clear();
  ui->distanceThresholdCylinderLine->clear();
  ui->minRadiusCylinderLine->clear();
  ui->maxRadiusCylinderLine->clear();
}

void Visualizer::getDefaultParameters(std::string& inputPath) {
  std::ifstream file(inputPath.c_str());

  std::string kSearch;

  std::string distanceWeight;
  std::string maxIterations;
  std::string distanceThreshold;

  std::string distanceWeightCylinder;
  std::string maxIterationsCylinder;
  std::string distanceThresholdCylinder;
  std::string minRadius;
  std::string maxRadius;

  while (file >> kSearch >> distanceWeight >> maxIterations >> distanceThreshold >> distanceWeightCylinder >> maxIterationsCylinder >> distanceThresholdCylinder >> minRadius >>
         maxRadius) {
  }

  ui->kSearchLine->setText(kSearch.c_str());

  ui->distanceWeightLine->setText(distanceWeight.c_str());
  ui->maxIterationsLine->setText(maxIterations.c_str());
  ui->distanceThresHoldLine->setText(distanceThreshold.c_str());

  ui->distanceWeightCylinderLine->setText(distanceWeightCylinder.c_str());
  ui->maxIterationsCylinderLine->setText(maxIterationsCylinder.c_str());
  ui->distanceThresholdCylinderLine->setText(distanceThresholdCylinder.c_str());
  ui->minRadiusCylinderLine->setText(minRadius.c_str());
  ui->maxRadiusCylinderLine->setText(maxRadius.c_str());
}
