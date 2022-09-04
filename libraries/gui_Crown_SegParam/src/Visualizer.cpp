#include "include/Visualizer.h"

#include "ui_Visualizer.h"

Visualizer::Visualizer(int argc, char** argv, QWidget* parent) : QWidget(parent), ui(new Ui::Visualizer) {
  ui->setupUi(this);

  std::string crownDefaultParameters = argv[1];
  output_dir = argv[2];
  getDefaultParameters(crownDefaultParameters);

  init();
}

Visualizer::~Visualizer() { delete ui; }

void Visualizer::init() {
  ui->epsLine->setValidator(new QDoubleValidator(0, 100, 2, this));
  ui->minPtsLine->setValidator(new QIntValidator(0, 900, this));
  ui->minPtsAuxLine->setValidator(new QIntValidator(0, 900, this));
  ui->octreeReLine->setValidator(new QIntValidator(0, 900, this));
}

void Visualizer::on_okButton_clicked() {
  eps = ui->epsLine->text().toStdString();
  octreeResolution = ui->octreeReLine->text().toStdString();
  minPts = ui->minPtsLine->text().toStdString();
  minPtsAux = ui->minPtsAuxLine->text().toStdString();

  std::string out = output_dir;
  out += "/crown_parameters.txt";

  std::ofstream ofs(out.c_str());

  ofs << octreeResolution << std::endl << eps << std::endl << minPtsAux << std::endl << minPts << std::endl;
  ofs.close();

  close();
}

void Visualizer::getDefaultParameters(std::string& inputPath) {
  std::ifstream file(inputPath.c_str());

  std::string eps;
  std::string minPts;
  std::string minPtsAux;
  std::string octreeR;

  while (file >> octreeR >> eps >> minPtsAux >> minPts) {
  }

  ui->epsLine->setText(eps.c_str());
  ui->minPtsLine->setText(minPts.c_str());
  ui->minPtsAuxLine->setText(minPtsAux.c_str());
  ui->octreeReLine->setText(octreeR.c_str());

  file.close();
}
