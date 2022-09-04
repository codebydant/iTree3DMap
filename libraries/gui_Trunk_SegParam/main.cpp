/*********************************
           HEADERS
**********************************/
#include <QApplication>

#include "include/Visualizer.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv) {
  QApplication container(argc, argv);

  if (argc < 3 or argc > 3) {
    std::cout << "Usage: <trunk parameters.txt> <output dir>" << std::endl;
    return -1;
  }

  Visualizer window(argc, argv);
  window.setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint);
  window.move(0, 0);
  window.show();
  window.setFixedSize(508, 340);

  return container.exec();

}  // end main
