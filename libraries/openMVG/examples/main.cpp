#include "3DMapper/Mapping.hpp"

int main(int argc, char **argv) {
  argparse::ArgumentParser arg_parser(argv[0]);

  arg_parser.add_description("The program execute an incremental structure from motion approach using OpenMVG");

  arg_parser.add_argument("--project-name").default_value(std::string("custom_project")).help("project name description");
  arg_parser.add_argument("-i", "--images-dir").required().help("path to images folder");
  arg_parser.add_argument("--focal-length").default_value(float(30)).scan<'g', float>().help("camera focal length");
  arg_parser.add_argument("-o", "--output-dir").required().help("output dir to save project");
  arg_parser.add_argument("-d", "--display").default_value(false).implicit_value(true).help("display an input cloud in the pcl visualizer");

  try {
    arg_parser.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << arg_parser;
    std::exit(0);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map3D = compute_sfm(arg_parser);

  return 0;
}
