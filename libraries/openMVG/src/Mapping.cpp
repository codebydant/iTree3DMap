#include "3DMapper/Mapping.hpp"

#include "cloudparse/parser.hpp"
#include "common/colors.h"
#include "common/ospath.h"

namespace itreemap {

void OpenMVGInterface::execute_openMVG_sfm(std::string &openMVG_pipeline, std::string &input_dir, std::string &output_dir) {
  std::string command = "python3 ";
  command += openMVG_pipeline;
  command += input_dir;
  command += " ";
  command += output_dir;

  std::cout << blue << "\n3D Mapping with openMVG initializing..." << reset << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  int dont_care = std::system(command.c_str());
  if (dont_care > 0) {
    PCL_ERROR("Failed. SfM_SequentialPipeline.py not found\n");
    std::exit(-1);
  }

  PCL_INFO("\n3D Mapping --> [COMPLETE].");
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  PCL_INFO("\n3D mapping time: %li %s", difference, "seconds");
}

void OpenMVGInterface::compute_sfm(std::string &input_dir, std::string &project_dir, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D) {
  std::cout << "\n************************************************" << std::endl;
  std::cout << "              3D MAPPING                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  // create project directory
  boost::filesystem::create_directories(project_dir);

  // create cloud parser object
  CloudParserLibrary::ParserCloudFile cloud_parser;

  /*
  check if project files already exists
  if does, then load the pre-generated cloud file into the 3DMap object
  */
  std::string project_pcd_file = os::path::join(project_dir, "3D_Mapping", "MAP3D.pcd");
  if (boost::filesystem::exists(project_dir) && boost::filesystem::exists(project_pcd_file)) {
    std::cout << blue << "\nProject Found!" << reset << std::endl;
    cloud_parser.load_cloudfile(project_pcd_file, Map3D);

    /*
    if there is not a project defined run the openMVG structure from motion pipeline
    */
  } else {
    // Choose SFM approach - Incremental or Global
    std::string openMVG_pipeline = "SfM_SequentialPipeline.py ";
    // std::string openMVG = "SfM_GlobalPipeline.py ";

    // execute structure from motion pipeline
    OpenMVGInterface::execute_openMVG_sfm(openMVG_pipeline, input_dir, project_dir);

    // read generated cloud file from the openMVG project
    std::cout << "\nGetting cloud from 3D reconstruction..." << std::endl;
    std::string polyFile = os::path::join(project_dir, "reconstruction_sequential", "colorized.ply");
    cloud_parser.load_cloudfile(polyFile.c_str(), Map3D);

    for (const auto &point : *Map3D) std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  }

  // set cloud metadata
  Map3D->width = (int)Map3D->points.size();
  Map3D->height = 1;
  Map3D->is_dense = false;

  if (Map3D->points.size() <= 0) {
    PCL_ERROR("Could not load: colorized.ply\n");
    std::exit(-1);
  }
}

void OpenMVGInterface::export_pcd_file(std::string &project_pcd_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D) {
  // export 3DMap in .pcd format
  pcl::io::savePCDFileBinary(project_pcd_file.c_str(), *Map3D);
}

void OpenMVGInterface::export_ply_file(std::string &project_plyfile, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D) {
  // export 3DMap in .ply format
  pcl::PLYWriter writer;
  writer.write(project_plyfile, *Map3D, false, false);
}
}  // namespace itreemap