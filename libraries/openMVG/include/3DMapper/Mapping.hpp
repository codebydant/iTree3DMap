#pragma once
#ifndef MAPPING_HPP
#define MAPPING_HPP
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>
#include <string>

namespace itreemap {

class OpenMVGInterface {
 public:
  OpenMVGInterface() {}

  void compute_sfm(std::string &input_dir, std::string &project_dir, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D);
  void export_pcd_file(std::string &project_pcd_file, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D);
  void export_ply_file(std::string &project_plyfile, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Map3D);

 private:
  void execute_openMVG_sfm(std::string &openMVG_pipeline, std::string &input_dir, std::string &output_dir);
};
}  // namespace itreemap
#endif