#pragma once
#ifndef SCALE_fACTOR_HPP
#define SCALE_fACTOR_HPP
#include <tinyxml2.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "argparse/argparse.hpp"
#include "common/colors.h"

namespace itreemap {

class ScaleFactorModule {
 public:
  ScaleFactorModule() {}
  float world_reference = 0;
  std::vector<std::string> images_filenames;
  bool convert_sfm_data_to_xml(std::string& project_dir) {
    std::cout << "\n************************************************" << std::endl;
    std::cout << "              SCALE FACTOR                      " << std::endl;
    std::cout << "************************************************" << std::endl;

    std::cout << blue << "Converting sfm_data.bin to sfm_data.xml..." << reset << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    std::string command = "openMVG_main_ConvertSfM_DataFormat -i ";
    command += project_dir;
    command += "/reconstruction_sequential/sfm_data.bin -o ";
    command += project_dir;
    command += "/reconstruction_sequential/sfm_data.xml -V -I -E -S";

    auto start = std::chrono::high_resolution_clock::now();
    int dont_care = std::system(command.c_str());
    if (dont_care > 0) {
      PCL_ERROR("Failed. Could not convert sfm_data.bin to xml\n");
      return false;
    }

    std::cout << "Created xml file in:" << yellow << project_dir << "/reconstruction_sequential/sfm_data.xml" << reset << std::endl;
    return true;
  }

  bool loadSFM_XML_Data(cv::Mat_<double>& intrinsic, std::vector<cv::Matx34d>& cameras_poses, std::string& output_dir) {
    // Empty document
    tinyxml2::XMLDocument xml_doc;
    // pcl::PolygonMesh polyMesh;

    /*READING FILE*/
    std::string sfm_data = output_dir;
    sfm_data += "/reconstruction_sequential/sfm_data.xml";

    // Load xml document
    tinyxml2::XMLError eResult = xml_doc.LoadFile(sfm_data.c_str());
    if (eResult != tinyxml2::XML_SUCCESS) {
      std::cout << red << "Error: Could not find a xml file." << reset << std::endl;
      sfm_data.clear();
      return false;
    }

    std::cout << yellow << "sfm_data.xml:" << std::endl;
    std::cout << "<views>" << std::endl;
    std::cout << "<intrinsics>" << std::endl;
    std::cout << "<extrinsics>" << reset << std::endl;
    // std::cout << "<structure>" << reset << std::endl;

    std::cout << "\nFounding root tag <cereal> ..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    // Root xml document
    tinyxml2::XMLNode* root = xml_doc.FirstChildElement("cereal");
    if (root != nullptr) {
      std::cout << yellow << "Found: tag <cereal>" << reset << std::endl;
      std::cout << "\nFounding <views> tag ..." << std::endl;
      std::cout << "------------------------------------------" << std::endl;

      /*VIEWS DATA*/
      // Root views xml document
      tinyxml2::XMLElement* views = root->FirstChildElement("views");
      if (views == nullptr) {
        std::cout << red << "Error: <views> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      std::cout << yellow << "Found: tag <views>" << reset << std::endl;

      // Iterate over <views> tag
      for (tinyxml2::XMLElement* child = views->FirstChildElement(); child != NULL; child = child->NextSiblingElement()) {
        // Root views-data xml document
        tinyxml2::XMLElement* data = child->FirstChildElement("value")->FirstChildElement("ptr_wrapper")->FirstChildElement("data");
        if (data == nullptr) {
          std::cout << red << "Error: <data> tag was not found in xml document." << reset << std::endl;
          return false;
        }

        std::string filename;
        filename = data->FirstChildElement("filename")->GetText();
        images_filenames.push_back(filename);
      }

      std::cout << "\nFounding <intrinsics> tag ..." << std::endl;
      std::cout << "------------------------------------------" << std::endl;

      /*INTRINSICS DATA*/
      // Root intrinsics xml document
      tinyxml2::XMLElement* intrinsics = root->FirstChildElement("intrinsics");
      if (intrinsics == nullptr) {
        std::cout << red << "Error: <intrinsics> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      std::cout << yellow << "Found: tag <intrinsics>" << reset << std::endl;

      // Root intrinsics data xml document
      tinyxml2::XMLElement* data = intrinsics->FirstChildElement("value0")->FirstChildElement("value")->FirstChildElement("ptr_wrapper")->FirstChildElement("data");
      if (data == nullptr) {
        std::cout << red << "Error: <data> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      double f, cx, cy;
      data->FirstChildElement("focal_length")->QueryDoubleText(&f);

      // Root intrinsics data - principal point xml document
      tinyxml2::XMLElement* pp = data->FirstChildElement("principal_point");
      if (pp == nullptr) {
        std::cout << red << "Error: <principal_point> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      // Iterate over <principal_point> tag
      for (tinyxml2::XMLElement* child = pp->FirstChildElement(); child != NULL; child = child->NextSiblingElement()) {
        cx = cy;
        child->QueryDoubleText(&cy);
      }

      // Matrix K
      intrinsic = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);

      std::cout << "\nFounding <extrinsics> tag ..." << std::endl;
      std::cout << "------------------------------------------" << std::endl;

      /*EXTRINSICS DATA*/
      // Root extrinsics xml document
      tinyxml2::XMLElement* extrinsics = root->FirstChildElement("extrinsics");
      if (extrinsics == nullptr) {
        std::cout << red << "Error: <extrinsics> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      std::cout << yellow << "Found: tag <extrinsics>" << reset << std::endl;
      int id_camera;

      // Iterate over <extrinsics> tag
      for (tinyxml2::XMLElement* child = extrinsics->FirstChildElement(); child != NULL; child = child->NextSiblingElement()) {
        child->FirstChildElement("key")->QueryIntText(&id_camera);

        // Root extrinsics data - rotation xml document
        tinyxml2::XMLElement* rotation = child->FirstChildElement("value")->FirstChildElement("rotation");
        if (rotation == nullptr) {
          std::cout << red << "Error: <rotation> tag was not found in xml document." << reset << std::endl;
          return false;
        }

        double r11, r12, r13;
        double r21, r22, r23;
        double r31, r32, r33;

        // Iterate over <rotation> tag
        for (tinyxml2::XMLElement* child = rotation->FirstChildElement(); child != NULL; child = child->NextSiblingElement()) {
          // Iterate over <value> rotation tag
          for (tinyxml2::XMLElement* child2 = child->FirstChildElement(); child2 != NULL; child2 = child2->NextSiblingElement()) {
            r11 = r21;
            r21 = r31;
            r31 = r12;

            r12 = r22;
            r22 = r32;
            r32 = r13;

            r13 = r23;
            r23 = r33;
            child2->QueryDoubleText(&r33);
          }
        }

        double t1, t2, t3;

        // Root extrinsics data - traslation xml document
        tinyxml2::XMLElement* traslation = child->FirstChildElement("value")->FirstChildElement("center");
        if (traslation == nullptr) {
          std::cout << red << "Error: <center> tag was not found in xml document." << reset << std::endl;
          return false;
        }

        // Iterate over <traslation> tag
        for (tinyxml2::XMLElement* child = traslation->FirstChildElement(); child != NULL; child = child->NextSiblingElement()) {
          t1 = t2;
          t2 = t3;
          child->QueryDoubleText(&t3);
        }

        // Camera pose
        cv::Matx34d pose = cv::Matx34d(r11, r12, r13, t1, r21, r22, r23, t2, r31, r32, r33, t3);

        // Saving camera pose
        cameras_poses.push_back(pose);
      }
    } else {
      PCL_ERROR("Error: root of xml could not found. Must be: <cereal>");
      return false;
    }

    return true;
  }

 private:
};
}  // namespace itreemap
#endif