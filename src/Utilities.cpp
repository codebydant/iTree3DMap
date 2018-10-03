#include "include/Utilities.h"

std::string output_dir;
std::string input_dir;
std::vector<std::string> images_filenames;
const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");
const std::string green("\033[0;32m");

// This function displays the help
void Utilities::help(){
  std::cout << green << "===============================================\n"
               "The program estimate dendrometric features of an individual tree from a 3D real scale mapping using "
               "a images sequence with:openMVG and PMVS2."
            << std::endl << "Enter:\n" << "<project name> and press \"Enter\"." << "\n"
            << "<path image sequence> and press \"Enter\"." << "\n"
            << "<output path> and press \"Enter\"." << std::endl
            << "<focal length> and press \"Enter\"." << std::endl
            << "===============================================" << reset << std::endl;
}

bool Utilities::run_openMVG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Map3D, std::string& output_path){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              3D MAPPING                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  //COMMAND LINE INPUT  
  std::string focal_length;
  std::string project_name; 
  auto start = std::chrono::high_resolution_clock::now();

  /*PROJECT NAME*/
  while(true){

    std::string answer;

    bool nameOk = false;
    if(project_name.size()<=0){

      std::cout << blue << "\nEnter the project name: (don't use space)" << reset << std::endl;
      std::cout << "------------------------------------------" << "\n" << "->" << std::flush;
      std::getline(std::cin, project_name);
      if(project_name.empty()){
        PCL_ERROR("Nothing entered.\n");
        project_name.clear();
        continue;
      }else if(project_name.size()<=1){
        PCL_ERROR("The input name is not valid. Please enter at least 2 characters!\n");
        project_name.clear();
        continue;
      }

      if(project_name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_")!= std::string::npos){
            PCL_ERROR("%s %s",project_name.c_str(),"is not a valid project name.\n");
            project_name.clear();
            continue;
        }
    }

    if(project_name.size()>0){

      std::cout << "\nproject name = " << yellow << project_name << reset << " are you sure? (yes/no)" << std::endl;
      std::cout << "->" << std::flush;
      std::getline(std::cin, answer);
      if(answer.empty()){
        PCL_ERROR("Nothing entered.\n");
        answer.clear();
        continue;
      }

      if(answer == "yes"){
        nameOk = true;
      }else if(answer == "no"){
        nameOk = false;
        project_name.clear();
        answer.clear();
        continue;
      }else{
        PCL_ERROR("%s %s",answer.c_str(),"is not a valid answer.\n");
        nameOk = false;
        answer.clear();
        continue;
      }
    }

    if(nameOk){
      std::cout << yellow << "Using project name as:" << project_name << reset << std::endl;
      break;
    }
  }

  /*INPUT PATH*/
  while(true){

    bool imagesOk = false;
    if(input_dir.size()<=0){

      std::cout << blue << "\nEnter the images directorie path:" << reset << std::endl;
      std::cout << "------------------------------------------" << "\n" << "->" << std::flush;
      std::getline(std::cin, input_dir);
      if(input_dir.empty()){
        PCL_ERROR("Nothing entered.\n");
        input_dir.clear();
        continue;
      }
    }

    if(input_dir.size()>0){

      boost::filesystem::path dirPath(input_dir);

      if(not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)){
        std::cout << red << "Error. cannot open directory: " << input_dir << reset <<std::endl;
        input_dir.clear();
        continue;
      }

      bool foundImages = false;

      for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(dirPath)){
        std::string extension = x.path().extension().string();
        boost::algorithm::to_lower(extension);
        if(extension == ".jpg" or extension == ".png"){
          imagesOk = true;
          foundImages = true;
          break;
        }else{
          foundImages = false;
          break;
        }
      }

      if(not foundImages){
        std::cout << red << "Unable to find images files in directory (\"" << input_dir << "\")."
                  << reset <<std::endl;
        imagesOk = false;
        input_dir.clear();
        continue;
      }
    }

    if(imagesOk){
      std::cout << yellow << "Found images file." << reset << std::endl;
      break;
    }
  }

  /*OUTPUT PATH*/
  while(true){

    bool dirOk = false;
    if(output_dir.size()<=0){

      std::cout << blue << "\nEnter the output directorie path:" << reset << std::endl;
      std::cout << "------------------------------------------" << "\n" << "->" << std::flush;
      std::getline(std::cin, output_dir);
      if(output_dir.empty()){
        PCL_ERROR("Nothing entered.");
        output_dir.clear();
        dirOk = false;
        continue;
      }
    }

    if(output_dir.size()>0){

      boost::filesystem::path dirPath(output_dir);

      if(not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)){
        std::cout << red <<"Error. cannot found directory: " << output_dir << reset << std::endl;
        output_dir.clear();
        dirOk = false;
        continue;
      }else{
        dirOk = true;
      }
    }

    if(dirOk){
      break;
    }
  }

  /*FOCAL LENGTH*/
/*
  double n=-1;  
  while(true){

    if(focal_length.size()<=0){

        std::cout << blue << "\nEnter the focal length:\n" << reset
                  << "------------------------------------------" << "\n" << "->" << std::flush;
        std::string f;
        std::getline(std::cin,f);
        n = std::strtod(f.c_str(),NULL);

        if(not is_number(f)){
            PCL_ERROR("Error: enter a valid focal length\n");
            f.clear();
            n = -1;
            continue;
        }

        if(n<=0){
            PCL_ERROR("Error: enter a valid focal_length\n");
            n = -1;
            f.clear();
            continue;
          }else{
            //std::ostringstream strs;
            //strs << n;
            //focal_length = strs.str();
            focal_length = f;
            //std::cout << "Using focal length:" << yellow << focal_length << reset << std::endl;
          }
   }

    if(focal_length.size()>0){

      std::string answer;
      bool focalOk = false;

      std::cout << "\nfocal length = " << yellow << focal_length << reset << " are you sure? (yes/no)" << std::endl;
      std::cout << "->" << std::flush;
      std::getline(std::cin, answer);
      if(answer.empty()){
        PCL_ERROR("Nothing entered.\n");
        answer.clear();
        continue;
      }

      if(answer == "yes"){
        focalOk = true;
      }else if(answer == "no"){
        focalOk = false;
        focal_length.clear();
        n = -1;
        answer.clear();
        continue;
      }else{
        PCL_ERROR("%s %s",answer.c_str(),"is not a valid answer.\n");
        focalOk = false;
        n = -1;
        answer.clear();
        continue;
      }

    if(focalOk){
      std::cout << yellow << "Using focal length:" << focal_length << reset << std::endl;
      break;
    }
   }
 }

  int dont_care;
  std::string folder_name = "mkdir ";
  folder_name += output_dir;
  folder_name += "/";
  folder_name += project_name;

  dont_care = std::system(folder_name.c_str());
*/  
  output_dir += "/";
  output_dir += project_name;
  output_path = output_dir;
/*
  std::string output_pcd_files = "3D_Mapping";
  std::string folder_name2 = "mkdir ";
  folder_name2 += output_dir;
  folder_name2 += "/";
  folder_name2 += output_pcd_files;

  dont_care = std::system(folder_name2.c_str());

  std::string command = "python ";
  std::string openMVG = "/home/daniel/Documents/iTree3DMap/libraries/openMVG/build/software/SfM/SfM_SequentialPipeline.py ";
 // std::string openMVG = "/home/daniel/Documents/iTree3DMap/libraries/openMVG/build/software/SfM/SfM_GlobalPipeline.py ";

  command += openMVG;
  command += input_dir;
  command += " ";
  command += output_dir;
  command += " ";
  command += focal_length;

  std::cout << blue << "\n3D Mapping with openMVG initializing..." << reset << std::endl;
  dont_care = std::system(command.c_str());

  if(dont_care > 0){
    PCL_ERROR("Failed. SfM_SequentialPipeline.py not found\n");
    return false;
  }

  */

  PCL_INFO("\n3D Mapping --> [COMPLETE].");
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  PCL_INFO("\n3D mapping time: %li %s",difference,"seconds");
  std::cout << std::endl;

  pcl::console::TicToc tt;

  std::string prefix = output_dir;
  prefix += "/3D_Mapping/";
  std::string prefix1 = prefix;
  prefix1 +="MAP3D.pcd";

  std::string prefix2 = prefix;
  prefix2 += "MAP3D.ply";

  std::string polyFile = output_dir;
  polyFile += "/reconstruction_sequential/colorized.ply";

  pcl::PolygonMesh polyMesh;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

  std::cout << "\nGetting cloud from 3D reconstruction..." << std::endl;

  pcl::io::loadPLYFile(polyFile.c_str(),*Map3D);
  if(Map3D->points.size()<=0 or Map3D->points.at(0).x <=0 and Map3D->points.at(0).y <=0 and Map3D->points.at(0).z <=0){
      pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...");
      pcl::io::loadPolygonFile(polyFile.c_str(), polyMesh);
      pcl::fromPCLPointCloud2(polyMesh.cloud, *Map3D);
      if(Map3D->points.size()<=0 or Map3D->points.at(0).x <=0 and Map3D->points.at(0).y <=0 and Map3D->points.at(0).z <=0){
          pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...");
          pcl::PLYReader plyRead;
          plyRead.read(polyFile.c_str(),*Map3D);
          if(Map3D->points.size()<=0 or Map3D->points.at(0).x <=0 and Map3D->points.at(0).y <=0 and Map3D->points.at(0).z <=0){
              pcl::console::print_error("\nError. ply file is not compatible.\n");
              return -1;
            }
        }
    }

  if(Map3D->points.size()<=0){
    PCL_ERROR("Could not load: colorized.ply\n");
    return false;
  }

  pcl::console::print_info ("[done, ");
  pcl::console::print_value ("%g", tt.toc ());
  pcl::console::print_info (" ms : ");
  pcl::console::print_value ("%d", Map3D->points.size ());
  pcl::console::print_info (" points]\n");

  pcl::io::savePCDFileBinary(prefix1.c_str(), *Map3D);
  pcl::PLYWriter writer;
  writer.write(prefix2.c_str(), *Map3D, false, false);

  return true;
}

bool Utilities::getScaleFactor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Map3D,double& scale_factor){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SCALE FACTOR                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << blue << "Converting sfm_data.bin to sfm_data.xml..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  /*
  ./openMVG_main_ConvertSfM_DataFormat
  Usage: ./openMVG_main_ConvertSfM_DataFormat
  [-i|--input_file] path to the input SfM_Data scene
  [-o|--output_file] path to the output SfM_Data scene
       .json, .bin, .xml, .ply, .baf

  [Options to export partial data (by default all data are exported)]

  Usable for json/bin/xml format
  [-V|--VIEWS] export views
  [-I|--INTRINSICS] export intrinsics
  [-E|--EXTRINSICS] export extrinsics (view poses)
  [-S|--STRUCTURE] export structure
  [-C|--CONTROL_POINTS] export control points
  */
/*
  std::string command = "/home/daniel/Documents/iTree3DMap/libraries/openMVG/build/Linux-x86_64-Release/openMVG_main_ConvertSfM_DataFormat -i ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.bin -o ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.xml -V -I -E -S";

  int dont_care = std::system(command.c_str());
  if(dont_care > 0){
   PCL_ERROR("Failed. Could not convert sfm_data.bin to xml\n");
   return false;
  }

  std::cout << "Created xml file in:" << yellow << output_dir
            << "/reconstruction_sequential/sfm_data.xml" << reset << std::endl;
*/
  cv::Mat_<double> intrinsic;
  std::vector<cv::Matx34d> cameras_poses;

  std::cout << blue << "\nGetting data from sfm_data.xml..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

  bool success = Utilities::loadSFM_XML_Data(intrinsic,cameras_poses);
  if(not success){
    PCL_ERROR("Could not get a scale factor.\n");
    return false;
  }
  /*
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(Map3D);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-900, 0);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);
  */
  PCL_INFO("\nImages: %lu",images_filenames.size());
  PCL_INFO("\nCamera Poses: %lu %s",cameras_poses.size(),"cameras");
  std::cout << "\nIntrinsic camera:\n" << intrinsic << std::endl;

  std::cout << blue << "\nGetting scale factor..." << reset << std::endl;
  std::cout << "------------------------------------------";

  std::string world_reference;

  double n=-1;
  while(true){

    if(world_reference.size()<=0){

        std::cout << blue << "\nEnter the world reference (cm):\n" << reset
                  << "------------------------------------------" << "\n" << "->" << std::flush;
        std::string f;
        std::getline(std::cin,f);
        n = std::strtod(f.c_str(),NULL);

        if(not is_number(f)){
            PCL_ERROR("Error: enter a valid world reference\n");
            f.clear();
            n = -1;
            continue;
        }

        if(n<=0){
            PCL_ERROR("Error: enter a valid world reference\n");
            n = -1;
            f.clear();
            continue;
          }else{
            world_reference = f;
          }
   }

   if(world_reference.size()>0){

        std::string answer;
        bool WRefOk = false;

      std::cout << "\nworld reference = " << yellow << world_reference << reset << " are you sure? (yes/no)" << std::endl;
      std::cout << "->" << std::flush;
      std::getline(std::cin, answer);
      if(answer.empty()){
        PCL_ERROR("Nothing entered.\n");
        answer.clear();
        continue;
      }

      if(answer == "yes"){
        WRefOk = true;
      }else if(answer == "no"){
        WRefOk = false;
        world_reference.clear();
        n = -1;
        answer.clear();
        continue;
      }else{
        PCL_ERROR("%s %s",answer.c_str(),"is not a valid answer.\n");
        WRefOk = false;
        n = -1;
        answer.clear();
        continue;
      }

    if(WRefOk){
      std::cout << yellow << "Using world reference:" << world_reference << reset << std::endl;
      break;
    }
   }
  }  

  std::string img_selected;
  int im_ID = -1;

  cv::Point2d img_p1,img_p2;
  std::vector<cv::Point2d> pts_for_circle1;
  std::vector<cv::Point2d> pts_for_circle2;
  int numImg = -1;

  while(true){

      bool bestImage = false;

      if(img_selected.size() <= 0){

          PCL_INFO("\nChoose a image pattern reference\n");

          std::string command2 = "/home/daniel/Documents/iTree3DMap/libraries/control_Image_Registration/build/control_point_Reg ";
          command2 += output_dir;
          command2 += "/reconstruction_sequential/sfm_data.xml ";
          command2 += output_dir;

          int dont_care = std::system(command2.c_str());
          if(dont_care > 0){
              PCL_ERROR("Failed. Could not find control_point_Reg.bin\n");
              return false;
            }

          std::string image_path = output_dir;
          image_path += "/image_selected.txt";

          std::ifstream img_path(image_path.c_str());
          if(!img_path.is_open()){
              std::cout << red << "Error: Could not find "<< image_path << reset << std::endl;
          }

          int img_id;
          std::string path;

          while(img_path >> path >> img_id){}

          std::string answer;
          bool WRefOk = false;

          while(true){

              std::cout << "\nimage selected = " << yellow << img_id << reset << " are you sure? (yes/no)" << std::endl;
              std::cout << "->" << std::flush;
              std::getline(std::cin, answer);
              if(answer.empty()){
                  PCL_ERROR("Nothing entered.\n");
                  answer.clear();
                  continue;
                }

              if(answer == "yes"){
                  im_ID = img_id;
                  img_selected = path;
                  WRefOk = true;
                }else if(answer == "no"){
                  WRefOk = false;
                  img_selected.clear();
                  im_ID = -1;
                  n = -1;
                  answer.clear();
                  break;
                }else{
                  PCL_ERROR("%s %s",answer.c_str(),"is not a valid answer.\n");
                  WRefOk = false;
                  n = -1;
                  answer.clear();
                  continue;
                }

              if(WRefOk){
                  //std::cout << yellow << "Using image reference:" << img_selected << reset << std::endl;
                  bestImage = true;
                  break;
              }
          }

      }

      if(img_selected.size() > 0){

          numImg = im_ID;

          bool foundBestImage = false;

          std::string images_path = input_dir;
          images_path += "/";
          images_path += images_filenames.at(numImg);

          //------------------------------------------------------
          // CIRCLE PATTERN DETECTION
          //------------------------------------------------------

          cv::Mat img = cv::imread(images_path.c_str(),1);
          if(!img.data ){
            std::cout << red <<"Could not open or find the image" << reset << std::endl;
            return false;
          }

          cv::Mat img_copy = img.clone();
          cv::Mat gray;

          cv::cvtColor(img_copy,gray,CV_BGR2GRAY);
          std::vector<cv::Vec3f> circles(2);

          std::cout << "\nFiltering...Canny!" << std::endl;
          cv::Canny(gray, gray, 100, 100*2, 3 );
          std::cout << "Filtering...GaussianBlur!" << std::endl;
          cv::GaussianBlur(gray,gray,cv::Size(7,7),2,2);

          std::cout << "Detecting circles in image..." << std::endl;
          cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,1, gray.rows/16, 200, 100, 0, 150);

          cv::Point2d center_pattern1,center_pattern2;
          int radius = 0;

          for(size_t i = 0; i < circles.size(); i++ ){

            cv::Point2d center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            center_pattern2 = center_pattern1;
            center_pattern1 = cv::Point2d(center.x,center.y);
            radius = cvRound(circles[i][2]);
            // circle center
            cv::circle(img_copy, center, 1, cv::Scalar(0,255,0),10);
            // circle outline
            cv::circle(img_copy, center, radius,cv::Scalar(0,255,0),30);
          }

          std::cout << "\nNum of circles detect:" << circles.size() << std::endl;
          std::cout << "Circle 1 center:" << center_pattern1 << std::endl;
          std::cout << "Circle 2 center:" << center_pattern2 << std::endl;

          cv::line(img_copy,center_pattern1,center_pattern2,cv::Scalar(0,255,0),30);

          Display* d = XOpenDisplay(NULL);
          Screen*  s = DefaultScreenOfDisplay(d);

          int x = s->width;

          cv::namedWindow("pattern",CV_WINDOW_NORMAL);
          cv::resizeWindow("pattern",640,480);
          cv::moveWindow("pattern",std::round(x/2),0);
          cv::imshow("pattern",img_copy);
          cv::waitKey(0);
          cv::destroyAllWindows();

          while(true){

            std::string answer;

            std::cout << yellow << "\nIs this lenght OK? (yes/no):" << reset << std::endl;
            std::cout << "------------------------------------------\n" << "->" << std::flush;
            std::getline(std::cin, answer);
            if(answer.empty()){
              std::cout << red << "Nothing entered." << reset << std::endl;
              answer.clear();
              std::cin.clear();
              continue;
            }

            if(answer == "yes"){

                std::string image_pattern_path = images_path;
                //std::cout << "before pop back:" << image_pattern_path << std::endl;
                std::string backG = images_filenames.at(0);
                //std::cout << "string reference:" << backG << std::endl;
                for(int i=0;i<backG.size();i++){
                  int j=i;
                  image_pattern_path.pop_back();
                }
                //std::cout << "after pop back:" << image_pattern_path << std::endl;
                std::string img_feat_filename;
                for(int i=0;i<images_filenames.size();i++){
                  std::string comp = image_pattern_path;
                  comp += images_filenames.at(i);
                  if(comp == images_path){
                    //std::cout << "Found:" << comp << std::endl;
                    img_feat_filename = images_filenames.at(i);
                    break;
                  }
                }

                img_feat_filename.pop_back();
                img_feat_filename.pop_back();
                img_feat_filename.pop_back();

                std::string feature_path = output_dir;
                feature_path += "/matches/";
                feature_path += img_feat_filename;
                feature_path += "feat";

                float x_, y_,s,orientation;
                std::vector<cv::Point2d> image_points;

                std::ifstream file(feature_path.c_str());
                if(!file.is_open()){
                  std::cout << red << "Error: Could not find "<< feature_path << reset << std::endl;
                  feature_path.clear();
                  break;
                }

                while(file >> x_ >> y_ >> s >> orientation){
                    image_points.push_back(cv::Point2f(x_,y_));
                }

                std::cout << yellow << "\nFeature selected:" << reset << feature_path << std::endl;
                std::cout << yellow << "Image points: " << reset << image_points.size() << " points" << std::endl;

                for(size_t i=0;i<image_points.size();i++){

                  cv::Point2d pt = image_points.at(i);
                  double error = cv::norm(cv::Mat(pt),cv::Mat(center_pattern1));
                  if(error < 30){
                    pts_for_circle1.push_back(pt);
                  }else{
                    continue;
                  }
                }

                pts_for_circle1.push_back(center_pattern1);

                for(size_t i=0;i<image_points.size();i++){

                  cv::Point2d pt = image_points.at(i);
                  double error = cv::norm(pt - center_pattern2);
                  if(error < 30){
                    pts_for_circle2.push_back(pt);
                  }else{
                    continue;
                  }

                }

                pts_for_circle2.push_back(center_pattern2);

              foundBestImage = true;
              break;
            }else if(answer == "no"){
              answer.clear();
              foundBestImage = false;
              img_selected.clear();
              break;
            }else{
              std::cout << red << "Is not a valid answer" << reset << std::endl;
              answer.clear();
              continue;
            }
          }


          if(foundBestImage){
              PCL_INFO("Id pose: %i",numImg);
              //std::cout << "\nFilename: " << images_filenames.at(numImg) << std::endl;
              std::cout << "\nCamera pose:\n" << cameras_poses[numImg] << std::endl;
              break;
          }
       }
    }

  cv::destroyAllWindows();

  cv::Matx34d pose =cameras_poses[numImg];
  cv::Mat Rc = cv::Mat(pose.get_minor<3,3>(0,0));
  cv::Mat C = cv::Mat(pose.get_minor<3,1>(0,3));

  cv::Mat R = Rc.t();
  cv::Mat t = -R*C;

  pose = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0),
                     R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1),
                     R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2));

  cv::Mat rvec;
  cv::Rodrigues(pose.get_minor<3,3>(0,0),rvec);
  cv::Mat tvec(pose.get_minor<3,1>(0,3));

  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> points3d;

  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=Map3D->begin(); it!=Map3D->end(); ++it){
      points3d.push_back(cv::Point3d(it->x,it->y,it->z));
    }

  cv::projectPoints(points3d,rvec,tvec,intrinsic,cv::Mat(),projected_points);
  std::cout << yellow << "\nPoints projected:" << reset << projected_points.size()<< std::endl;
  double error;
  pcl::PointXYZ ptt1,pt2;

  std::map<double,std::pair<std::pair<cv::Point2d,cv::Point2d>,cv::Point3d>> p1_map;

  std::cout << yellow << "\ncheck if point reprojection error is small enough..." << reset << std::endl;

  std::string dendrometric_results = output_dir;
  dendrometric_results += "/pts_for_circle1.txt";

  ofstream cir(dendrometric_results.c_str());

  std::string dendrometric_results2 = output_dir;
  dendrometric_results2 += "/pts_for_circle2.txt";

  ofstream cir2(dendrometric_results2.c_str());

  //check if point reprojection error is small enough
  for(std::vector<cv::Point2d>::iterator it = pts_for_circle1.begin();it!=pts_for_circle1.end();++it){

      cv::Point2d pt2d_circle1 = cv::Point2d(it->x,it->y);
      cir << "ptss:" << pt2d_circle1 << std::endl;

      for(int i=0;i<projected_points.size();i++){

          error = cv::norm(projected_points.at(i) - pt2d_circle1);
          p1_map[error] = std::make_pair(std::make_pair(pt2d_circle1,projected_points[i]),points3d.at(i));
        }
    }
  cir.close();

  std::map<double,std::pair<std::pair<cv::Point2d,cv::Point2d>,cv::Point3d>> p2_map;

  for(std::vector<cv::Point2d>::iterator it = pts_for_circle2.begin();it!=pts_for_circle2.end();++it){

      cv::Point2d pt2d_circle2 = cv::Point2d(it->x,it->y);
      cir2 << "ptss:" << pt2d_circle2 << std::endl;

      for(int i=0;i<projected_points.size();i++){

          error = cv::norm(projected_points.at(i) - pt2d_circle2);
          p2_map[error] = std::make_pair(std::make_pair(pt2d_circle2,projected_points.at(i)),points3d.at(i));

        }
    }

  std::map<double,std::pair<std::pair<cv::Point2d,cv::Point2d>,cv::Point3d>>::iterator it1 = p1_map.begin();
  ptt1 = pcl::PointXYZ(it1->second.second.x,it1->second.second.y,it1->second.second.z);
  std::cout << "\nOriginal point1:" << it1->second.first.first << " projected point1:" << it1->second.first.second
            << std::endl;
  std::cout << "Error: " << it1->first << std::endl;
  std::cout << "Point3D:" << it1->second.second << std::endl;

  std::map<double,std::pair<std::pair<cv::Point2d,cv::Point2d>,cv::Point3d>>::iterator it2 = p2_map.begin();
  pt2 = pcl::PointXYZ(it2->second.second.x,it2->second.second.y,it2->second.second.z);
  std::cout << "\nOriginal point2:" << it2->second.first.first << " projected point2:" << it2->second.first.second
            << std::endl;
  std::cout << "Error: " << it2->first << std::endl;
  std::cout << "Point3D:" << it2->second.second << std::endl;

  ptt1.z = pt2.z;
  ptt1.x = pt2.x;

  vtkVisualizer(Map3D,ptt1,pt2);

  double Ref_PCL = pcl::geometry::distance(ptt1,pt2);
  double W_reference = std::stold(world_reference);

  scale_factor = W_reference/Ref_PCL;
  PCL_INFO("\n\nModel reference: %f",Ref_PCL);
  PCL_INFO("\nscale_factor: %f",scale_factor);

  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  PCL_INFO("\nScale factor time: %lu %s",difference,"seconds");

  return true;

}

bool Utilities::loadSFM_XML_Data(cv::Mat_<double>& intrinsic,
                                 std::vector<cv::Matx34d>& cameras_poses){

  // Empty document
  tinyxml2::XMLDocument xml_doc;
  //pcl::PolygonMesh polyMesh;

  /*READING FILE*/
  std::string sfm_data = output_dir;
  sfm_data += "/reconstruction_sequential/sfm_data.xml";

  // Load xml document
  tinyxml2::XMLError eResult = xml_doc.LoadFile(sfm_data.c_str());
  if(eResult != tinyxml2::XML_SUCCESS){
    std::cout << red << "Error: Could not find a xml file." << reset << std::endl;
    sfm_data.clear();
    return false;
  }

  std::cout << yellow << "sfm_data.xml:" << std::endl;
  std::cout << "<views>" << std::endl;
  std::cout << "<intrinsics>" << std::endl;
  std::cout << "<extrinsics>" << reset << std::endl;
  //std::cout << "<structure>" << reset << std::endl;

  std::cout << "\nFounding root tag <cereal> ..." << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  // Root xml document
  tinyxml2::XMLNode * root = xml_doc.FirstChildElement("cereal");
  if(root!=nullptr){

    std::cout << yellow << "Found: tag <cereal>" << reset << std::endl;
    std::cout  << "\nFounding <views> tag ..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    /*VIEWS DATA*/
    // Root views xml document
    tinyxml2::XMLElement * views = root->FirstChildElement("views");
    if(views == nullptr){
      std::cout << red << "Error: <views> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    std::cout << yellow << "Found: tag <views>" << reset << std::endl;

    // Iterate over <views> tag
    for(tinyxml2::XMLElement* child = views->FirstChildElement();child != NULL;
        child = child->NextSiblingElement()){

      // Root views-data xml document
      tinyxml2::XMLElement * data = child->FirstChildElement("value")->FirstChildElement("ptr_wrapper")->FirstChildElement("data");
      if(data == nullptr){
        std::cout << red << "Error: <data> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      std::string filename;
      filename = data->FirstChildElement("filename")->GetText();
      images_filenames.push_back(filename);
    }

    std::cout  << "\nFounding <intrinsics> tag ..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    /*INTRINSICS DATA*/
    // Root intrinsics xml document
    tinyxml2::XMLElement * intrinsics = root->FirstChildElement("intrinsics");
    if(intrinsics == nullptr){
      std::cout << red << "Error: <intrinsics> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    std::cout << yellow << "Found: tag <intrinsics>" << reset << std::endl;

    // Root intrinsics data xml document
    tinyxml2::XMLElement * data = intrinsics->FirstChildElement("value0")->FirstChildElement("value")->FirstChildElement("ptr_wrapper")->FirstChildElement("data");
    if(data == nullptr){
      std::cout << red << "Error: <data> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    double f,cx,cy;
    data->FirstChildElement("focal_length")->QueryDoubleText(&f);

    // Root intrinsics data - principal point xml document
    tinyxml2::XMLElement * pp = data->FirstChildElement("principal_point");
    if(pp == nullptr){
      std::cout << red <<  "Error: <principal_point> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    // Iterate over <principal_point> tag
    for(tinyxml2::XMLElement* child = pp->FirstChildElement();child != NULL;
        child = child->NextSiblingElement()){
      cx=cy;
      child->QueryDoubleText(&cy);
    }

    // Matrix K
    intrinsic = (cv::Mat_<double>(3, 3) << f, 0, cx,
                                          0, f, cy,
                                          0, 0, 1);

    std::cout  << "\nFounding <extrinsics> tag ..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    /*EXTRINSICS DATA*/
    // Root extrinsics xml document
    tinyxml2::XMLElement * extrinsics = root->FirstChildElement("extrinsics");
    if(extrinsics == nullptr){
      std::cout << red << "Error: <extrinsics> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    std::cout << yellow << "Found: tag <extrinsics>" << reset << std::endl;
    int id_camera;

    // Iterate over <extrinsics> tag
    for(tinyxml2::XMLElement* child = extrinsics->FirstChildElement();child != NULL;
        child = child->NextSiblingElement()){

      child->FirstChildElement("key")->QueryIntText(&id_camera);

      // Root extrinsics data - rotation xml document
      tinyxml2::XMLElement * rotation = child->FirstChildElement("value")->FirstChildElement("rotation");
      if(rotation == nullptr){
        std::cout << red << "Error: <rotation> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      double r11,r12,r13;
      double r21,r22,r23;
      double r31,r32,r33;

      // Iterate over <rotation> tag
      for(tinyxml2::XMLElement* child = rotation->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        // Iterate over <value> rotation tag
        for(tinyxml2::XMLElement* child2 = child->FirstChildElement();child2 != NULL;
            child2 = child2->NextSiblingElement()){

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

      double t1,t2,t3;

      // Root extrinsics data - traslation xml document
      tinyxml2::XMLElement * traslation = child->FirstChildElement("value")->FirstChildElement("center");
      if(traslation == nullptr){
        std::cout << red << "Error: <center> tag was not found in xml document." << reset << std::endl;
        return false;
      }

      // Iterate over <traslation> tag
      for(tinyxml2::XMLElement* child = traslation->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        t1 = t2;
        t2 = t3;
        child->QueryDoubleText(&t3);
      }

      // Camera pose
      cv::Matx34d pose = cv::Matx34d(r11, r12, r13, t1,
                             r21, r22, r23, t2,
                             r31, r32, r33, t3);

      // Saving camera pose
      cameras_poses.push_back(pose);
    }

    //std::cout  << "\nFounding <structure> tag ..." << std::endl;
    //std::cout << "------------------------------------------" << std::endl;

    /*POINTCLOUD DATA*/
    /*
    // Root structure xml document
    tinyxml2::XMLElement * structure = root->FirstChildElement("structure");
    if(structure == nullptr){
      std::cout << red <<"Error: <structure> tag was not found in xml document." << reset << std::endl;
      return false;
    }

    std::cout << yellow << "Found: tag <structure>" << reset << std::endl;

    // Iterate over <structure> tag
    for(tinyxml2::XMLElement* child = structure->FirstChildElement();child != NULL;
        child = child->NextSiblingElement()){

      Point3DInMap * pt3d = new Point3DInMap();

      tinyxml2::XMLElement * X_root = child->FirstChildElement("value")->FirstChildElement("X");
      if(X_root==nullptr){
        std::cout << red << "Error: 3D point not found in <X> tag." << reset << std::endl;
        return false;
      }

      double x,y,z;

      // Iterate over <X> tag
      for(tinyxml2::XMLElement* child = X_root->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        x=y;
        y=z;
        eResult = child->QueryDoubleText(&z);   //z=coordinate;
      }

      // Filling pt feature
      pt3d->pt = cv::Point3d(x,y,z);

      // Root <observations> tag
      tinyxml2::XMLElement * Observ_root = child->FirstChildElement("value")->FirstChildElement("observations");
      if(Observ_root==nullptr){
        std::cout << "Error: 3D-2d view point not found in <observations> tag." << std::endl;
        continue;
      }

      // Iterate over <observations> tag
      for(tinyxml2::XMLElement* child = Observ_root->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        int img_id;
        child->FirstChildElement("key")->QueryIntText(&img_id);

        // Root value - <bservations> tag
        tinyxml2::XMLElement * id_feat_root = child->FirstChildElement("value");
        if(id_feat_root==nullptr){
          std::cout << red << "Error: <id_feat> tag not found." << reset << std::endl;
          return false;
        }

        int img_id_feat;
        id_feat_root->FirstChildElement("id_feat")->QueryIntText(&img_id_feat);

        // Root feature - <bservations> tag
        tinyxml2::XMLElement * feature_root = id_feat_root->FirstChildElement("x");
        if(feature_root==nullptr){
          std::cout << red << "Error: <x> tag not found." << reset << std::endl;
          return false;
        }

        double x,y;

        // Iterate over <x> tag
        for(tinyxml2::XMLElement* child = feature_root->FirstChildElement();child != NULL;
            child = child->NextSiblingElement()){

          x = y;
          eResult = child->QueryDoubleText(&y);   //y=coordinate;
        }

         // 2D Point - <bservations> tag
         cv::Point2d pt2d(x,y);
         std::map<const int,cv::Point2d> feature;
         feature[img_id_feat]=pt2d;

         pt3d->feat_ref[img_id]=feature;

      }

      // Saving pt3d
      pts3d.push_back(*pt3d);
    }
     */

  }else{
    PCL_ERROR("Error: root of xml could not found. Must be: <cereal>");
    return false;
  }

  return true;
}

bool Utilities::createPMVS_Files(){

  std::cout << "\n------------------------------------------" << std::endl;
  std::cout << blue <<"Creating files for PMVS2..." << reset << std::endl;

  std::string command2 = "../libraries/openMVG/openMVG_Build/Linux-x86_64-RELEASE/openMVG_main_openMVG2PMVS -i ";
  command2 += output_dir;
  command2 += "/reconstruction_sequential/sfm_data.bin -o ";
  command2 += output_dir;

  int dont_care = std::system(command2.c_str());

  if(dont_care > 0){
   PCL_ERROR("Failed. openMVG_main_openMVG2PMVS not found");
   return false;
  }

  return true;
}

bool Utilities::densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud4){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              DENSIFICATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "------------------------------------------" << std::endl;
  std::cout << blue << "Densify cloud process initializing..." << reset << std::endl;
  /*
  std::string command3 = "../libraries/pmvs2 ";
  command3 += output_dir;
  command3 += "/PMVS/ ";
  command3 += "pmvs_options.txt";

  int dont_care = std::system("chmod 771 ../libraries/pmvs2");
  dont_care = std::system(command3.c_str());

  if(dont_care > 0){
   std::cout << "Failed. ./pmvs2 no found" << std::endl;
   std::exit(-1);
  }
*/
  std::string cloudPLY = output_dir;
  cloudPLY += "/PMVS/models/pmvs_options.txt.ply";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::io::loadPLYFile(cloudPLY.c_str(),*output_cloud);

  // Find the planar coefficients for floor plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold(100);
  seg.setInputCloud (output_cloud);
  seg.segment (*floor_inliers, *coefficients);
  std::cout << "Floor Plane Model coefficients:\n" << coefficients->values[0] << "\n"
            << coefficients->values[1] << "\n"
            << coefficients->values[2] << "\n"
            << coefficients->values[3] << std::endl;

  Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  std::cout << "\nFloor plane normal vector:\n" << floor_plane_normal_vector << std::endl;

  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = 1.0;

  std::cout << "\nXY plane normal vector:\n" <<xy_plane_normal_vector << std::endl;

  rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
  std::cout << "\nRotation Vector: "<< rotation_vector << std::endl;

  float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.rotate (Eigen::AngleAxisf (theta, rotation_vector));
  std::cout << "\nTransformation matrix: " << std::endl << transform_2.matrix() << std::endl;
  pcl::transformPointCloud (*output_cloud, *output_cloud2, transform_2);

  //Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();


  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float betha = M_PI; // The angle of rotation in radians
  transform_1.rotate (Eigen::AngleAxisf(betha, Eigen::Vector3f::UnitY()));

  std::cout << "Executing the transformation..." << std::endl;
  pcl::transformPointCloud (*output_cloud2, *output_cloud3, transform_1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

  Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
  float ghama = M_PI/2; // The angle of rotation in radians
  transform_3.rotate (Eigen::AngleAxisf(ghama, Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud (*output_cloud3, *cloud_filtered, transform_3);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1000, 0);
  pass.setFilterLimitsNegative(true);
  pass.filter(*output_cloud4);

  std::cout << yellow <<"\nDensify proccess --> [OK]" << reset << std::endl;
  std::cout << "Saving dense 3d mapping file with prefix --> MAP3D_dense.pcd" << std::endl;
  std::cout << "Dense points:" << output_cloud4->points.size() << std::endl;

  std::string prefix = output_dir;
  prefix += "/3D_Mapping/";
  std::string prefix1 = prefix;
  prefix1 += "MAP3D_dense.pcd";

  std::string prefix2 = prefix;
  prefix2 += "MAP3D_dense.ply";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *output_cloud4);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *output_cloud4);

  return true;

  std::cout << "\n------------------------------------------" << std::endl; 
}

bool Utilities::uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const double scale){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              UNIFORM SCALING                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  PCL_INFO("Scaling pointcloud to real measurements...\n");
  if(cloud->points.size() <= 0){
    PCL_ERROR("Input point cloud has no data!\n");
    return false;
  }

  Eigen::MatrixXf scale_matrix(4,4);

                                            //Uniform scaling: vx = vy = vz = s --> Common scale factor
  scale_matrix << scale,  0,    0,    0,    //       |vx  0   0   0|
                   0,   scale,  0,    0,    //  Sv = |0   vy  0   0| => Scale matrix
                   0,     0,  scale,  0,    //       |0   0   vz  0|
                   0,     0,    0,    1;    //       |0   0   0   1|
                                            //https://en.wikipedia.org/wiki/Scaling_(geometry)

  std::cout << "Here is the matrix scale factor:\n" << scale_matrix << std::endl;

  PCL_INFO("Executing the transformation...");
  pcl::transformPointCloud(*cloud, *cloud_scaled, scale_matrix);  

  std::string prefix1 = output_dir;
  std::string output_pcd_files = "3D_Mapping";
  prefix1 += "/";
  prefix1 += output_pcd_files;
  prefix1 += "/";
  prefix1 += "MAP3D_scaled.pcd";

  std::string prefix2 = output_dir;
  prefix2 += "/";
  prefix2 += output_pcd_files;
  prefix2 += "/";
  prefix2 += "MAP3D_scaled.ply";

  PCL_INFO("\nSaved in: %s %s %s",prefix1.c_str(),"and",prefix2.c_str());
  std::cout << std::endl;

  pcl::io::savePCDFileBinary(prefix1.c_str(), *cloud_scaled);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *cloud_scaled); 

  if(cloud_scaled->points.size()<=0){
    PCL_ERROR("Could not scaled point cloud.");
    return false;
  }

  return true;
}

bool Utilities::alignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_align){
                           
  PCL_INFO("Aligning point cloud...");
  if(cloud->points.size() <= 0){
    PCL_ERROR("Input point cloud has no data!");
    return false;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transform1 (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transform2 (new pcl::PointCloud<pcl::PointXYZRGB>());

  // Find the planar coefficients for floor plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1);
  seg.setInputCloud(cloud);
  seg.segment(*floor_inliers, *coefficients);
  PCL_INFO("Floor Plane Model coefficients: %f %f %f",coefficients->values[0],coefficients->values[1],
                                                      coefficients->values[2],coefficients->values[3]);
  
  Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;
  Eigen::Matrix<float, 4, 1> centroid_original,centroid_new,centroid_relative;
  size_t success =  pcl::compute3DCentroid(*cloud,centroid_original);

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  std::cout << floor_plane_normal_vector << std::endl;

  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = 1.0;

  std::cout << xy_plane_normal_vector << std::endl;

  rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
  std::cout << "Rotation Vector: "<< rotation_vector << std::endl;

  float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));

  //-------------------------------
  //TRANSFORM 1: X-Y Plane Z-depth
  //-------------------------------
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  transform_1.rotate(Eigen::AngleAxisf (theta, rotation_vector));
  std::cout << "Transformation matrix: " << "\n" << transform_1.matrix() << std::endl;
  pcl::transformPointCloud(*cloud, *cloud_transform1, transform_1);

  //-------------------------------
  //TRANSFORM 2: Rotation in X (90°)
  //-------------------------------
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  float betha = M_PI/2; // The angle of rotation in radians
  transform_2.rotate(Eigen::AngleAxisf(betha, Eigen::Vector3f::UnitX()));

  std::cout << "Executing the transformation..." << std::endl;
  pcl::transformPointCloud (*cloud_transform1, *cloud_transform2, transform_2);
  success =  pcl::compute3DCentroid(*cloud_transform2,centroid_new);

  //-------------------------------
  //TRANSFORM 3: Rotation in X (90°)
  //-------------------------------
  centroid_relative[0] = centroid_new[0] - centroid_original[0];
  centroid_relative[1] = centroid_new[1] - centroid_original[1];
  centroid_relative[2] = centroid_new[2] - centroid_original[2];
  
  Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
  transform_3.translation() << centroid_relative[0], centroid_relative[1], centroid_relative[2];
  pcl::transformPointCloud (*cloud_transform2, *cloud_align, transform_3);

  if(cloud_align->points.size()<=0){
    PCL_ERROR("Could not align this cloud.");
    return false;
  }

  PCL_INFO("Cloud aligned!");
  return true;
}

void Utilities::create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh){

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads (8);
  // ne.setInputCloud (cloud_smoothed);
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setKSearch (10); //20
  //ne.setRadiusSearch (0.5); // no funciona
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  for(std::size_t i = 0; i < cloud_normals->size (); ++i){
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);//x

  pcl::Poisson<pcl::PointNormal> poisson;

  poisson.setDepth(7);//9
  poisson.setInputCloud (cloud_smoothed_normals);
  poisson.setPointWeight(4);//4
  //poisson.setDegree(5);
  poisson.setSamplesPerNode(1.5);//1.5
  poisson.setScale(1.1);//1.1
  poisson.setIsoDivide(8);//8
  poisson.setConfidence(1);
  poisson.setOutputPolygons(true);
  poisson.setManifold(0);
  //poisson.setOutputPolygons(0);
  poisson.setSolverDivide(8);//8
  pcl::PolygonMesh mesh2;
  poisson.reconstruct(mesh2);

  pcl::surface::SimplificationRemoveUnusedVertices rem;
  rem.simplify(mesh2,mesh);

  std::string out = output_dir;
  out += "/3D_Mapping/mesh.ply";

  std::cout << "\nSaving mesh:" << out << std::endl;

  pcl::io::savePolygonFilePLY(out.c_str(),mesh);
  
}

void Utilities::vizualizeMesh(vtkSmartPointer<vtkPolyData>& vtkCloud){

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));
//  viewer->setBackgroundColor(0, 0, 0);
//  viewer->addPolygonMesh(mesh,"meshes",0);
//  viewer->addCoordinateSystem(1.0);
//  viewer->initCameraParameters();
//  viewer->resetCamera();

//  std::cout << "\nPress [q] to continue." << std::endl;
//  while(!viewer->wasStopped()){
//      viewer->spin();
//  }

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(vtkCloud);
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(vertexFilter->GetOutput());

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  actor1->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor1->GetProperty()->SetPointSize(1);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(0.0, 0.0, 0.0);
  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
  renderer->ResetCamera();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

  renderWindow->SetSize(800, 600);
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene
  renderer->AddActor(actor1);
  renderer->ResetCamera();
  renderer->SetViewPoint(0,0,0);


  // Render and interact
  renderWindow->Render();
  renderWindow->SetWindowName("VTK VISUALIZER");



  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);



  PCL_INFO("\nPress [q] to close visualizer");
  renderWindowInteractor->Start();


/*
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( inputFilename.c_str() );

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(0.0,0.0,0.0); // Sea green

  renderWindow->Render();
  renderWindowInteractor->Start();
  */
}

void Utilities::vtkVisualizer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,pcl::PointXYZ& pt1,pcl::PointXYZ& pt2){

  if(cloud->points.size()<=0){
    PCL_ERROR("Input point cloud has no data!");
    return std::exit(-1);
  }

  vtkSmartPointer<vtkPolyData> cloudVTK = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

  for(int n=0;n<cloud->points.size();n++){
    pcl::PointXYZRGB pt = cloud->points.at(n);
    pts->InsertNextPoint(pt.x,pt.y,pt.z);
  }

  cloudVTK->SetPoints(pts);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(cloudVTK);
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(vertexFilter->GetOutput());

  // Create two points, P0 and P1
  double p0[3] = {pt1.x, pt1.y, pt1.z};
  double p1[3] = {pt2.x, pt2.y, pt2.z};

  vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
  lineSource->SetPoint1(p0);
  lineSource->SetPoint2(p1);
  lineSource->Update();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  actor1->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor1->GetProperty()->SetPointSize(1);

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(lineSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);
  actor2->GetProperty()->SetColor(0.0, 1.0, 0.0);
  actor2->GetProperty()->SetPointSize(1);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(0.0, 0.0, 0.0);
  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
  renderer->ResetCamera();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

  renderWindow->SetSize(800, 600);
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene

  renderer->AddActor(axes);
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);
  renderer->ResetCamera();
  renderer->SetViewPoint(0,0,0);


  // Render and interact
  renderWindow->Render();
  renderWindow->SetWindowName("VTK VISUALIZER");



  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);



  PCL_INFO("\nPress [q] to close visualizer");
  renderWindowInteractor->Start();

}

bool Utilities::is_number(const std::string& s){

  std::string::const_iterator it = s.begin();
      while (it != s.end() && std::isdigit(*it)) ++it;
      return !s.empty() && it == s.end();
  /*
    return !s.empty() && std::find_if(s.begin(),
        s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
        */
}







