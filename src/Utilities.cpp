#include "include/Utilities.h"

std::string output_dir;
const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");

// This function displays the help
void Utilities::help(){
  const std::string green("\033[0;32m");
  const std::string reset("\033[0m");
  std::cout << green << "===============================================\n"
               "The program create a 3D mapping of individual tree from a images sequence using openMVG and PMVS2."
            << std::endl << "Enter:\n" << "<project name> and press \"Enter\"." << "\n"
            << "<path image sequence> and press \"Enter\"." << "\n"
            << "<output path> and press \"Enter\"." << std::endl
            << "<focal length> and press \"Enter\"." << std::endl
            << "==============================================="<< std::endl
            << "once the sfm finish, enter the image pattern path and press \"Enter\"." << std::endl
            << "<reference measure> and press \"Enter\"." << reset << std::endl;
}

void Utilities::run_openMVG(){

  //COMMAND LINE INPUT
  std::string input_dir;
  std::string focal_length;
  std::string project_name;
  double n=-1;
  const std::string red("\033[0;31m");
  const std::string blue("\033[0;34m");
  const std::string yellow("\033[0;33m");
  const std::string reset("\033[0m");
  std::string answer;
  auto start = std::chrono::high_resolution_clock::now();

  bool choise = false;

  while(output_dir.size()<=0 or input_dir.size()<=0 or focal_length.size() <=0){

    if(project_name.size()<=0){

      std::cout << blue << "\nEnter the project name:" << reset << std::endl;
      std::cout << "------------------------------------------" << std::endl;
      std::getline(std::cin, project_name);

    }

    if(not choise){

      std::cout << yellow << "project name = " << project_name << " are you sure? (yes/no)\n" << reset << std::endl;
      std::getline(std::cin, answer);
      if(answer == "yes"){
        std::cout << yellow << "Using project name as --> " << project_name << reset << std::endl;
        choise = true;
      }else if(answer == "no"){
        project_name.clear();
        choise = false;
        continue;
      }else{
        std::cout << red << answer << " is not a valid answer." << reset << std::endl;
        choise = false;
        project_name.clear();
        continue;
      }
    }

    if(input_dir.size()<=0){

         std::cout << blue << "\nEnter the images directorie path:" << reset << std::endl;
         std::cout << "------------------------------------------" << std::endl;
         std::getline(std::cin, input_dir);

         boost::filesystem::path dirPath(input_dir);

         if(not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)){
             std::cout << red << "Error. cannot open directory: " << input_dir << reset <<std::endl;
             input_dir.clear();
             continue;
         }

         bool images_files = true;

         for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(dirPath)){
               std::string extension = x.path().extension().string();
               boost::algorithm::to_lower(extension);
               if(extension == ".jpg" or extension == ".png"){
                 std::cout << yellow << "Found images file." << reset << std::endl;
                 break;
               }else{
                 images_files = false;
                 break;
               }
         }
         if(not images_files){
           std::cout << red << "Unable to find images files in directory (\"" << input_dir << "\")."
                     << reset <<std::endl;
           input_dir.clear();
           continue;
         }

         if(output_dir.size()<=0){

             std::cout << blue << "\nEnter the output directorie path:" << reset << std::endl;
             std::cout << "------------------------------------------" << std::endl;
             std::getline(std::cin, output_dir);

             boost::filesystem::path dirPath2(output_dir);

             if(not boost::filesystem::exists(dirPath2) or not boost::filesystem::is_directory(dirPath2)){
                 std::cout << red <<"Error. cannot found directory: " << output_dir << reset << std::endl;
                 output_dir.clear();
                 continue;
             }
         }

     }else if(output_dir.size()<=0){

         std::cout << blue << "\nEnter the output directorie path:" << reset << std::endl;
         std::cout << "------------------------------------------" << std::endl;
         std::getline(std::cin, output_dir);

         boost::filesystem::path dirPath2(output_dir);

         if(not boost::filesystem::exists(dirPath2) or not boost::filesystem::is_directory(dirPath2)){
             std::cout << red << "Cannot found directory: " << output_dir << reset << std::endl;
             output_dir.clear();
             continue;
         }
     }

     while(std::cout << blue << "\nEnter the focal length:\n" << reset <<
           "------------------------------------------" << std::endl
           && ! (std::cin >> n)){
             std::cin.clear();
             std::getline(std::cin, focal_length);
             std::cout << yellow << "I am sorry, but '" << focal_length << "' is not a number" << reset
                       << std::endl;
     }

     if(n<=0){
         std::cout << red << "Error: insert a valid focal length" << reset << std::endl;
         n = -1;
         focal_length.clear();
         continue;
     }

     std::ostringstream strs;
     strs << n;
     focal_length = strs.str();
     break;
 }

 int dont_care;
 std::string folder_name = "mkdir ";
 folder_name += output_dir;
 folder_name += "/";
 folder_name += project_name;

 dont_care = std::system(folder_name.c_str());

 output_dir += "/";
 output_dir += project_name;

 std::string output_pcd_files = "3D_Mapping";
 std::string folder_name2 = "mkdir ";
 folder_name2 += output_dir;
 folder_name2 += "/";
 folder_name2 += output_pcd_files;

 dont_care = std::system(folder_name2.c_str());

 std::string command = "python ";
 std::string openMVG = "~/catkin_ws/src/iTree3DMap/openMVG/openMVG_Build/software/SfM/SfM_SequentialPipeline.py ";

 command += openMVG;
 command += input_dir;
 command += " ";
 command += output_dir;
 command += " ";
 command += focal_length;

 std::cout << blue << "\n3D Mapping with openMVG initializing..." << reset << std::endl;
 dont_care = std::system(command.c_str());

 if(dont_care > 0){
  std::cout << red << "Failed. SfM_SequentialPipeline.py not found" << reset << std::endl;
  std::exit(-1);
 }
 std::cout << "\n" << "3D Mapping --> [COMPLETE]." << std::endl;
 auto end = std::chrono::high_resolution_clock::now();
 auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
 if(difference >=60){
   std::cout << "3D Mapping Time: " << difference/60 << " minutes" << std::endl;
 }else{
   std::cout << "3D Mapping Time: " << difference << " seconds" << std::endl;
 }
}

bool Utilities::getScaleFactor(pcl::PointCloud<pcl::PointXYZ>::Ptr& Map3D, float& scale_factor){

  std::cout << "Showing 3D Mapping..." << std::endl;
  std::cout << "Converting sfm_data.bin to sfm_data.xml..." << std::endl;
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
  std::string command = "~/catkin_ws/src/iTree3DMap/openMVG/openMVG_Build/Linux-x86_64-RELEASE/openMVG_main_ConvertSfM_DataFormat -i ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.bin -o ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.xml -I -E -S";

  int dont_care = std::system(command.c_str());
  if(dont_care > 0){
   std::cout << "Failed. Could not convert sfm_data.bin to xml" << std::endl;
   std::exit(-1);
  }
  */
  cv::Mat_<float> intrinsic;
  std::vector<cv::Matx34f> cameras_poses;
  std::vector<Point3DInMap> cloud;

  bool success = Utilities::loadSFM_XML_Data(cloud,intrinsic,cameras_poses,true);
  if(not success){
    std::cout << "Could not get a scale factor." << std::endl;
    return false;
  }

  std::cout << "\nCloud xml: " << cloud.size() << " pts" << std::endl;
  std::cout << "Poses: " << cameras_poses.size() << " cameras" << std::endl;
  std::cout << "Intrinsic camera:\n" << intrinsic << std::endl;

  fromPoint3DToPCLCloud(cloud,Map3D);

  std::cout << blue << "\nGetting the scale factor." << reset << std::endl;

  float W_reference;
  std::string world_reference;
  float n=-1;

  while(n<=0){

    while(std::cout << blue << "\nEnter the world reference (mm/cm/m/) etc!\n" << reset <<
            "------------------------------------------" << std::endl
            && !(std::cin >> n)){
              //std::cin.clear();
              std::getline(std::cin, world_reference);
              std::cout << yellow << "I am sorry, but '" << world_reference << "' is not a number" << reset
                        << std::endl;
    }

    if(n<=0){
        std::cout << red << "Error: insert a valid world reference" << reset << std::endl;
        n = -1;
        world_reference.clear();
        continue;
    }
  }

  W_reference = n;
  std::cout <<yellow << "Using world reference:" << W_reference << "\n" << reset << std::endl;

  float image_pixel_reference=-1;
  std::string answer;
  std::string img_path;
  cv::Mat image_2DReference;

  while(image_pixel_reference<=0){

    std::cout << blue << "\nEnter image pattern full path:" << reset << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cin >> img_path;
    //std::getline(std::cin, img_path);

    cv::Mat img = cv::imread(img_path.c_str(),1);
    if(!img.data )                    // Check for invalid input
      {
          std::cout << red <<"Could not open or find the image" << reset << std::endl;
          continue;
      }
    cv::Mat img_copy = img.clone();
    image_2DReference = img.clone();
    cv::Mat gray;

    cv::cvtColor(img_copy,gray,CV_BGR2GRAY);
    cv::GaussianBlur(gray,gray,cv::Size(9,9),2,2);
    //cv::medianBlur(gray,gray,5);

    std::vector<cv::Vec3f> circles(2);

    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 150);
    cv::Point pattern1,pattern2;

    for(size_t i = 0; i < circles.size(); i++ ){

      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      pattern2 = pattern1;
      pattern1 = cv::Point(center.x,center.y);
      int radius = cvRound(circles[i][2]);
      // circle center
      cv::circle(img_copy, center, 1, cv::Scalar(0,255,0),10);
      // circle outline
      cv::circle(img_copy, center, radius,cv::Scalar(0,255,0),30);

    }

    float pixel_length = cv::norm(pattern1 - pattern2);

    std::cout << "\nNum of circles detect:" << circles.size() << std::endl;
    std::cout << "Circle 1 center:" << pattern1 << std::endl;
    std::cout << "Circle 2 center:" << pattern2 << std::endl;
    std::cout << "Image:"<< img_path << " Num rows:" << gray.rows << " Num cols:" << gray.cols << std::endl;
    std::cout << "Pixels length:" << pixel_length << std::endl;
    cv::line(img_copy,pattern1,pattern2,cv::Scalar(0,255,0),30);

    cv::namedWindow("pattern",CV_WINDOW_NORMAL);
    cv::resizeWindow("pattern",640,480);
    cv::moveWindow("pattern",0,0);
    cv::imshow("pattern",img_copy);
    cv::waitKey(700);

    bool choise = false;

    if(not choise){

      std::cout << yellow << "\nIs this lenght OK? (yes/no):" << reset << std::endl;
      std::cout << "------------------------------------------" << std::endl;
      std::cin.ignore();
      std::getline(std::cin, answer);
      if(answer == "yes"){
        std::cout << yellow << "Using pixels reference length: " << pixel_length << reset << std::endl;
        choise = true;
        image_pixel_reference = pixel_length;
        cv::destroyAllWindows();
        break;
      }else if(answer == "no"){
        image_pixel_reference = -1;
        answer.clear();
        choise = true;
        continue;
      }else{
        while(not choise){
          std::cout << red << answer << " is not a valid answer." << reset << std::endl;
          choise = false;
          answer.clear();
          std::cout << yellow << "Is this lenght OK? (yes/no):" << reset << std::endl;
          std::cout << "------------------------------------------" << std::endl;
          //std::cin.ignore();
          std::getline(std::cin, answer);
          if(answer == "yes"){
            std::cout << yellow << "Using pixels reference length: " << pixel_length << reset << std::endl;
            choise = true;
            image_pixel_reference = pixel_length;
            cv::destroyAllWindows();
            break;
          }else if(answer == "no"){
            answer.clear();
            choise = true;
             image_pixel_reference = -1;
            break;
          }else{
            continue;
          }
        }
      }
    }
    cv::destroyAllWindows();
  }

  std::string image_pattern_path;
  float x_, y_,s,orientation;
  std::vector<cv::Point2f> image_points;

  while(image_pattern_path.size()<=0){

    std::cout << blue << "\nEnter the feature file associated to pattern:" << reset << std::endl;
    std::cout << blue << "Must be: file.feat in "<< output_dir << "/matches" << reset << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::getline(std::cin, image_pattern_path);

    std::ifstream file(image_pattern_path.c_str());
    if(!file.is_open()){
      std::cout << red << "Error: Could not find "<< image_pattern_path << reset << std::endl;
      image_pattern_path.clear();
      continue;
    }

    if(image_points.size()<=0){
      while(file >> x_ >> y_ >> s >> orientation){
        image_points.push_back(cv::Point2f(x_,y_));
      }
    }
  }

  std::cout << yellow << "\nFeature selected:" << image_pattern_path << std::endl;
  std::cout << "Image points: " << image_points.size() << " points" << reset << std::endl;

  /*
  cv::Mat rvecLeft;
  cv::Rodrigues(cameras_poses[0].get_minor<3,3>(0,0),rvecLeft);
  cv::Mat tvecLeft(cameras_poses[0].get_minor<3,1>(0,3));

  std::vector<cv::Point2f> projected_points(cloudPCL->size());
  cv::projectPoints(cloud,rvecLeft,tvecLeft,intrinsic,cv::Mat(),projected_points);

  const float MIN_REPROJECTION_ERROR = 8.0; //Maximum 10-pixel allowed re-projection error

  //check if point reprojection error is small enough
  const float error = cv::norm(projected_points[0]  - image_points[0]);
  std::cout << "projected p1:" << projected_points[0] << " p1:" << image_points[0] << std::endl;
  std::cout << "error" << error << std::endl;
  */

  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  if(difference >=60){
    std::cout << "Scale factor Time: " << difference/60 << " minutes" << std::endl;
  }else{
    std::cout << "Scale factor Time: " << difference << " seconds" << std::endl;
  }

}

bool Utilities::loadSFM_XML_Data(std::vector<Point3DInMap>& pts3d,
                                 cv::Mat_<float>& intrinsic,
                                 std::vector<cv::Matx34f>& cameras_poses,bool show){

  const std::string red("\033[0;31m");
  const std::string blue("\033[0;34m");
  const std::string yellow("\033[0;33m");
  const std::string reset("\033[0m");

  // Empty document
  tinyxml2::XMLDocument xml_doc;

  /*READING FILE*/
  std::cout << blue <<"Reading sfm_data.xml file. Please wait." << reset << std::endl;
  std::string sfm_data;
  sfm_data += output_dir;
  sfm_data += "/reconstruction_sequential/sfm_data.xml";

  // Load xml document
  tinyxml2::XMLError eResult = xml_doc.LoadFile("sfm_data.xml");
  if(eResult != tinyxml2::XML_SUCCESS){
    std::cout << red << "Error: Could not find a xml file." << reset << std::endl;
    sfm_data.clear();
    return false;
  }

  std::cout << yellow << "sfm_data.xml:" << std::endl;
  std::cout << "<intrinsics>" << std::endl;
  std::cout << "<extrinsics>" << std::endl;
  std::cout << "<structure>" << reset << std::endl;

  std::cout << "\nFounding root tag <cereal> ..." << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  // Root xml document
  tinyxml2::XMLNode * root = xml_doc.FirstChildElement("cereal");
  if(root!=nullptr){

    std::cout << yellow << "Found: tag <cereal>" << reset << std::endl;
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

    float f,cx,cy;
    data->FirstChildElement("focal_length")->QueryFloatText(&f);

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
      child->QueryFloatText(&cy);
    }

    // Matrix K
    intrinsic = (cv::Mat_<float>(3, 3) << f, 0, cx,
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

      float r11,r12,r13;
      float r21,r22,r23;
      float r31,r32,r33;

      // Iterate over <rotation> tag
      for(tinyxml2::XMLElement* child = rotation->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        // Iterate over <value> rotation tag
        for(tinyxml2::XMLElement* child2 = child->FirstChildElement();child2 != NULL;
            child2 = child2->NextSiblingElement()){

          r11 = r12;
          r12 = r13;
          r13 = r21;
          r21 = r22;
          r22 = r23;
          r23 = r31;
          r31 = r32;
          r32 = r33;
          child2->QueryFloatText(&r33);
          /*
          r11 = r21;
          r21 = r31;
          r31 = r12;

          r12 = r22;
          r22 = r32;
          r32 = r13;

          r13 = r23;
          r23 = r33;
          child2->QueryFloatText(&r33);
          */

        }
      }

      float t1,t2,t3;

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
        child->QueryFloatText(&t3);
      }

      // Camera pose
      cv::Matx34f pose = cv::Matx34f(r11, r12, r13, t1,
                                     r21, r22, r23, t2,
                                     r31, r32, r33, t3);

      // Saving camera pose
      cameras_poses.push_back(pose);
    }

    std::cout  << "\nFounding <structure> tag ..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    /*POINTCLOUD DATA*/
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

      float x,y,z;

      // Iterate over <X> tag
      for(tinyxml2::XMLElement* child = X_root->FirstChildElement();child != NULL;
          child = child->NextSiblingElement()){

        x=y;
        y=z;
        eResult = child->QueryFloatText(&z);   //z=coordinate;
      }

      // Filling pt feature
      pt3d->pt = cv::Point3f(x,y,z);

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

        float x,y;

        // Iterate over <x> tag
        for(tinyxml2::XMLElement* child = feature_root->FirstChildElement();child != NULL;
            child = child->NextSiblingElement()){

          x = y;
          eResult = child->QueryFloatText(&y);   //y=coordinate;
        }

         // 2D Point - <bservations> tag
         cv::Point2f pt2d(x,y);
         std::map<const int,cv::Point2f> feature;
         feature[img_id_feat]=pt2d;

         pt3d->feat_ref[img_id]=feature;

      }

      // Saving pt3d
      pts3d.push_back(*pt3d);
    }

    /*
    std::cout << "pt3d ini: " << pts3d[0].pt << std::endl;
    std::cout << "pt3d ini num observations:" << pts3d[0].feat_ref.size() << std::endl;
    for(std::pair<const int,std::map<const int,cv::Point2f>>&  ft : pts3d[0].feat_ref){

        for(std::pair<const int,cv::Point2f>&  pt2d : ft.second){
          std::cout << "img:" << ft.first <<" ft_id:" << pt2d.first << " pt2d:" << pt2d.second << std::endl;
        }
    }

    std::cout << "pt3d end: " << pts3d[pts3d.size()-1].pt << std::endl;
    std::cout << "pt3d end num observations:" << pts3d[pts3d.size()-1].feat_ref.size() << std::endl;
    for(std::pair<const int,std::map<const int,cv::Point2f>>&  ft : pts3d[pts3d.size()-1].feat_ref){

        for(std::pair<const int,cv::Point2f>&  pt2d : ft.second){
          std::cout << "img:" << ft.first <<" ft_id:" << pt2d.first << " pt2d:" << pt2d.second << std::endl;
        }
    }
    */

  }else{
    std::cout << red << "Error: root of xml could not found. Must be: <cereal>" << reset << std::endl;
    return false;
  }

  if(show){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZ>());
    fromPoint3DToPCLCloud(pts3d,cloudPCL);
/*
    std::string prefix1 = output_dir;
    std::string output_pcd_files = "3D_Mapping";
    prefix1 += "/";
    prefix1 += output_pcd_files;
    prefix1 += "/";
    prefix1 += "MAP3D.pcd";

    pcl::io::savePCDFileBinary(prefix1.c_str(), *cloudPCL);
*/

  }

  return true;
}

void Utilities::createPMVS_Files(){

  const std::string red("\033[0;31m");
  const std::string blue("\033[0;34m");
  const std::string reset("\033[0m");

  std::cout << "\n------------------------------------------" << std::endl;
  std::cout << blue <<"Creating files for PMVS2..." << reset << std::endl;

  std::string command2 = "~/catkin_ws/src/iTree3DMap/openMVG/openMVG_Build/Linux-x86_64-RELEASE/openMVG_main_openMVG2PMVS -i ";
  command2 += output_dir;
  command2 += "/reconstruction_sequential/sfm_data.bin -o ";
  command2 += output_dir;

  int dont_care = std::system(command2.c_str());

  if(dont_care > 0){
   std::cout << red << "Failed. openMVG_main_openMVG2PMVS not found" << reset << std::endl;
   std::exit(-1);
  }

}

void Utilities::densifyWithPMVS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud){

  const std::string blue("\033[0;34m");
  const std::string reset("\033[0m");

  std::cout << "\n------------------------------------------" << std::endl;
  std::cout << blue << "Densify cloud process initializing..." << reset << std::endl;

  std::string command3 = "~/catkin_ws/src/iTree3DMap/programs/pmvs2 ";
  command3 += output_dir;
  command3 += "/PMVS/ ";
  command3 += "pmvs_options.txt";

  int dont_care = std::system("chmod 771 ~/catkin_ws/src/iTree3DMap/programs/pmvs2");
  dont_care = std::system(command3.c_str());

  if(dont_care > 0){
   std::cout << "Failed. ./pmvs2 no found" << std::endl;
   std::exit(-1);
  }

  std::string cloudPLY = output_dir;
  cloudPLY += "/PMVS/models/pmvs_options.txt.ply";

  pcl::PLYReader inputPlyCloud;
  inputPlyCloud.read(cloudPLY,*output_cloud);

  std::cout << "Densify proccess --> [OK]" << std::endl;
  std::cout << "Saving dense 3d mapping file with prefix --> MAP3D_dense.pcd" << std::endl;

  std::string prefix1 = output_dir;
  std::string output_pcd_files = "3D_Mapping";
  prefix1 += "/";
  prefix1 += output_pcd_files;
  prefix1 += "/";
  prefix1 += "MAP3D_dense.pcd";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *output_cloud);

  std::cout << "\n------------------------------------------" << std::endl;
  std::cout << "Showing 3D mapping" << std::endl;

  pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("MAP3D",true);

  viewer.setPosition(0,0);
  viewer.setSize(640,480);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.addCoordinateSystem ();
  viewer.setCameraPosition(0,0,1,0,0,0);
  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0,0.1,1;

  viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
  viewer.addPointCloud(output_cloud,"tree_cloud");
  viewer.resetCamera();

  std::cout << "Press [q] to continue --> SEGMENTATION PROCESS!" << std::endl;

  while(!viewer.wasStopped ()) {
         viewer.spin();
  }
}

void Utilities::uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const float scale,
                               bool show){

  std::cout << "Scaling pointcloud to real measurements..." << std::endl;

  Eigen::MatrixXf scale_matrix(4,4);
                                            //Uniform scaling: vx = vy = vz = s --> Common scale factor
  scale_matrix << scale,  0,    0,    0,    //       |vx  0   0   0|
                   0,   scale,  0,    0,    //  Sv = |0   vy  0   0| => Scale matrix
                   0,     0,  scale,  0,    //       |0   0   vz  0|
                   0,     0,    0,    1;    //       |0   0   0   1|
                                            //https://en.wikipedia.org/wiki/Scaling_(geometry)

  std::cout << "Here is the matrix scale factor:\n" << scale_matrix << std::endl;

  std::cout << "Executing the transformation..." << std::endl;
  pcl::transformPointCloud(*cloud, *cloud_scaled, scale_matrix);

  if(show){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*cloud_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scale_xyz (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_scaled,*cloud_scale_xyz);

    // Visualization
    pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("cloud scale",true);
    viewer.setPosition(0,0);
    viewer.setSize(640,480);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setCameraPosition(0,0,1,0,0,0);

    /*Coordinate system*/
    viewer.addCoordinateSystem ();

    pcl::PointXYZ p1, p2, p3;
    p1.getArray3fMap() << 1, 0, 0;
    p2.getArray3fMap() << 0, 1, 0;
    p3.getArray3fMap() << 0,0.1,1;

    viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
    viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
    viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");

    // White --> original cloud
    viewer.addPointCloud(cloud_xyz, "original_cloud");

    // Red --> cloud scale
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_scale_color(cloud_scale_xyz, 230, 20, 20);
    viewer.addPointCloud(cloud_scale_xyz,cloud_scale_color, "transformed_cloud");

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    viewer.resetCamera();

    std::cout << "Press [q] to continue process segmentation!" << std::endl;

    while(!viewer.wasStopped ()) {
           viewer.spin();
    }

    viewer.close();
  }
}

void Utilities::fromPoint3DToPCLCloud(const std::vector<Point3DInMap> &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL){

  for(size_t i = 0; i < input_cloud.size(); ++i){
      cv::Point3f pt3d = input_cloud[i].pt;
      pcl::PointXYZ pclp;
      pclp.x  = pt3d.x;
      pclp.y  = pt3d.y;
      pclp.z  = pt3d.z;
      cloudPCL->push_back(pclp);
   }
   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->header.frame_id ="map";
   cloudPCL->is_dense = false;
}




