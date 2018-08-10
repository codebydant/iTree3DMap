#include "include/Utilities.h"

//std::string output_dir="/home/daniel/catkin_ws";
std::string output_dir;
std::string input_dir;
std::vector<std::string> images_filenames;
const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");

// This function displays the help
void Utilities::help(){
  const std::string green("\033[0;32m");
  const std::string reset("\033[0m");
  std::cout << green << "===============================================\n"
               "The program estimate dendrometric features of individual tree from a 3D real scale mapping using "
               "a images sequence with:openMVG and PMVS2."
            << std::endl << "Enter:\n" << "<project name> and press \"Enter\"." << "\n"
            << "<path image sequence> and press \"Enter\"." << "\n"
            << "<output path> and press \"Enter\"." << std::endl
            << "<focal length> and press \"Enter\"." << std::endl
            << "==============================================="<< std::endl
            << "Once the 3D Mapping finish, enter the image pattern path and press \"Enter\"." << std::endl
            << "<reference measure> and press \"Enter\"." << reset << std::endl;
}

bool Utilities::run_openMVG(){

  //COMMAND LINE INPUT  
  std::string focal_length;
  std::string project_name; 
  std::string answer;
  auto start = std::chrono::high_resolution_clock::now();


  /*PROJECT NAME*/
  while(true){

    bool nameOk = false;
    if(project_name.size()<=0){

      std::cout << blue << "\nEnter the project name: (don't use space)" << reset << std::endl;
      std::cout << "------------------------------------------" << "\n" << "->" << std::flush;
      std::getline(std::cin, project_name);
      if(project_name.empty()){
        std::cout << red << "Nothing entered." << reset << std::endl;
        project_name.clear();
        continue;
      }
    }

    if(project_name.size()>0){

      std::cout << yellow << "\nproject name = " << project_name << " are you sure? (yes/no)" << reset << std::endl;
      std::cout << "->" << std::flush;
      std::getline(std::cin, answer);
      if(answer.empty()){
        std::cout << red << "Nothing entered." << reset << std::endl;
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
        std::cout << red << answer << " is not a valid answer." << reset << std::endl;
        nameOk = false;
        answer.clear();
        continue;
      }
    }

    if(nameOk){
      std::cout << "Using project name as:" << project_name << std::endl;
      break;
    }
  }

  /*IMAGES SEQUENCE PATH*/
  while(true){

    bool imagesOk = false;
    if(input_dir.size()<=0){

      std::cout << blue << "\nEnter the images directorie path:" << reset << std::endl;
      std::cout << "------------------------------------------" << "\n" << "->" << std::flush;
      std::getline(std::cin, input_dir);
      if(input_dir.empty()){
        std::cout << red << "Nothing entered." << reset << std::endl;
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
        std::cout << red << "Nothing entered." << reset << std::endl;
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
  /*
  double n=-1;

  while(n<=0){

    std::cout << blue << "\nEnter the focal length:\n" << reset
              << "------------------------------------------" << "\n" << "->" << std::flush;
    std::cin >> n;

    if(std::cin.fail()){
      std::getline(std::cin, focal_length);
      std::cout << yellow << "I am sorry, but '" << focal_length << "' is not a number" << reset
                << std::endl;
      focal_length.clear();
      n = -1;
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      continue;
    }else if(n<=0){
      std::cout << red << "Error: insert a valid focal_length" << reset << std::endl;
      n = -1;
      focal_length.clear();
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      continue;
    }else{
      std::ostringstream strs;
      strs << n;
      focal_length = strs.str();
      std::cout << "Using focal length:" << focal_length << std::endl;
      break;
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
/*
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
    return false;
  }
  std::cout << "\n" << "3D Mapping --> [COMPLETE]." << std::endl;
  */
  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "3D Mapping Time: " << difference << " seconds" << std::endl;

  return true;

}

bool Utilities::getScaleFactor(pcl::PointCloud<pcl::PointXYZ>::Ptr& Map3D, float& scale_factor){

  std::cout << blue << "\nConverting sfm_data.bin to sfm_data.xml..." << reset << std::endl;
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
  std::string command = "~/catkin_ws/src/iTree3DMap/openMVG/openMVG_Build/Linux-x86_64-RELEASE/openMVG_main_ConvertSfM_DataFormat -i ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.bin -o ";
  command += output_dir;
  command += "/reconstruction_sequential/sfm_data.xml -V -I -E -S";

  int dont_care = std::system(command.c_str());
  if(dont_care > 0){
   std::cout << "Failed. Could not convert sfm_data.bin to xml" << std::endl;
   return false;
  }

  std::cout << yellow << "Created xml file in:" << reset << output_dir
            << "/reconstruction_sequential/sfm_data.xml" <<  std::endl;
  */

  cv::Mat_<float> intrinsic;
  std::vector<cv::Matx34f> cameras_poses;
  std::vector<Point3DInMap> cloud;

  std::cout << blue << "\nGetting data from sfm_data.xml..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  bool success = Utilities::loadSFM_XML_Data(cloud,intrinsic,cameras_poses);
  if(not success){
    std::cout << "Could not get a scale factor." << std::endl;
    return false;
  }

  std::cout << "\nImages: " << images_filenames.size() << std::endl;
  std::cout << "Cloud xml: " << cloud.size() << " pts" << std::endl;
  std::cout << "Camera Poses: " << cameras_poses.size() << " cameras" << std::endl;
  std::cout << "Intrinsic camera:\n" << intrinsic << std::endl;

  fromPoint3DToPCLCloud(cloud,Map3D);

  std::cout << blue << "\nGetting scale factor..." << reset << std::endl;
  std::cout << "------------------------------------------";

  float W_reference;
  std::string world_reference;
  float n=-1;

  while(n<=0){

    while(std::cout << blue << "\nEnter the world reference (mm/cm/m/) etc!\n" << reset <<
            "------------------------------------------" << std::endl
            && !(std::cin >> n)){             
              std::getline(std::cin, world_reference);
              std::cout << yellow << "I am sorry, but '" << world_reference << "' is not a number" << reset
                        << std::endl;
              std::cin.clear();
              std::cin.ignore(1000, '\n');
    }

    if(n<=0){
        std::cout << red << "Error: insert a valid world reference" << reset << std::endl;
        n = -1;
        world_reference.clear();
        continue;
    }
  }

  W_reference = n;
  std::cout <<yellow << "Using world reference:" << reset << W_reference << "\n" << std::endl;
  std::cout << "Choose a image pattern reference" << std::endl;
  std::cout << "Select --> 1 or 13 or 5 etc! and press Enter." << std::endl;

  float image_pixel_reference=-1;

  while(image_pixel_reference<=0){

    for(int i=0;i<images_filenames.size();i++){

      std::string images_path = input_dir;
      images_path += "/";
      images_path += images_filenames.at(i);

      std::cout << "Image #:" << i << std::endl;

      cv::Mat img = cv::imread(images_path.c_str(),1);
      if(!img.data ){
        std::cout << red <<"Could not open or find the image" << reset << std::endl;
        continue;
      }
      cv::namedWindow("images",CV_WINDOW_NORMAL);
      cv::resizeWindow("images",cv::Size(640,480));
      cv::imshow("images",img);
      cv::waitKey(0);
    }

    int numImg;
    cv::destroyAllWindows();

    std::cout << "Which one?\n" << "------------------------------------------" << std::endl;
    std::cin >> numImg;

    std::cout << yellow << "Image selected:" << reset << numImg << std::endl;

    std::string images_path = input_dir;
    images_path += "/";
    images_path += images_filenames.at(numImg);

    cv::Mat img = cv::imread(images_path.c_str(),1);
    if(!img.data ){
      std::cout << red <<"Could not open or find the image" << reset << std::endl;
      return false;
    }
    cv::Mat img_copy = img.clone();
    cv::Mat gray;

    cv::cvtColor(img_copy,gray,CV_BGR2GRAY);
    std::vector<cv::Vec3f> circles(2);

    std::cout << "Filtering...Canny!" << std::endl;
    cv::Canny( gray, gray, 100, 100*2, 3 );
    std::cout << "Filtering...GaussianBlur!" << std::endl;
    cv::GaussianBlur(gray,gray,cv::Size(7,7),2,2);

    std::cout << "Detecting circles in image..." << std::endl;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,1, gray.rows/16, 200, 100, 0, 150);

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
    std::cout << "Image #:"<< numImg << " Num rows:" << gray.rows << " Num cols:" << gray.cols << std::endl;
    std::cout << "Pixels length:" << pixel_length << std::endl;
    cv::line(img_copy,pattern1,pattern2,cv::Scalar(0,255,0),30);

    cv::namedWindow("pattern",CV_WINDOW_NORMAL);
    cv::resizeWindow("pattern",640,480);
    cv::moveWindow("pattern",0,0);
    cv::imshow("pattern",img_copy);
    cv::waitKey(0);
    cv::destroyAllWindows();

    break;

  }



  /*

  float image_pixel_reference=-1;
  std::string answer;
  std::string img_path;
  cv::Mat image_2DReference;
  cv::Point2f img_p1,img_p2;

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

    std::vector<cv::Vec3f> circles(2);

    std::cout << "Filtering...Canny!" << std::endl;
    cv::Canny( gray, gray, 100, 100*2, 3 );
    std::cout << "Filtering...GaussianBlur!" << std::endl;
    cv::GaussianBlur(gray,gray,cv::Size(7,7),2,2);

    std::cout << "Detecting circles in image..." << std::endl;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,1, gray.rows/16, 200, 100, 0, 150);

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
      //std::cin.ignore();
      std::getline(std::cin, answer);
      if(answer == "yes"){
        std::cout << yellow << "Using pixels reference length: " << pixel_length << reset << std::endl;
        choise = true;
        image_pixel_reference = pixel_length;
        img_p1 = pattern1;
        img_p2 = pattern2;
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
            img_p1 = pattern1;
            img_p2 = pattern2;
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
  */
/*
  std::string image_pattern_path;
  image_pattern_path += img_path;

  std::cout << "antes del pop:" << image_pattern_path << std::endl;

  std::string backG = images_filenames.at(0);

  std::cout << "string reference:" << backG << std::endl;

  for(int i=0;i<backG.size();i++){
    int j=i;
    image_pattern_path.pop_back();
  }

  std::cout << "pop back:" << image_pattern_path << std::endl;
  std::string img_feat_filename;

  for(int i=0;i<images_filenames.size();i++){

    std::string comp = image_pattern_path;
    comp += images_filenames.at(i);

    if(comp == img_path){
      std::cout << "Found:" << comp << std::endl;
      img_feat_filename = images_filenames.at(i);
      break;
    }
  }

  img_feat_filename.pop_back();
  img_feat_filename.pop_back();
  img_feat_filename.pop_back();

  std::string feature_path;
  feature_path += output_dir;
  feature_path += "/matches/";
  feature_path += img_feat_filename;
  feature_path += "feat";

  float x_, y_,s,orientation;
  std::vector<cv::Point2f> image_points;

  std::ifstream file(feature_path.c_str());
  if(!file.is_open()){
    std::cout << red << "Error: Could not find "<< feature_path << reset << std::endl;
    feature_path.clear();
    return false;
  }

  while(file >> x_ >> y_ >> s >> orientation){
      image_points.push_back(cv::Point2f(x_,y_));
  }

  std::cout << yellow << "\nFeature selected:" << feature_path << std::endl;
  std::cout << "Image points: " << image_points.size() << " points" << reset << std::endl;
*/
  /*
  int id_pose = 21;
  std::cout << "camera pose:\n" << cameras_poses[id_pose] << std::endl;

  cv::Matx34f pose1 =cameras_poses[id_pose];
  cv::Mat Rc = cv::Mat(pose1.get_minor<3,3>(0,0));
  cv::Mat C = cv::Mat(pose1.get_minor<3,1>(0,3));

  cv::Mat R = Rc.t();
  cv::Mat t = -R*C;

  pose1 = cv::Matx34f(R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),t.at<float>(0),
                     R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),t.at<float>(1),
                     R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),t.at<float>(2));

  cv::Mat rvec;
  cv::Rodrigues(pose1.get_minor<3,3>(0,0),rvec);
  cv::Mat tvec(pose1.get_minor<3,1>(0,3));

  std::vector<cv::Point2f> projected_points;
  std::vector<cv::Point3f> points3d;

  std::cout << "cloud size:" << cloud.size() << std::endl;

  for(int i=0;i<cloud.size();i++){
    points3d.push_back(cloud.at(i).pt);
  }

  std::cout << "pts3d size:" << points3d.size() << std::endl;
  std::cout << "intrinsic:\n" << intrinsic << std::endl;

  cv::projectPoints(points3d,rvec,tvec,intrinsic,cv::Mat(),projected_points);
  std::cout << "Projected points:" << projected_points.size() << std::endl;

  cv::Point2f pr_1,pr_2;
  double error;
     pcl::PointXYZ ptt1,pt2;

  std::map<double,std::pair<cv::Point2f,cv::Point3f>> p1_map;

  //check if point reprojection error is small enough
  for(int i=0;i<points3d.size();i++){
    pr_1 = projected_points[i];
    error = cv::norm(pr_1 - img_p1);

      p1_map[error] = std::make_pair(pr_1,points3d.at(i));

  }

  std::map<double,std::pair<cv::Point2f,cv::Point3f>> p2_map;

  //check if point reprojection error is small enough
  for(int i=0;i<points3d.size();i++){
    pr_2 = projected_points[i];
    error = cv::norm(pr_2 - img_p2);

      p2_map[error] = std::make_pair(pr_2,points3d.at(i));

  }


  for(auto ptp2 : p1_map){
       std::cout << "projected p1:" << ptp2.second.first << " p1:" << img_p1 << std::endl;
       std::cout << "error: " << ptp2.first << std::endl;
       std::cout << "P1 Point3D:" << ptp2.second.second << std::endl;
       ptt1 = pcl::PointXYZ(ptp2.second.second.x,ptp2.second.second.y,ptp2.second.second.z);
       break;

  }

  for(auto ptp2 : p2_map){
       std::cout << "projected p2:" << ptp2.second.first << " p2:" << img_p2 << std::endl;
       std::cout << "error: " << ptp2.first << std::endl;
       std::cout << "P2 Point3D:" << ptp2.second.second << std::endl;
       pt2 = pcl::PointXYZ(ptp2.second.second.x,ptp2.second.second.y,ptp2.second.second.z);
       break;
  }




 // ptt1 = pcl::PointXYZ(0.11601659388820981,0.33759194974945006,1.5855298677299712);



  std::cout << "puntos p2 elegidos:" << p2_map.size() << std::endl;


  float Ref_PCL = pcl::geometry::distance(ptt1,pt2);
  scale_factor = W_reference/Ref_PCL;
  std::cout << "Model reference:" << Ref_PCL << std::endl;
    std::cout << "scale_factor:" << scale_factor << std::endl;

  //scale = Ref_W/Ref_PCL




  // pcl::PointXYZ ptt1,pt2;
  */
/*
//  for(std::map<float,std::pair<pcl::PointXYZ,pcl::PointXYZ>>::iterator it=bestPts.begin(); it!=bestPts.end(); ++it){
//scale_factor = it->first;

  //}



 // scale_factor = W_reference/0.788;

*/
  /*

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

  viewer.addLine(ptt1,pt2,0,255,0 ,"lenght",0);

  viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");

  viewer.addPointCloud(Map3D,"tree_cloud");
  viewer.resetCamera();

  std::cout << "Press [q] to continue --> SEGMENTATION PROCESS!" << std::endl;

  while(!viewer.wasStopped ()) {
         viewer.spin();
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  if(difference >=60){
    std::cout << "Scale factor Time: " << difference/60 << " minutes" << std::endl;
  }else{
    std::cout << "Scale factor Time: " << difference << " seconds" << std::endl;
  }
*/
}

bool Utilities::loadSFM_XML_Data(std::vector<Point3DInMap>& pts3d,
                                 cv::Mat_<float>& intrinsic,
                                 std::vector<cv::Matx34f>& cameras_poses){

  // Empty document
  tinyxml2::XMLDocument xml_doc;

  /*READING FILE*/
  std::cout << blue <<"Reading sfm_data.xml file. Please wait." << reset << std::endl;
  std::string sfm_data;
  sfm_data += output_dir;
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
  std::cout << "<extrinsics>" << std::endl;
  std::cout << "<structure>" << reset << std::endl;

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

          r11 = r21;
          r21 = r31;
          r31 = r12;

          r12 = r22;
          r22 = r32;
          r32 = r13;

          r13 = r23;
          r23 = r33;
          child2->QueryFloatText(&r33);
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

  }else{
    std::cout << red << "Error: root of xml could not found. Must be: <cereal>" << reset << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZ>());
  fromPoint3DToPCLCloud(pts3d,cloudPCL);

  std::string prefix1 = output_dir;
  std::string output_pcd_files = "3D_Mapping";
  prefix1 += "/";
  prefix1 += output_pcd_files;
  prefix1 += "/";
  prefix1 += "MAP3D.pcd";

  std::string prefix2 = output_dir;
  prefix2 += "/";
  prefix2 += output_pcd_files;
  prefix2 += "/";
  prefix2 += "MAP3D.ply";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *cloudPCL);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *cloudPCL);

  return true;
}

void Utilities::createPMVS_Files(){

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

  std::string prefix2 = output_dir;
  prefix2 += "/";
  prefix2 += output_pcd_files;
  prefix2 += "/";
  prefix2 += "MAP3D_dense.ply";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *output_cloud);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *output_cloud);

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

    pcl::io::savePCDFileBinary(prefix1.c_str(), *cloud_scale_xyz);
    pcl::io::savePLYFileBinary(prefix2.c_str(), *cloud_scale_xyz);

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





