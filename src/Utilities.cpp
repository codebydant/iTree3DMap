#include "include/Utilities.h"

std::string output_dir;
std::string input_dir;
std::vector<std::string> images_filenames;
const std::string red("\033[0;31m");
const std::string blue("\033[0;34m");
const std::string yellow("\033[0;33m");
const std::string reset("\033[0m");
const std::string green("\033[0;32m");

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkPolyData.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkSimplePointsReader.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>

#include <pcl/common/pca.h>

// This function displays the help
void Utilities::help(){
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

  /*INPUT PATH*/
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

  /*FOCAL LENGTH*/
/*
  double n=-1;
  while(n<=0){

    std::cout << blue << "\nEnter the focal length:\n" << reset
              << "------------------------------------------" << "\n" << "->" << std::flush;
    std::cin >> n;

    if(std::cin.fail()){
      std::getline(std::cin, focal_length);
      std::cout << red << "I am sorry, but '" << focal_length << "' is not a number" << reset
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

bool Utilities::getScaleFactor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Map3D,
                               double& scale_factor,std::string& output_path){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              SCALE FACTOR                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << blue << "\nConverting sfm_data.bin to sfm_data.xml..." << reset << std::endl;
  std::cout << "------------------------------------------" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  output_path = output_dir;

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

//  cv::Mat_<double> intrinsic;
//  std::vector<cv::Matx34d> cameras_poses;
//  //std::vector<Point3DInMap> cloud;

//  std::cout << blue << "\nGetting data from sfm_data.xml..." << reset << std::endl;
//  std::cout << "------------------------------------------" << std::endl;

//  bool success = Utilities::loadSFM_XML_Data(Map3D,intrinsic,cameras_poses);
//  if(not success){
//    std::cout << "Could not get a scale factor." << std::endl;
//    return false;
//  }

//  std::cout << "\nImages: " << images_filenames.size() << std::endl;
//  std::cout << "Cloud xml: " << Map3D->points.size() << " pts" << std::endl;
//  std::cout << "Camera Poses: " << cameras_poses.size() << " cameras" << std::endl;
//  std::cout << "Intrinsic camera:\n" << intrinsic << std::endl;

//  //fromPoint3DToPCLCloud(cloud,Map3D);

//  std::cout << blue << "\nGetting scale factor..." << reset << std::endl;
//  std::cout << "------------------------------------------";

//  double W_reference;
//  std::string world_reference;
//  double n=-1;

//  while(n<=0){

//    while(std::cout << blue << "\nEnter the world reference (mm/cm/m/) etc!\n" << reset <<
//            "------------------------------------------\n" << "->" << std::flush
//            && !(std::cin >> n)){
//              std::getline(std::cin, world_reference);
//              std::cout << yellow << "I am sorry, but '" << world_reference << "' is not a number" << reset
//                        << std::endl;
//              std::cin.clear();
//              std::cin.ignore(1000, '\n');
//    }

//    if(n<=0){
//        std::cout << red << "Error: insert a valid world reference" << reset << std::endl;
//        n = -1;
//        world_reference.clear();
//        continue;
//    }
//  }

//  W_reference = n;
//  std::cout <<yellow << "Using world reference:" << reset << W_reference << "\n" << std::endl;
//  std::cout << "Choose a image pattern reference" << std::endl;

//  double image_pixel_reference=-1;
//  int numImg = -1;
//  cv::Point2d img_p1,img_p2;

//  while(image_pixel_reference<=0){

//    std::string answer;
//    bool bestImage = false;
//     std::string imageChoose;

//    int inter = std::round(images_filenames.size()/4);
//    int cont =0;

//    for(int i=0;i<images_filenames.size();i++){

//      std::string images_path = input_dir;
//      images_path += "/";
//      images_path += images_filenames.at(i);

//      std::cout << "Image #:" << i << std::endl;

//      cv::Mat img = cv::imread(images_path.c_str(),1);
//      if(!img.data ){
//        std::cout << red <<"Could not open or find the image" << reset << std::endl;
//        continue;
//      }
//      cv::namedWindow("images",CV_WINDOW_NORMAL);
//      cv::resizeWindow("images",cv::Size(640,480));
//      cv::imshow("images",img);
//      cv::waitKey(0);
//      cont +=1;

//      if(cont == inter or cont == 2*inter or cont == 3*inter){
//        cv::destroyAllWindows();
//        while(true){

//          std::cout << yellow << "\nWhich one?" << "\n" << reset << "------------------------------------------\n"
//                    << "->" << std::flush;

//          std::cin >> numImg;

//          if(std::cin.fail()){
//            std::getline(std::cin, imageChoose);
//            std::cout << red << "I am sorry, but '" << numImg << "' is not a number" << reset
//                      << std::endl;
//            imageChoose.clear();
//            numImg = -1;
//            std::cin.clear();
//            std::cin.ignore(1000, '\n');
//            bestImage = false;
//            continue;
//          }else if(numImg<0){
//            std::cout << yellow << "Continue!" << reset << std::endl;
//            numImg = -1;
//            imageChoose.clear();
//            std::cin.clear();
//            bestImage = false;
//            break;
//          }else{
//            bestImage = true;
//            break;
//          }
//        }
//        if(bestImage){
//          break;
//        }
//      }
//    }

//    cv::destroyAllWindows();

//    while(numImg<0){

//      std::cout << yellow << "\nWhich one?" << "\n" << reset << "------------------------------------------\n"
//                << "->" << std::flush;

//      std::cin >> numImg;

//      if(std::cin.fail()){
//        std::getline(std::cin, imageChoose);
//        std::cout << red << "I am sorry, but '" << numImg << "' is not a number" << reset
//                  << std::endl;
//        imageChoose.clear();
//        numImg = -1;
//        std::cin.clear();
//        std::cin.ignore(1000, '\n');
//        bestImage = false;
//        continue;
//      }else if(numImg<0){
//        std::cout << red << "Skip." << reset << std::endl;
//        numImg = -1;
//        imageChoose.clear();
//        std::cin.clear();
//        std::cin.ignore(1000, '\n');
//        bestImage = false;
//        continue;
//      }else{
//        std::cout << "Using image #:" << numImg << std::endl;
//        bestImage = true;
//        break;
//      }
//    }

//    while(numImg > -1){

//      bool foundBestImage = false;
//      std::cout << yellow << "Image selected:" << reset << numImg << " filename:" << images_filenames.at(numImg)
//                << std::endl;

//      std::string images_path = input_dir;
//      images_path += "/";
//      images_path += images_filenames.at(numImg);

//      cv::Mat img = cv::imread(images_path.c_str(),1);
//      if(!img.data ){
//        std::cout << red <<"Could not open or find the image" << reset << std::endl;
//        return false;
//      }

//      cv::Mat img_copy = img.clone();
//      cv::Mat gray;

//      cv::cvtColor(img_copy,gray,CV_BGR2GRAY);
//      std::vector<cv::Vec3f> circles(2);

//      std::cout << "\nFiltering...Canny!" << std::endl;
//      cv::Canny(gray, gray, 100, 100*2, 3 );
//      std::cout << "Filtering...GaussianBlur!" << std::endl;
//      cv::GaussianBlur(gray,gray,cv::Size(7,7),2,2);

//      std::cout << "Detecting circles in image..." << std::endl;
//      cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,1, gray.rows/16, 200, 100, 0, 150);

//      cv::Point2d pattern1,pattern2;

//      for(size_t i = 0; i < circles.size(); i++ ){

//        cv::Point2d center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//        pattern2 = pattern1;
//        pattern1 = cv::Point2d(center.x,center.y);
//        int radius = cvRound(circles[i][2]);
//        // circle center
//        cv::circle(img_copy, center, 1, cv::Scalar(0,255,0),10);
//        // circle outline
//        cv::circle(img_copy, center, radius,cv::Scalar(0,255,0),30);
//      }

//      double pixel_length = cv::norm(pattern1 - pattern2);

//      std::cout << "\nNum of circles detect:" << circles.size() << std::endl;
//      std::cout << "Circle 1 center:" << pattern1 << std::endl;
//      std::cout << "Circle 2 center:" << pattern2 << std::endl;
//      std::cout << "Image #:"<< numImg << " Num rows:" << gray.rows << " Num cols:" << gray.cols << std::endl;
//      std::cout << "Pixels length:" << pixel_length << std::endl;
//      cv::line(img_copy,pattern1,pattern2,cv::Scalar(0,255,0),30);

//      Display* d = XOpenDisplay(NULL);
//      Screen*  s = DefaultScreenOfDisplay(d);

//      int x = s->width;

//      cv::namedWindow("pattern",CV_WINDOW_NORMAL);
//      cv::resizeWindow("pattern",640,480);
//      cv::moveWindow("pattern",std::round(x/2),0);
//      cv::imshow("pattern",img_copy);
//      cv::waitKey(0);
//      cv::destroyAllWindows();

//      while(true){

//        std::cout << yellow << "\nIs this lenght OK? (yes/no):" << reset << std::endl;
//        std::cout << "------------------------------------------\n" << "->" << std::flush;
//        std::cin.clear();
//        std::cin.ignore(100, '\n');
//        std::getline(std::cin, answer);
//        if(answer.empty()){
//          std::cout << red << "Nothing entered." << reset << std::endl;
//          answer.clear();
//          std::cin.clear();
//          continue;
//        }

//        if(answer == "yes"){
//          img_p1 = pattern1;
//          img_p2 = pattern2;
//          foundBestImage = true;
//          break;
//        }else if(answer == "no"){
//          image_pixel_reference = -1;
//          answer.clear();
//          foundBestImage = false;
//        }else{
//          std::cout << red << "Is not a valid answer" << reset << std::endl;
//          answer.clear();
//          continue;
//        }

//        if(not foundBestImage){
//          std::cout << yellow << "\nWhich one?" << "\n" << reset << "------------------------------------------\n"
//                    << "->" << std::flush;
//          std::cin >> numImg;
//          break;
//        }
//      }

//      if(foundBestImage){
//        std::cout << yellow << "Using pixels reference length: " << pixel_length << reset << std::endl;
//        image_pixel_reference = pixel_length;
//        bestImage = true;
//        break;
//      }

//      if(not foundBestImage){
//        continue;
//      }
//    }

//    if(not bestImage){
//      std::cout << red << "No image selected." << reset << std::endl;
//      continue;
//    }

//    if(bestImage){
//      break;
//    }
//  }

//  std::cout << "id pose:" << numImg << std::endl;
//  std::cout << "Filename:" << images_filenames.at(numImg) << std::endl;
//  std::cout << "camera pose:\n" << cameras_poses[numImg] << std::endl;

//  cv::Matx34d pose =cameras_poses[numImg];
//  cv::Mat Rc = cv::Mat(pose.get_minor<3,3>(0,0));
//  cv::Mat C = cv::Mat(pose.get_minor<3,1>(0,3));

//  cv::Mat R = Rc.t();
//  cv::Mat t = -R*C;

//  pose = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0),
//                     R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1),
//                     R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2));

//  cv::Mat rvec;
//  cv::Rodrigues(pose.get_minor<3,3>(0,0),rvec);
//  cv::Mat tvec(pose.get_minor<3,1>(0,3));

//  std::vector<cv::Point2d> projected_points;
//  std::vector<cv::Point3d> points3d;

//  for(int i=0;i<Map3D->points.size();i++){
//    points3d.push_back(cv::Point3d(Map3D->points[i].x,Map3D->points[i].y,Map3D->points[i].z));
//  }

//  cv::projectPoints(points3d,rvec,tvec,intrinsic,cv::Mat(),projected_points);
//  double error;
//  pcl::PointXYZ ptt1,pt2;

//  std::map<double,std::pair<cv::Point2d,cv::Point3d>> p1_map;

//  //check if point reprojection error is small enough
//  for(int i=0;i<points3d.size();i++){
//    error = cv::norm(projected_points[i] - img_p1);
//    p1_map[error] = std::make_pair(projected_points[i],points3d.at(i));
//  }

//  std::map<double,std::pair<cv::Point2d,cv::Point3d>> p2_map;

//  //check if point reprojection error is small enough
//  for(int i=0;i<points3d.size();i++){
//    error = cv::norm(projected_points[i] - img_p2);
//    p2_map[error] = std::make_pair(projected_points[i],points3d.at(i));
//  }

//  std::vector<cv::Point2d> p1_map_pts;
//  std::vector<cv::Point2d> p2_map_pts;

//  for(auto ptp2 : p1_map){

//    p1_map_pts.push_back(ptp2.second.first);
//    std::cout << "Projected point1:" << ptp2.second.first << " original point1:" << img_p1 << std::endl;
//    std::cout << "Error: " << ptp2.first << std::endl;
//    std::cout << "Point3D:" << ptp2.second.second << std::endl;
//    ptt1 = pcl::PointXYZ(ptp2.second.second.x,ptp2.second.second.y,ptp2.second.second.z);
//    break;
//  }

//  for(auto ptp2 : p2_map){
//    p2_map_pts.push_back(ptp2.second.first);

//    std::cout << "\nProjected point2:" << ptp2.second.first << " original point2:" << img_p2 << std::endl;
//    std::cout << "Error: " << ptp2.first << std::endl;
//    std::cout << "Point3D:" << ptp2.second.second << std::endl;
//    pt2 = pcl::PointXYZ(ptp2.second.second.x,ptp2.second.second.y,ptp2.second.second.z);
//    break;
//  }

//  vtkSmartPointer<vtkPolyData> cloudVTK = vtkSmartPointer<vtkPolyData>::New();
//  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

//  for(int n=0;n<points3d.size();n++){
//    cv::Point3d p = points3d.at(n);
//    pts->InsertNextPoint(p.x,p.y,p.z);
//  }
//  cloudVTK->SetPoints(pts);

//  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
//  vertexFilter->SetInputData(cloudVTK);
//  vertexFilter->Update();

//  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//  polydata->ShallowCopy(vertexFilter->GetOutput());

//  // Create two points, P0 and P1
//  double p0[3] = {ptt1.x, ptt1.y, ptt1.z};
//  double p1[3] = {pt2.x, pt2.y, pt2.z};

//  vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
//  lineSource->SetPoint1(p0);
//  lineSource->SetPoint2(p1);
//  lineSource->Update();

//  // Create a mapper and actor
//  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
//  mapper1->SetInputData(polydata);

//  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
//  actor1->SetMapper(mapper1);
//  actor1->GetProperty()->SetColor(1.0, 1.0, 1.0);
//  actor1->GetProperty()->SetPointSize(1);

//  // Create a mapper and actor
//  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
//  mapper2->SetInputConnection(lineSource->GetOutputPort());

//  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
//  actor2->SetMapper(mapper2);
//  actor2->GetProperty()->SetColor(0.0, 1.0, 0.0);
//  actor2->GetProperty()->SetPointSize(1);

//  // Create a renderer, render window, and interactor
//  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//  renderer->SetBackground(0.0, 0.0, 0.0);
//  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
//  renderer->ResetCamera();
//  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

//  renderWindow->SetSize(800, 600);
//  renderWindow->AddRenderer(renderer);

//  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//  renderWindowInteractor->SetRenderWindow(renderWindow);

// // vtkSmartPointer<vtkCallbackCommand> keypressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
// // keypressCallback->SetCallback(KeypressCallbackFunction);
////  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent,keypressCallback,);

//  // Add the actor to the scene
//  renderer->AddActor(actor1);
//  renderer->AddActor(actor2);

//  // Render and interact
//  renderWindow->Render();
//  renderWindow->SetWindowName("VISUALIZER");
//  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER"));

//  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
//    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview
//  renderWindowInteractor->SetInteractorStyle(style);
//  std::cout << "Press [q] to continue" << std::endl;
//  renderWindowInteractor->Start();
///*
//  vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(renderWindowInteractor);
//  // Close the window
//  iren->GetRenderWindow()->Finalize();

//  // Stop the interactor
//  iren->TerminateApp();
//  std::cout << "Closing window..." << std::endl;
//*/
//  double Ref_PCL = pcl::geometry::distance(ptt1,pt2);
//  scale_factor = W_reference/Ref_PCL;
//  std::cout << "\nModel reference:" << Ref_PCL << std::endl;
//  std::cout << "scale_factor:" << scale_factor << std::endl;

////  for(std::map<double,std::pair<pcl::PointXYZ,pcl::PointXYZ>>::iterator it=bestPts.begin(); it!=bestPts.end(); ++it){
////scale_factor = it->first;

//  //}

//  auto end = std::chrono::high_resolution_clock::now();
//  auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
//  std::cout << "Scale factor Time: " << difference << " seconds" << std::endl;
}

bool Utilities::loadSFM_XML_Data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pts3d/*std::vector<Point3DInMap>& pts3d*/,
                                 cv::Mat_<double>& intrinsic,
                                 std::vector<cv::Matx34d>& cameras_poses){

  // Empty document
  tinyxml2::XMLDocument xml_doc;  
  pcl::PolygonMesh polyMesh;

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
    std::cout << red << "Error: root of xml could not found. Must be: <cereal>" << reset << std::endl;
    return false;
  }

  std::string polyFile = output_dir;
  polyFile += "/reconstruction_sequential/colorized.ply";

  pcl::io::loadPolygonFile(polyFile.c_str(),polyMesh);
  pcl::fromPCLPointCloud2(polyMesh.cloud, *pts3d);
  //fromPoint3DToPCLCloud(pts3d,cloudPCL);

  pcl::PointCloud<pcl::PointXYZ>::Ptr invY (new pcl::PointCloud<pcl::PointXYZ>());

  for(int i=0;i<pts3d->points.size();i++){
    pcl::PointXYZ pt;
    pt.x = pts3d->points[i].x;
    pt.y = pts3d->points[i].y*(-1);
    pt.z = pts3d->points[i].z;
    invY->points.push_back(pt);
  }

  invY->width = pts3d->points.size ();
  invY->height = 1;
  invY->is_dense = true;

  for(int i=0;i<invY->points.size();i++){
    pts3d->points[i].x=invY->points[i].x;
    pts3d->points[i].y=invY->points[i].y;
    pts3d->points[i].z=invY->points[i].z;
  }



  std::string prefix = output_dir;
  prefix += "/3D_Mapping/";
  std::string prefix1 = prefix;
  prefix1 +="MAP3D.pcd";

  std::string prefix2 = prefix;
  prefix2 += "MAP3D.ply";

  pcl::io::savePCDFileBinary(prefix1.c_str(), *pts3d);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *pts3d);

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

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              DENSIFICATION                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "\n------------------------------------------" << std::endl;
  std::cout << blue << "Densify cloud process initializing..." << reset << std::endl;
  /*
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
*/
  std::string cloudPLY = output_dir;
  cloudPLY += "/PMVS/models/pmvs_options.txt.ply";

  pcl::PLYReader inputPlyCloud;
  inputPlyCloud.read(cloudPLY,*output_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr invY (new pcl::PointCloud<pcl::PointXYZ>());

  for(int i=0;i<output_cloud->points.size();i++){
    pcl::PointXYZ pt;
    pt.x = output_cloud->points[i].x;
    pt.y = output_cloud->points[i].y*(-1);
    pt.z = output_cloud->points[i].z;
    invY->points.push_back(pt);
  }

  invY->width = output_cloud->points.size ();
  invY->height = 1;
  invY->is_dense = true;

  for(int i=0;i<invY->points.size();i++){
    output_cloud->points[i].x=invY->points[i].x;
    output_cloud->points[i].y=invY->points[i].y;
    output_cloud->points[i].z=invY->points[i].z;
  }

//  std::cout << "Densify proccess --> [OK]" << std::endl;
//  std::cout << "Saving dense 3d mapping file with prefix --> MAP3D_dense.pcd" << std::endl;
//  std::cout << "Dense points:" << output_cloud->points.size() << std::endl;

//  std::string prefix = output_dir;
//  prefix += "/3D_Mapping/";
//  std::string prefix1 = prefix;
//  prefix1 += "MAP3D_dense.pcd";

//  std::string prefix2 = prefix;
//  prefix2 += "MAP3D_dense.ply";

//  pcl::io::savePCDFileBinary(prefix1.c_str(), *output_cloud);
//  pcl::io::savePLYFileBinary(prefix2.c_str(), *output_cloud);

  std::cout << "\n------------------------------------------" << std::endl; 
}

void Utilities::uniformScaling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_scaled,const double scale){

  std::cout << "\n************************************************" << std::endl;
  std::cout << "              UNIFORM SCALING                      " << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "Scaling pointcloud to real measurements..." << std::endl;
  if(cloud->points.size() <= 0){
    std::cout << red << "Input point cloud has no data!" << reset << std::endl;
    return std::exit(-1);
  }

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

  std::cout << "Saved in:" << prefix1 << " and "<< prefix2 << std::endl;

  pcl::io::savePCDFileBinary(prefix1.c_str(), *cloud_scaled);
  pcl::io::savePLYFileBinary(prefix2.c_str(), *cloud_scaled); 

}

void Utilities::fromPoint3DToPCLCloud(const std::vector<Point3DInMap> &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL){

  for(size_t i = 0; i < input_cloud.size(); ++i){
      cv::Point3d pt3d = input_cloud[i].pt;
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





