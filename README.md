# 3D Mapping of individual tree with OpenMVG-PMVS2

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this project is to do a 3D reconstruction of a  individual tree using open Multiple View Geometry (openMVG) and get dendrometry estimation (trunk diamater DBH, crop height, total height, volume crop). 

----------------------
## Example

<img src="./example/tree.jpg" align="center" height="500" width="640"><br>

* openMVG --> incremental structure from motion.
* pmvs2 --> densify cloud process
* PCL --> segmentation cloud process
* ROS --> package project

## Build 

To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Datasets

Images dataset: https://drive.google.com/drive/folders/1-JVw5yQG1W8lTxsDRouK0nL871xBpY02?usp=sharing

### Prerequisite

- OpenCV 3.4.1: https://github.com/opencv
- PCL 1.8.1: https://github.com/PointCloudLibrary/pcl
- ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu
- OpenMVG:https://github.com/openMVG/openMVG
- PMVS: https://github.com/pmoulon/CMVS-PMVS

### Other Prerequisite

- Boost
- Eigen3

### How to make

This is a ros package project and depends of openMVG and Pmvs2.
* Download the src code and Unpack .zip
* Copy the package to ROS catkin_ws/src
* Compile openMVG as: openMVG_Build
* Copy the openMVG_Build folder to same level directory of package eg:

catkin_ws/src:
  	
	iTree3DMap - src 
		   - include
		   - programs
		   - openMVG
		   
* replace the "SfM_SequentialPipeline.py" in openMVG/openMVG_Build/software/SfM/ for "SfM_SequentialPipeline.py" of repository   
* If pmvs2 binary file is not working, download and compile cmvs-pmvs library and replace the pmvs2 bin to /programs folder of the package 

Compile with catkin:

    cd ~/catkin_ws
	catkin_make
 	 
### Test

	cd ~/catkin_ws
	source devel/setup.bash
	roscore  --> optional
	rosrun itree_3dmap itree_3dmap		

*Note:*
If OpenCV are not install. just compiled. please set the path to the current build directory in CMakeList.txt file.
(equal to pcl 1.8.1) <--
If openMVG compile without error and is not working, add this to the cmake command line for compilation:

	-DTARGET_ARCHITECTURE=generic

and try again.




