# 3D Mapping of individual tree with OpenMVG-PMVS2

This is a reference implementation of a Structure-from-Motion pipeline in OpenCV, following the work of Roy Shilkrot at SfM-Toy-Library. https://github.com/royshil/SfM-Toy-Library

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this code is to do a 3D reconstruction of a tree and get dendrometry estimation. 

----------------------
## Example

<img src="./example/tree.jpg" align="center" height="500" width="640"><br>

A simple incremental SFM pipeline for 3D reconstruction of a tree with bundle adjustment. 
* Incremental SFM
* Bundle Adjustment - Ceres solver
* Segmentation - PCL color based growing segmentation
* Densify cloud - PMVS2 

### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

## Build 

To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Datasets
Images dataset: https://drive.google.com/drive/folders/1-JVw5yQG1W8lTxsDRouK0nL871xBpY02?usp=sharing


### Prerequisite
- OpenCV 3.4.1: https://github.com/opencv
- PCL 1.8.1.99: https://github.com/PointCloudLibrary/pcl
- ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu
- OpenMVG:https://github.com/openMVG/openMVG
- PMVS: https://github.com/pmoulon/CMVS-PMVS

### How to make
This project depends of openMVG and Pmvs2.
* Download the src code and Unpack .zip
* Copy the package to ROS workspace/src
* Compile openMVG as: openMVG_Build
* Copy the openMVG_Build folder to same level directory of package eg:

catkin_ws/src:
  	
	iTree3DMap - src 
		   - include
		   - programs
		   - openMVG
		   
* replace the "SfM_SequentialPipeline.py" in openMVG/openMVG_Build/software/SfM/ for "SfM_SequentialPipeline.py" of repository   
* If pmvs2 binary file is not working, compile cmvs-pmvs and replace the pmvs2 bin to /programs folder of the package 

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




