# 3D Mapping of individual tree with OpenMVG-PMVS2

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this project is to do a 3D reconstruction of a  individual tree using open Multiple View Geometry (openMVG) and get dendrometry estimation (trunk diamater DBH, crop height, total height, volume crop). 

----------------------
## Example

<img src="./example/tree.jpg" align="center" height="500" width="640"><br>

* openMVG --> incremental structure from motion.
* pmvs2 --> densify cloud process
* PCL --> segmentation cloud process

## Build 

To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Datasets

Images dataset in Google Drive:
* Tree1: https://drive.google.com/open?id=1AsdQgc2l6l8o7U2K4WO3KJO4slQSHrQK
* Tree2: https://drive.google.com/open?id=1rUcZ2k-Pie_U_kv5qRIJg-vVpyem0e74
*
* Tree4: https://drive.google.com/open?id=1AZPB0J58qxXQ4_ad-XHbkAinNvna5faW
* Trunk: https://drive.google.com/open?id=1R9ubdsSIOiJsaHf98XhkoQySATVV-ymP

### Prerequisite

- OpenCV 3.4.1: https://github.com/opencv
- PCL 1.8.1: https://github.com/PointCloudLibrary/pcl
- OpenMVG:https://github.com/openMVG/openMVG
- PMVS: https://github.com/pmoulon/CMVS-PMVS

### Other Prerequisite

- Boost
- Eigen3

### How to make

This is a ros package project and depends of openMVG and Pmvs2.
* Download the src code and Unpack .zip

catkin_ws/src:
  	
	iTree3DMap - src 
		   - include
		   - libraries
		   - build
		   
* replace the "SfM_SequentialPipeline.py" in openMVG/openMVG_Build/software/SfM/ for "SfM_SequentialPipeline.py" of repository   
* If pmvs2 binary file is not working, download and compile cmvs-pmvs library and replace the pmvs2 bin to /programs folder of the package 

Compile with cmake:

    cmake ../
    make
 	 
### Test

	cd /build/bin
	./itree_3dmap		

*Note:*
If OpenCV are not install. just compiled. please set the path to the current build directory in CMakeList.txt file.
(equal to pcl 1.8.1) <--
If openMVG compile without error and is not working, add this to the cmake command line for compilation:

	-DTARGET_ARCHITECTURE=generic

and try again.




