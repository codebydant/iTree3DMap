# 3D Mapping of individual tree with OpenMVG-PMVS2

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this project is to do a 3D reconstruction of a  individual tree using open Multiple View Geometry (openMVG) and get dendrometry estimation (trunk diamater DBH, crop height, total height, volume crop). 

----------------------
## Example

<img src="./example/tree.jpg" align="center" height="500" width="640"><br>
<img src="./example/system.png" align="center" height="500" width="640"><br>

* openMVG --> incremental structure from motion.
* pmvs2 --> densify cloud process
* PCL --> segmentation cloud process

## Build 

To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Datasets

Images dataset in Google Drive:
* Tree1: https://drive.google.com/drive/folders/15e5q8XZuJUcQLk_ynZeljlkJmXFAAqVG?usp=sharing
* Tree2: https://drive.google.com/drive/folders/1SHkres6Ex0UFzMuqG9V7CBDIEz4nIDf5?usp=sharing
* Tree3: https://drive.google.com/drive/folders/19hX1J3fSw8WvX7-ma4me7_hjh9lAjsk-?usp=sharing
* Tree4: https://drive.google.com/drive/folders/1cN7NyDK1VAGgMcARtP4hV0MdVsHLNZc_?usp=sharing
* Tree5: https://drive.google.com/drive/folders/1ifLVliqBYHj6_6wCBpN2zcHa5HV4F6Rq?usp=sharing
* Tree6: https://drive.google.com/drive/folders/1cHlqOBxYP0mTnO-Y9pluAx98Ey32ENKt?usp=sharing

### PCD files

### Prerequisite

- OpenCV 3.4.1: https://github.com/opencv
- PCL 1.8.1: https://github.com/PointCloudLibrary/pcl
- OpenMVG:https://github.com/openMVG/openMVG
- CMVS-PMVS: https://github.com/pmoulon/CMVS-PMVS
- Qt creator 5.5.1: https://www.qt.io/download

### Other Prerequisite

- Boost
- Eigen3

### How to make

This is depends of openMVG, PCL, OpenCV, Qt and CMVS-PMVS, please install dependencies before.
* Download the src code and Unpack .zip

/src:
  	
	iTree3DMap - include
		   - libraries
		   - src
		   - CMakeLists.txt
		   - main.cpp
		   		   
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




