# 3D Dendrometric feature estimation of an individual tree with OpenMVG-PMVS2
<p align="justify">
*Note:* This project is a photogrammetric system for dendrometric feature estimation of individual trees. The purpose of this project is to do a 3D reconstruction of an individual tree using Open Multiple View Geometry (openMVG) and get dendrometry estimation (diameter at breast height (DBH), tree crown height, total tree height, crown volume, morphic factor and percentage canopy missing) of a stem tree. For that, were used OpenMVG and PMVS for 3D Mapping, a circular pattern for scale factor estimation, PCL Library and DBScan for segmentation and Euclidean distance for dendrometric feature estimation.
</p>

----------------------
## Example

<img src="./example/tree.jpg" align="center" height="500" width="640"><br>
<img src="./example/system.png" align="center" height="500" width="640"><br>

## Build cmake
<p align="justify">
To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake or:
					
	$ sudo apt-get install cmake && cmake-gui && openssl && git -> This install cmake-3.5.1
	
</p>

### Dataset
Images dataset in Google Drive:
* Tree1: https://drive.google.com/drive/folders/15e5q8XZuJUcQLk_ynZeljlkJmXFAAqVG?usp=sharing
* Tree2: https://drive.google.com/drive/folders/1SHkres6Ex0UFzMuqG9V7CBDIEz4nIDf5?usp=sharing
* Tree3: https://drive.google.com/drive/folders/19hX1J3fSw8WvX7-ma4me7_hjh9lAjsk-?usp=sharing
* Tree4: https://drive.google.com/drive/folders/1cN7NyDK1VAGgMcARtP4hV0MdVsHLNZc_?usp=sharing
* Tree5: https://drive.google.com/drive/folders/1ifLVliqBYHj6_6wCBpN2zcHa5HV4F6Rq?usp=sharing
* Tree6: https://drive.google.com/drive/folders/1cHlqOBxYP0mTnO-Y9pluAx98Ey32ENKt?usp=sharing

### PCD files
* https://drive.google.com/drive/folders/1sW4oqcaKPsupEaSFkoGAgdHjv6ydwUFF?usp=sharing

### Ground Truth Data
* https://docs.google.com/spreadsheets/d/1cs-I-1BxetnAPiWe4e07TfKXyPMTo7x_rN72FC26kHU/edit?usp=sharing

### Dependencies
- Eigen3: http://eigen.tuxfamily.org/index.php?title=Main_Page
- Glut (OpenGL): 

		$ sudo apt-get install libglu1-mesa-dev
		$ sudo apt-get install freeglut3-dev
		$ sudo apt-get install mesa-common-dev
		
- Blas and Lapack: 

		$ sudo apt-get install liblapack-dev && libblas-dev 
		$ sudo apt-get install libgoogle-glog-dev
- Boost:

		$ sudo apt-get install libboost-all-dev
		
- Cereal:
 
 		$ sudo apt-get install gcc-multilib g++-multilib
		$ Download the source code, compile and install: https://github.com/USCiLab/cereal
		
- Flann: 

		$ sudo apt-get install liblz4-dev
		$ sudo apt-get install libhdf5-dev
		$ sudo apt-get install libgtest-dev  
		(for gtest)
		
		cd /usr/src/gtest
		sudo cmake CMakeLists.txt
		sudo make
		sudo cp *.a /usr/lib
		create a gtest folder in usr/local/lib
		sudo ln -s /usr/lib/libgtest.a /usr/local/lib/gtest/libgtest.a
		sudo ln -s /usr/lib/libgtest_main.a /usr/local/lib/gtest/libgtest_main.a
		
		Edit CMakeLists.txt on flann/src/cpp/CMakeLists.txt and replace:
		add_library(flann_cpp SHARED "") -> add_library(flann_cpp SHARED "main.cpp")
		Create a main.cpp file at the samen directory
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")		
		
		
		$ https://github.com/mariusmuja/flann -> compile and install

- TinyXML2: https://github.com/leethomason/tinyxml2 -> compile and install

- GTK2.0: 

		$ sudo apt-get install libgtk2.0-dev
		
- QT creator:

		$ sudo apt-get install qtcreator

		$ sudo apt-get install git build-essential qt5-default qtscript5-dev libssl-dev qttools5-dev qttools5-dev-tools 
		  qtmultimedia5-dev libqt5svg5-dev libqt5webkit5-dev libsdl2-dev libasound2 libxmu-dev libxi-dev freeglut3-dev 
		  libasound2-dev libjack-jackd2-dev libxrandr-dev libqt5xmlpatterns5-dev libqt5xmlpatterns5
		  libqt5xmlpatterns5-private-dev
		  
- VTK: 

		$ sudo apt-get install xorg 
		$ sudo apt-get install qhull-bin
		$ sudo apt-get install libqhull-dev
		https://github.com/Kitware/VTK

- OpenMVG:https://github.com/openMVG/openMVG
- CMVS-PMVS: https://github.com/pmoulon/CMVS-PMVS

### Prerequisite (just compiled)
<p align="justify">
For this project it is necessary to have compiled the following libraries (it is not necessary to have installed):

- OpenCV 3.4.1: https://github.com/opencv/opencv/tree/3.4.1
- PCL 1.8.1 or 1.9.1: https://github.com/PointCloudLibrary/pcl/tree/pcl-1.8.1

*Note:* Just set the path to the build directory of each library in the principal CMakeList.txt
 </p>
 
### How to make
<p align="justify">
This project depends of openMVG, PCL, OpenCV, Qt and CMVS-PMVS, please compile the dependencies before.
	
* Download the src code and Unpack .zip

/src:
  	
	iTree3DMap - include
		   - libraries
		   - src
		   - CMakeLists.txt
		   - main.cpp
		   
* Once OpenMVG was compiled, just set the path build directory in the CMakeList.txt		   		   
* Edit the "SfM_SequentialPipeline.py" file in the project build directorie (ItreeMapper_Build) with your own OpenMVG path directorie.
* If pmvs2 binary file is not working, download and compile the cmvs-pmvs library and replace the pmvs2 bin to /libraries folder of the package 

Within build folder, compile with cmake:

    cmake ../
    make
 </p> 
 
### Test

	cd /build/bin
	./itree_3dmap		

*Note:*
<p align="justify">
Just for OpenMVG compilation: If OpenMVG compiled without error and is not working; compile OpenMVG with this line:

	cmake -DTARGET_ARCHITECTURE=generic ../src/

</p>

Extra packages:
* [Canopy Missing](https://github.com/danielTobon43/canopyMissing) C++ program for percentage canopy missing
* [Pointcloud to Mesh channel](https://github.com/danielTobon43/pointcloudToMesh) C++ program to convert pcd to mesh
* [Control image selector](https://github.com/danielTobon43/gui-Control-points) GUI interface for images
* [Align point cloud](https://github.com/danielTobon43/align_pointcloud) C++ program for alignment pointcloud to axis
* [DBScan](https://github.com/danielTobon43/DBScan-PCL-Optimized) C++ program for clustering 3D points in a space
