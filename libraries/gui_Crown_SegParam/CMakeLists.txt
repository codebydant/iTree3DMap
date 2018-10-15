##############################################################################
# ROS CONFIGURATION
##############################################################################

#if(ROSBUILD)
 # include(rosbuild.cmake OPTIONAL)
  #return()
#endif()

##############################################################################
# CMAKE CONFIGURATION
##############################################################################

cmake_minimum_required(VERSION 3.10.2)
project(interfaz VERSION 1.0.0)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(QT_QMAKE_EXECUTABLE "/usr/lib/x86_64-linux-gnu/qt5/bin/qmake")

# set build type = Release mode
set(CMAKE_BUILD_TYPE Release)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_DIR /opt/opencv-3.4.1/build)
set(PCL_DIR /opt/pcl-1.8.1/build)
set(tinyXM_DIR /opt/tinyxml2/build)
set(PATH_EIGEN /opt/eigen-3.3.4/build)

#set the CMP0074 policy to old behavior (disable warnings) (CMake 3.12.0-rc1)
if(${CMAKE_VERSION} MATCHES 3.12.0)
    cmake_policy(SET CMP0074 OLD)
    if(POLICY CMP0048)
        cmake_policy(SET CMP0048 NEW)
    endif(POLICY CMP0048)
endif()

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set turn off the output rule messages of cmake
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

##############################################################################
# PACKAGES
##############################################################################
message("***********************")
message("Eigen3 pkg")
message("***********************")

find_package(Eigen3 3.3.4 REQUIRED NO_MODULE PATHS ${PATH_EIGEN})
if(Eigen3_FOUND)
   message(STATUS "Eigen status:")
   message(STATUS "    version: ${Eigen3_VERSION}")
else()
   message(FATAL_ERROR "    ERROR: Eigen ${Eigen3_VERSION} not found")
endif()

message("***********************")

find_package(tinyxml2 REQUIRED PATHS ${tinyXM_DIR})
if(tinyxml2_FOUND)
   message(STATUS "tinyxml2 status: found")
else()
   message(FATAL_ERROR "    ERROR: tinyxml2 not found")
endif()

message("***********************")
message("point cloud library pkg")
message("***********************")

find_package(PCL 1.8 PATHS ${PCL_DIR})
if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    pcl directorie: ${PCL_INCLUDE_DIRS}")
else()
    message(WARNING "   PCL 1.8 not found, attempting 1.7...")
    find_package(PCL 1.7 REQUIRED)
    if(PCL_FOUND)
        message(STATUS "PCL status:")
        message(STATUS "    version: ${PCL_VERSION}")
        message(STATUS "    pcl directorie: ${PCL_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "    ERROR: PCL minimum required version 1.7. Not found")
    endif()
endif()

#if(${PCL_VERBOSITY_LEVEL} STREQUAL Error)
  set(VERBOSITY_LEVEL_ERROR 1)
#endif()

message("***********************")
message("OpenCV pkg")
message("***********************")

find_package(OpenCV 3.4.1 REQUIRED PATHS ${OPENCV_DIR})
if(OpenCV_FOUND)
   message(STATUS "OpenCV status:")
   message(STATUS "    version: ${OpenCV_VERSION}")
else()
   message(FATAL_ERROR "    ERROR: OpenCV ${OpenCV_VERSION} not found")
endif()


find_package(Qt5Widgets REQUIRED)
if(Qt5Widgets_FOUND)
    message(STATUS "Qt5 library status:")
    message(STATUS "    version: ${Qt5Widgets_VERSION}")
    message(STATUS "    libraries: ${Qt5Widgets_LIBRARIES}")
    message(STATUS "    include path: ${Qt5Widgets_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: Qt5 not found")
endif()

##############################################################################
# HEADERS
##############################################################################
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}  
  ${Qt5Widgets_INCLUDE_DIRS}

)
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})
add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${OpenCV_LIBRARIES}
                 ${PCL_LIBRARY_DIRS}
                 ${Qt5Widgets_LIBRARIES}

                 )

##############################################################################
# SOURCE CODE
##############################################################################

set(GUI_HEADERS "include/Visualizer.h")
set(GUI_SOURCE "src/Visualizer.cpp")
set(GUI_FILE "src/Visualizer.ui")
set(MAIN_SOURCE "main.cpp")
FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE SOURCES_FILES include *.cpp)
FILE(GLOB_RECURSE GUI_FILE include *.ui)
add_custom_target(headers SOURCES ${HEADER_FILES} ${SOURCES_FILES} ${GUI_FILE} ${MAIN_SOURCE})

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} ${GUI_HEADERS} ${GUI_SOURCE} ${GUI_FILE})

##############################################################################
# TARGET LIBRARIES
##############################################################################

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen Qt5::Widgets)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")

