##############################################################################
# CMAKE CONFIGURATION
##############################################################################

cmake_minimum_required(VERSION 3.10.2)
project(control_point_Reg VERSION 1.0.0)

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
set(PATH_EIGEN /opt/eigen-3.3.4/build)
set(openMVG_PATH /home/daniel/Documents/iTree3DMap/libraries/openMVG/build)
#set(openMVG_PATH /usr/local/share/openMVG/cmake)

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH /home/daniel/Documents/iTree3DMap/libraries/openMVG/src/cmakeFindModules)

set(CMAKE_INSTALL_PREFIX ${openMVG_PATH})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# PACKAGES
##############################################################################
find_package(OpenMVG REQUIRED)
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(OpenCV 3.4.1 REQUIRED PATHS ${OpenCV_DIR})
if(OpenCV_FOUND )
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 3.2.0 not found")
endif()

find_package(Ceres REQUIRED)

find_package(Qt5Widgets REQUIRED)
if(Qt5Widgets_FOUND)
    message(STATUS "Qt5 library status:")
    message(STATUS "    version: ${Qt5Widgets_VERSION}")
    message(STATUS "    libraries: ${Qt5Widgets_LIBRARIES}")
    message(STATUS "    include path: ${Qt5Widgets_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: Qt5 not found")
endif()

find_package(Eigen3 3.3.4 REQUIRED NO_MODULE PATHS ${PATH_EIGEN})
if(Eigen3_FOUND)
   message(STATUS "Eigen status:")
   message(STATUS "    version: ${Eigen3_VERSION}")
else()
   message(FATAL_ERROR "    ERROR: Eigen ${Eigen3_VERSION} not found")
endif()

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


##############################################################################
# HEADERS
##############################################################################

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS} )
include_directories(${OPENMVG_INCLUDE_DIRS})

include(CheckFunctionExists)

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

LINK_DIRECTORIES(${OpenCV_LIBS_DIR})
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES})
LINK_DIRECTORIES(${OPENMVG_LIBRARIES})

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "main.cpp")


##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME}  ${MAIN_SOURCE} src/MainWindow.hpp src/MainWindow.cpp src/GraphicsView.cpp src/GraphicsView.hpp src/ControlPoint2DNode.cpp src/ControlPoint2DNode.hpp src/document.hpp src/ControlPointTableView.cpp src/ControlPointTableView.hpp )

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen Qt5::Widgets ${OPENMVG_LIBRARIES} OpenMVG::openMVG_sfm OpenMVG::openMVG_system OpenMVG::openMVG_stlplus OpenMVG::openMVG_features OpenMVG::openMVG_image OpenMVG::openMVG_numeric OpenMVG::openMVG_lemon OpenMVG::openMVG_exif OpenMVG::openMVG_geometry)

MESSAGE("=========================================")
MESSAGE(" COMPILED")
MESSAGE("=========================================")

