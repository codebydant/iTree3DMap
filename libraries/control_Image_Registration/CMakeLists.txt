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

set(OpenMVG_PATH /home/daniel/Documents/iTree3DMap/libraries/openMVG/build)
#set(CMAKE_INSTALL_PREFIX ${OpenMVG_PATH})
set(CMAKE_MODULE_PATH ${OpenMVG_PATH}/../src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# PACKAGESss
##############################################################################
find_package(OpenMVG REQUIRED PATHS ${OpenMVG_PATH})
if(OpenMVG_FOUND)
    message(STATUS "OpenMVG library status:")
    message(STATUS "    version: ${OPENMVG_VERSION}")
    message(STATUS "path: ${OPENMVG_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: OpenMVG not found")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

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
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS}
                    ${OPENMVG_INCLUDE_DIRS}
)

include(CheckFunctionExists)

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES}
                 ${OPENMVG_LIBRARIES}
)

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} "src/MainWindow.hpp" "src/MainWindow.cpp" "src/document.hpp")

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${OPENMVG_LIBRARIES} OpenMVG::openMVG_sfm OpenMVG::openMVG_system OpenMVG::openMVG_stlplus OpenMVG::openMVG_features OpenMVG::openMVG_image OpenMVG::openMVG_numeric OpenMVG::openMVG_lemon OpenMVG::openMVG_exif OpenMVG::openMVG_geometry)

MESSAGE("=========================================")
MESSAGE(" COMPILED")
MESSAGE("=========================================")

