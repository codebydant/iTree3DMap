##############################################################################
# CMAKE CONFIGURATION
##############################################################################
project(control_point_Reg VERSION 1.0.0)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

message(STATUS "Child: ${PROJECT_NAME}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_C_COMPILER /usr/bin/gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/g++)
#set(QT_QMAKE_EXECUTABLE /usr/lib/x86_64-linux-gnu/qt5/bin/qmake)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

## Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

###############################################################################
## HEADERS
###############################################################################
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS}
                    ${OPENMVG_INCLUDE_DIRS}
)

include(CheckFunctionExists)

## Use the compile definitions defined in the Qt 5 Widgets module
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
target_link_libraries(${PROJECT_NAME} Qt5::Widgets OpenMVG::openMVG_sfm OpenMVG::openMVG_system OpenMVG::openMVG_stlplus OpenMVG::openMVG_features OpenMVG::openMVG_image OpenMVG::openMVG_numeric OpenMVG::openMVG_lemon OpenMVG::openMVG_exif OpenMVG::openMVG_geometry)

MESSAGE("=========================================")
MESSAGE(" COMPILED")
MESSAGE("=========================================")

