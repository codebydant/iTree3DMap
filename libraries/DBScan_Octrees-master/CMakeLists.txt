##############################################################################
# CMAKE CONFIGURATION
##############################################################################
project(dbscan VERSION 1.0.0)

# set build type = Debug mode
#set(CMAKE_BUILD_TYPE Release)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the include directive in the same project folder
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

##############################################################################
# HEADERS
##############################################################################
include_directories(${PCL_INCLUDE_DIRS})
#include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${PCL_LIBRARY_DIRS})

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} "src/dbScan.cpp" "src/OctreeGenerator.cpp" "src/cluster.cpp" "src/dbScan.h" "src/OctreeGenerator.h" "src/cluster.h" "src/HTRBasicDataStructures.h")

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
