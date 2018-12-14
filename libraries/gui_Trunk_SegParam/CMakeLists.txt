##############################################################################
# CMAKE CONFIGURATION
##############################################################################
project(gui-Trunk VERSION 1.0.0)

#set(CMAKE_C_COMPILER /usr/bin/gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/g++)
#set(QT_QMAKE_EXECUTABLE "/usr/lib/x86_64-linux-gnu/qt5/bin/qmake")

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# HEADERS
##############################################################################
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRARIES
##############################################################################
link_directories(${Qt5Widgets_LIBRARIES})

##############################################################################
# SOURCE CODE
##############################################################################
set(GUI_HEADERS "include/Visualizer.h")
set(GUI_SOURCE "src/Visualizer.cpp")
set(GUI_FILE "src/Visualizer.ui")
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} ${GUI_HEADERS} ${GUI_SOURCE} ${GUI_FILE})

##############################################################################
# TARGET LIBRARIES
##############################################################################

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")

