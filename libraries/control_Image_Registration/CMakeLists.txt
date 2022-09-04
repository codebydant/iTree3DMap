# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.15)
project(control_point_Reg VERSION 1.0.0)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

message(STATUS "Child: ${PROJECT_NAME}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
include(ExternalProject)

# fetch_project(
# NAME cereal
# URL https://github.com/USCiLab/cereal/archive/v1.3.2.tar.gz}
# CMAKE_ARGS
# )
find_package(OpenMVG REQUIRED)

if(OpenMVG_FOUND)
    message(STATUS "    version: ${OPENMVG_VERSION}")
    message(STATUS "    path: ${OPENMVG_INCLUDE_DIRECTORY}")

# set(OpenMVG_isFound true CACHE BOOL "openMVG FOUND")
# set(OpenMVG_INCLUD_DIR ${OPENMVG_INCLUDE_DIRS} CACHE INTERNAL "" FORCE)
else()
    message(FATAL_ERROR "    ERROR: OpenMVG not found")
endif()

ExternalProject_Add(cerealdep

    PREFIX 3rd_party
    URL https://github.com/USCiLab/cereal/archive/v1.3.2.tar.gz
    LIST_SEPARATOR | # Use the alternate list separator
    CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DWITH_WERROR=OFF
    -DBUILD_DOC=OFF
    -DBUILD_SANDBOX=OFF
    -DBUILD_SANDBOX=OFF
    -DBUILD_TESTS=OFF
)

SET(CEREAL_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/cerealdep/include)
SET(CEREAL_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rd_party/src/cerealdep-build)
add_library(cereal INTERFACE IMPORTED)
set_target_properties(cereal PROPERTIES IMPORTED_LOCATION ${CEREAL_LIB_DIR})

# target_link_libraries(cereal INTERFACE cerealdep)

# Instruct CMake to run moc automatically when needed.
# set(CMAKE_C_COMPILER /usr/bin/gcc)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)
# set(QT_QMAKE_EXECUTABLE /usr/lib/x86_64-linux-gnu/qt5/bin/qmake)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# # Find includes in corresponding build directories
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ##############################################################################
# # HEADERS
# ##############################################################################
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS}

    # ${OPENMVG_INCLUDE_DIRS}
)

include(CheckFunctionExists)

# # Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

# #############################################################################
# LIBRERIAS
# #############################################################################
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES}

    ${OPENMVG_LIBRARIES}
)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE} "src/MainWindow.hpp" "src/MainWindow.cpp" "src/document.hpp")
message(STATUS "    packages: ${OPENMVG_LIBRARIES}")
message(STATUS "    packages: ${OPENMVG}")

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_include_directories(${PROJECT_NAME} PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Widgets OpenMVG::openMVG_sfm OpenMVG::openMVG_system OpenMVG::openMVG_features OpenMVG::openMVG_image OpenMVG::openMVG_numeric OpenMVG::openMVG_lemon OpenMVG::openMVG_exif OpenMVG::openMVG_geometry cereal Eigen3::Eigen)

# target_link_libraries(${PROJECT_NAME} Qt5::Widgets cereal ${OPENMVG_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -mno-avx -mno-avx2 -Werror)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

MESSAGE("=========================================")
MESSAGE(" COMPILED")
MESSAGE("=========================================")
