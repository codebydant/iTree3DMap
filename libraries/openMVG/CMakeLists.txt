# minimum cmake version #
cmake_minimum_required(VERSION 3.15)

# append functions.cmake to current workspace #
include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

# library description #
project(3DMappingOpenMVG
    VERSION 1.0.3
    DESCRIPTION "3D Mapping library to interface OpenMVG sfm incremental pipeline"
    LANGUAGES CXX
)

# option to build example project #
option(BUILD_EXAMPLE "Build an example file to run the pipeline." OFF)

# dependencies #
find_package(OpenMVG REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io)
fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
)

# build library #
add_library(${PROJECT_NAME} SHARED)

# define source files #
target_sources(${PROJECT_NAME} PRIVATE "src/Mapping.cpp")

# define target properties #
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${CMAKE_CURRENT_LIST_DIR}/include/3DMapper/Mapping.hpp
)

# define header files location #
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE ${PCL_INCLUDE_DIRS}
)

# link library dependencies to target library #
target_link_libraries(${PROJECT_NAME}
    PRIVATE ${PCL_LIBRARIES}
    PUBLIC cloudparse
)

# compilation flags: MMX, SSE(1, 2, 3, 3S, 4.1, 4.2), CLMUL, RdRand, VT-x, x86-64 #
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp
    -mmmx
    -msse
    -msse2
    -msse3
    -mssse3
    -msse4.2
    -msse4.1
    -mno-sse4a
    -mno-avx
    -mno-avx2
    -mno-fma
    -mno-fma4
    -mno-f16c
    -mno-xop
    -mno-bmi
    -mno-bmi2
    -mrdrnd
    -mno-3dnow
    -mlzcnt
    -mfsgsbase
    -mpclmul
)

# install library #
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install thirdparty dependencies for runtime #
file(COPY thirdparty/SfM_SequentialPipeline.py DESTINATION ${CMAKE_BINARY_DIR})
file(COPY thirdparty/sensor_width_camera_database.txt DESTINATION ${CMAKE_BINARY_DIR})

# build example project #
if(BUILD_EXAMPLE)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/examples)
endif()