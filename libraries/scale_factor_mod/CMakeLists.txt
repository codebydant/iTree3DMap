cmake_minimum_required(VERSION 3.15)

# append functions.cmake to current workspace #
include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

project(scalefactormod
        VERSION 1.0.0
        DESCRIPTION "A single header scale factor module"
        LANGUAGES CXX
)

find_package(tinyxml2 REQUIRED)

if(tinyxml2_FOUND)
        message(STATUS "tinyxml2 status: found")
else()
        message(FATAL_ERROR "    ERROR: tinyxml2 not found")
endif()

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>)

# link library dependencies to target library #
target_link_libraries(${PROJECT_NAME}
        INTERFACE tinyxml2
)

install(FILES ${CMAKE_CURRENT_LIST_DIR}/include/itreemap/scale_factor.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/itreemap)
