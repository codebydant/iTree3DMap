set(EXECUTABLE_NAME "3dmapping")

# build example binary #
add_executable(${EXECUTABLE_NAME} "examples/main.cpp")

# link dependencies #
target_link_libraries(${EXECUTABLE_NAME} PRIVATE
    ${PCL_LIBRARIES}
    3DMappingOpenMVG
)