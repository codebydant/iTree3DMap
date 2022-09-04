# add_subdirectory(control_Image_Registration)

# add_subdirectory(DBScan_Octrees-master)
# add_subdirectory(gui_Crown_SegParam)
# add_subdirectory(gui_Trunk_SegParam)
add_subdirectory(openMVG)
add_subdirectory(scale_factor_mod)

add_library(app_external_deps INTERFACE)

# target_include_directories(app_external_deps INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(app_external_deps INTERFACE
    3DMappingOpenMVG
    scalefactormod
)