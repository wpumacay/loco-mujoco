
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

add_definitions( -DMUJOCO_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/")

add_subdirectory( glengine )
add_subdirectory( editor )
add_subdirectory( examples )

