
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

add_definitions( -DTYSOC_DLLIBS_PATH="${CMAKE_BINARY_DIR}/" )

# Configure core functionality
add_subdirectory( tysoc )

# Configure tysocMjc project *************************************************

# Definitions used for development. Python bindings do not use these definitions, but ...
# instead we load the paths dynamically according to the user configuration/installation.
add_definitions( -DTYSOC_MUJOCO_LIB="${CMAKE_BINARY_DIR}/libtysocMujoco.so" )
add_definitions( -DTYSOC_MUJOCO_VIZ_LIB="${CMAKE_BINARY_DIR}/mujocoviz/libtysocMujocoViz.so" )
# repeat definitions from core library, as it seems they are not propagated up
add_definitions( -DTYSOCCORE_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tysoc/res/" )
add_definitions( -DTYSOCCORE_WORKING_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tysoc/res/xml/" )
add_definitions( -DTYSOCCORE_MJCF_TEMPLATES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tysoc/res/templates/mjcf/" )
add_definitions( -DTYSOCCORE_URDF_TEMPLATES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tysoc/res/templates/urdf/" )
add_definitions( -DTYSOCCORE_RLSIM_TEMPLATES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tysoc/res/templates/rlsim/" )
add_definitions( -DTYSOCCORE_GLVIZ_LIB="${CMAKE_BINARY_DIR}/tysoc/glviz/libtysocGlviz.so" )

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysoc/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysoc/ext/json" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysoc/ext/tinyxml2" )

include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )
get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
add_definitions( -DMUJOCO_LICENSE_FILE="${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "MuJoCo license path: ${MUJOCO_LICENSE_ABSOLUTE_PATH}" )
message( "Sources: ${TYSOCMJC_SOURCES}" )

add_library( tysocMujoco SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMujoco
                       tysocCore
                       mujoco200nogl )

# ****************************************************************************

# Configure visualizer with mujoco's rendering backend
add_subdirectory( mujocoviz )

# Configure examples
add_subdirectory( examples )