
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

add_definitions( -DTYSOCMJC_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/" )

# Configure external dependencies
add_subdirectory( ext )

# Configure visualizer // @TODO: Should move this out from src to the ext directory
add_subdirectory( tysocViz )

# Configure tysocMjc project *************************************************

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/mjCpp/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/tysocTerrain/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui/examples" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysocViz/include" )

# mujoco dependencies
include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )
# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )
get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
add_definitions( -DMUJOCO_LICENSE_FILE="${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

#message( "MUJOCO_LICENSE_ABSOLUTE_PATH: ${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "TYSOCMJC_SOURCES: ${TYSOCMJC_SOURCES}" )

# Configure the resulting library
add_library( tysocMjc
			 ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMjc
					   tysocViz
					   mjcppnogl
                       mjcpp
					   tysoc )

# ****************************************************************************

# Configure examples
add_subdirectory( examples )
