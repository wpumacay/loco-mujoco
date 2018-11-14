
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

add_definitions( -DTYSOCMJC_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/")

# Configure external dependencies
add_subdirectory( ext )

# Configure tysocMjc project *************************************************

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/mjCpp/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/tysocTerrain/include" )

# mujoco dependencies
include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )
# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )

message( "TYSOCMJC_SOURCES: ${TYSOCMJC_SOURCES}" )

# Configure the resulting library
add_library( tysocMjc
			 ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMjc
					   cat1
					   mjcppnogl
					   tysocterrain )

# ****************************************************************************

# Configure examples
add_subdirectory( examples )
