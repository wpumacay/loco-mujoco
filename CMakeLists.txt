
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

add_definitions( -DTYSOCMJC_RESOURCES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/res/" )
add_definitions( -DTYSOC_DLLIBS_PATH="${CMAKE_BINARY_DIR}/" )

# Configure external dependencies
add_subdirectory( ext )
# Configure pybind11
add_subdirectory( pybind11 )

# Configure visualizer // @TODO: Should move this out from src to the ext directory ?
add_subdirectory( tysocCustomViz )
add_subdirectory( tysocMujocoViz )

# Configure tysocMjc project *************************************************

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/tysocCore/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/tysocCore/ext/json" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui/examples" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysocCustomViz/include" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/tysocMujocoViz/include" )

# mujoco dependencies
find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )
include_directories( ${GLEW_INCLUDE_DIRS} )

include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )
get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
add_definitions( -DMUJOCO_LICENSE_FILE="${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

#message( "MUJOCO_LICENSE_ABSOLUTE_PATH: ${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "TYSOCMJC_SOURCES: ${TYSOCMJC_SOURCES}" )

## @DIRTY: for testing each library, we are making use of ...
##         compiling the whole library each time and check it works.
##         For other purposes, we must use dynamic loading, and because the ...
##         implementations are polymorphic and (as far as I've made them) because ...
##         they implement a shared API, everything should work fine in *nix ...
##         of windows systems, as long as we implement the appropiate loading ...
##         functionality using the appropiate library loading APIs in these platforms.

## @TOUGHT: this seems kind of similar to how opengl has core functionality and it's loaded ...
##          using a loader like glew, etc, and also how some offer extensions (which could be ...
##          similar to how mujoco could provide this in some cases, and then maybe bullet could ...
##          catch up by providing this functionality as well be an user implementing what is needed)

## @TOUGHT: there seems to be a potential issue to integrate with special full "rendering" engines like ...
##          unreal, because it is very coupled. Perhaps we could extract a portion of the code to make ...
##          it work the way we expect backends to be provided.

# @TEST: Testing tysocMjc with the custom visualizer
add_library( tysocMjcCustomViz SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMjcCustomViz
                       tysocCustomViz # this uses the custom visualizer
                       mujoco200nogl
                       tysocCore )

# @TEST: Testing tysocMjc with the mujoco visualizer (try to force first mix of gl and nongl mujoco)
add_library( tysocMjcMujocoViz SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMjcMujocoViz
                       tysocMujocoViz # this uses the mujoco visualizer
                       mujoco200
                       glfw
                       ${OPENGL_LIBRARIES}
                       ${GLEW_LIBRARIES}
                       tysocCore )

# @TEST: Testing tysocMjc with no visualizer
add_library( tysocMujoco SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocMujoco
                       tysocCore
                       mujoco200nogl )

# ****************************************************************************

# Configure examples
add_subdirectory( examples )

## # Configure bindings
add_subdirectory( pytysoc )