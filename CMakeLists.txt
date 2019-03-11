
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

if ( NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    message( "TYSOCMJC> Sending generated libs to OWN build directory" )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libs" )
else()
    message( "TYSOCMJC> Sending generated libs to EXTERNAL build directory" )
endif()

message( "TYSOCMJC> LIBRARY_OUT_DIR: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" )

set( TYSOCMJC_IS_MASTER_PROJECT OFF )
if( CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( "TYSOCMJC> Configuring as MASTER project" )
    set( TYSOCMJC_IS_MASTER_PROJECT ON )
else()
    message( "TYSOCMJC> Configuring as CHILD project" )
endif()

# Add core targets only if master project. If not, parent project must configure it
if ( TYSOCMJC_IS_MASTER_PROJECT )
    # Configure core functionality
    add_subdirectory( tysoc )
endif()

# Configure tysocMjc project *************************************************

# Set the lbirary paths as props to allow usage from other projects
set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsMujoco.so" )
set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocRenderingMjcViz.so" )

# get properties defined in core library
get_property( VAR_TYSOC_PATH_RESOURCES              GLOBAL PROPERTY PROP_TYSOC_PATH_RESOURCES )
get_property( VAR_TYSOC_PATH_WORKING_DIR            GLOBAL PROPERTY PROP_TYSOC_PATH_WORKING_DIR )
get_property( VAR_TYSOC_PATH_MJCF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_MJCF_TEMPLATES )
get_property( VAR_TYSOC_PATH_URDF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_URDF_TEMPLATES )
get_property( VAR_TYSOC_PATH_RLSIM_TEMPLATES        GLOBAL PROPERTY PROP_TYSOC_PATH_RLSIM_TEMPLATES )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ      GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ     GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ )
get_property( VAR_TYSOC_BACKEND_PHYSICS_MUJOCO      GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO )

# repeat definitions from core library, as they are not propagated up
add_definitions( -DTYSOC_PATH_RESOURCES="${VAR_TYSOC_PATH_RESOURCES}" )
add_definitions( -DTYSOC_PATH_WORKING_DIR="${VAR_TYSOC_PATH_WORKING_DIR}" )
add_definitions( -DTYSOC_PATH_MJCF_TEMPLATES="${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_URDF_TEMPLATES="${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_RLSIM_TEMPLATES="${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_GLVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_MJCVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ}" )
add_definitions( -DTYSOC_BACKEND_PHYSICS_MUJOCO="${VAR_TYSOC_BACKEND_PHYSICS_MUJOCO}" )

# test all properties
message( "PROP_TYSOC_PATH_RESOURCES:            ${VAR_TYSOC_PATH_RESOURCES}" )
message( "PROP_TYSOC_PATH_WORKING_DIR:          ${VAR_TYSOC_PATH_WORKING_DIR}" )
message( "PROP_TYSOC_PATH_MJCF_TEMPLATES:       ${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_URDF_TEMPLATES:       ${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_RLSIM_TEMPLATES:      ${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ:    ${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ:   ${VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ}" )
message( "PROP_TYSOC_BACKEND_PHYSICS_MUJOCO:    ${VAR_TYSOC_BACKEND_PHYSICS_MUJOCO}" )

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add this project headers to includepath
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )

# add the core dependency headers to includepath
if ( TYSOCMJC_IS_MASTER_PROJECT )
    # core dependency to use is own submodule
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/tinyxml2" )
else()
    # core dependency to use if parent core submodule, names as "core"
    include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )
endif()


include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )
get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
add_definitions( -DMUJOCO_LICENSE_FILE="${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "MuJoCo license path: ${MUJOCO_LICENSE_ABSOLUTE_PATH}" )
message( "Sources: ${TYSOCMJC_SOURCES}" )

add_library( tysocPhysicsMujoco SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocPhysicsMujoco
                       tysocCore
                       mujoco200nogl )

# ****************************************************************************

# Configure visualizer with mujoco's rendering backend
add_subdirectory( mujocoviz )

# Only generate samples if not using as dependency
if ( TYSOCMJC_IS_MASTER_PROJECT )
    # Configure examples
    add_subdirectory( examples )
endif()