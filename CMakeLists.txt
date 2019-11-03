
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

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
    # Configure dependencies for tysoc
    add_subdirectory( ext )
    # Configure core functionality
    add_subdirectory( core )
endif()

# Configure tysocMjc project *************************************************

# Set the library paths as props to allow usage from other projects
if ( WIN32 )
    set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsMujoco.dll" )
    set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocRenderingMjcViz.dll" )
elseif ( UNIX )
    if ( APPLE )
        set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsMujoco.dylib" )
        set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocRenderingMjcViz.dylib" )
    else ()
        set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocPhysicsMujoco.so" )
        set_property( GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libtysocRenderingMjcViz.so" )
    endif()
endif()

# get properties defined in core library
get_property( VAR_TYSOC_PATH_RESOURCES              GLOBAL PROPERTY PROP_TYSOC_PATH_RESOURCES )
get_property( VAR_TYSOC_PATH_WORKING_DIR            GLOBAL PROPERTY PROP_TYSOC_PATH_WORKING_DIR )
get_property( VAR_TYSOC_PATH_MJCF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_MJCF_TEMPLATES )
get_property( VAR_TYSOC_PATH_URDF_TEMPLATES         GLOBAL PROPERTY PROP_TYSOC_PATH_URDF_TEMPLATES )
get_property( VAR_TYSOC_PATH_RLSIM_TEMPLATES        GLOBAL PROPERTY PROP_TYSOC_PATH_RLSIM_TEMPLATES )
get_property( VAR_TYSOC_PATH_MESHES_DIR             GLOBAL PROPERTY PROP_TYSOC_PATH_MESHES_DIR )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ      GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ )
get_property( VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ     GLOBAL PROPERTY PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ )
get_property( VAR_TYSOC_BACKEND_PHYSICS_MUJOCO      GLOBAL PROPERTY PROP_TYSOC_BACKEND_PHYSICS_MUJOCO )

# repeat definitions from core library, as they are not propagated up
add_definitions( -DTYSOC_PATH_RESOURCES="${VAR_TYSOC_PATH_RESOURCES}" )
add_definitions( -DTYSOC_PATH_WORKING_DIR="${VAR_TYSOC_PATH_WORKING_DIR}" )
add_definitions( -DTYSOC_PATH_MJCF_TEMPLATES="${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_URDF_TEMPLATES="${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_RLSIM_TEMPLATES="${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
add_definitions( -DTYSOC_PATH_MESHES_DIR="${VAR_TYSOC_PATH_MESHES_DIR}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_GLVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
add_definitions( -DTYSOC_BACKEND_GRAPHICS_MJCVIZ="${VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ}" )
add_definitions( -DTYSOC_BACKEND_PHYSICS_MUJOCO="${VAR_TYSOC_BACKEND_PHYSICS_MUJOCO}" )

# test all properties
message( "PROP_TYSOC_PATH_RESOURCES:            ${VAR_TYSOC_PATH_RESOURCES}" )
message( "PROP_TYSOC_PATH_WORKING_DIR:          ${VAR_TYSOC_PATH_WORKING_DIR}" )
message( "PROP_TYSOC_PATH_MJCF_TEMPLATES:       ${VAR_TYSOC_PATH_MJCF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_URDF_TEMPLATES:       ${VAR_TYSOC_PATH_URDF_TEMPLATES}" )
message( "PROP_TYSOC_PATH_RLSIM_TEMPLATES:      ${VAR_TYSOC_PATH_RLSIM_TEMPLATES}" )
message( "PROP_TYSOC_PATH_MESHES_DIR:           ${VAR_TYSOC_PATH_MESHES_DIR}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_GLVIZ:    ${VAR_TYSOC_BACKEND_GRAPHICS_GLVIZ}" )
message( "PROP_TYSOC_BACKEND_GRAPHICS_MJCVIZ:   ${VAR_TYSOC_BACKEND_GRAPHICS_MJCVIZ}" )
message( "PROP_TYSOC_BACKEND_PHYSICS_MUJOCO:    ${VAR_TYSOC_BACKEND_PHYSICS_MUJOCO}" )

# grab all the sources for compilation
file( GLOB_RECURSE TYSOCMJC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

# add this project headers to includepath
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )

# include_dirs for tysoc-core dependency: either case (master or non-master project) ... 
# we should find it at root level
include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )

if ( WIN32 )
    # Path to mujoco library in the windows system
    include_directories( "C:/.mujoco/mujoco200_win64/include" )
    link_directories( "C:/.mujoco/mujoco200_win64/bin" )
elseif ( UNIX )
    if ( APPLE )
        # Apple kind of needs to specify this (at least in the system I had available, ...
        # which was a brand new macbookair from a friend)
        include_directories( /usr/local/include )
        link_directories( /usr/local/lib )
        # Path to mujoco library in the macos system
        include_directories( "~/.mujoco/mujoco200_macos/include" )
        link_directories( "~/.mujoco/mujoco200_macos/bin" )
    else ()
        # Path to mujoco library in the linux system
        include_directories( "~/.mujoco/mujoco200_linux/include" )
        link_directories( "~/.mujoco/mujoco200_linux/bin" )
    endif()
endif()

# mujoco configuration
set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )

if ( WIN32 )
    get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "C:/.mujoco/mjkey.txt" ABSOLUTE )
elseif ( UNIX )
    get_filename_component( MUJOCO_LICENSE_ABSOLUTE_PATH "~/.mujoco/mjkey.txt" ABSOLUTE )
endif()

add_definitions( -DMUJOCO_LICENSE_FILE="${MUJOCO_LICENSE_ABSOLUTE_PATH}" )

message( "MuJoCo license path: ${MUJOCO_LICENSE_ABSOLUTE_PATH}" )
message( "Sources: ${TYSOCMJC_SOURCES}" )

add_library( tysocPhysicsMujoco SHARED
             ${TYSOCMJC_SOURCES} )
target_link_libraries( tysocPhysicsMujoco
                       tysocCore
                       mujoco200nogl )

# ****************************************************************************

# Only generate samples if not using as dependency
if ( TYSOCMJC_IS_MASTER_PROJECT )
    # Configure examples
    add_subdirectory( examples )
endif()

#*****************************************************************************

#### # Configure tests (barebone external tests)
#### add_subdirectory( tests )