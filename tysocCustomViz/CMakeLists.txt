cmake_minimum_required( VERSION 3.0.0 )

project( tysocCustomViz )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

file( GLOB_RECURSE TYSOC_CUSTOM_VIZ_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/mjCpp/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/tysocCore/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )

add_library( tysocCustomViz
             ${TYSOC_CUSTOM_VIZ_SOURCES} )
target_link_libraries( tysocCustomViz
                       cat1
                       tysocCore
                       imgui )