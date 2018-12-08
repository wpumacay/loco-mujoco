cmake_minimum_required( VERSION 3.0.0 )

project( tysocViz )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

file( GLOB_RECURSE TYSOC_VIZ_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/tysocTerrain/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )

## message( "IMGUI_INCLUDE_DIRS: ${IMGUI_INCLUDE_DIRS}" )

add_library( tysocViz
             ${TYSOC_VIZ_SOURCES} )
target_link_libraries( tysocViz
                       cat1
                       tysoc
                       imgui )
