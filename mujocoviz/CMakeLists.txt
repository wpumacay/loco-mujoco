cmake_minimum_required( VERSION 3.0.0 )

project( mujocoviz )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

# some dependencies
find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )
include_directories( ${GLEW_INCLUDE_DIRS} )

include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

file( GLOB_RECURSE TYSOC_MUJOCO_VIZ_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" )

if ( TYSOCMJC_IS_MASTER_PROJECT )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/tinyxml2" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/imgui" )
    include_directories( "${CMAKE_SOURCE_DIR}/tysoc/ext/imgui/examples" )
else()
    include_directories( "${CMAKE_SOURCE_DIR}/core/include" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/json" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/tinyxml2" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/imgui" )
    include_directories( "${CMAKE_SOURCE_DIR}/core/ext/imgui/examples" )
endif()

add_library( tysocRenderingMjcViz SHARED
             ${TYSOC_MUJOCO_VIZ_SOURCES} )
target_link_libraries( tysocRenderingMjcViz
                       tysocCore
                       imgui_gl2
                       mujoco200
                       glfw
                       ${OPENGL_LIBRARIES}
                       ${GLEW_LIBRARIES} )