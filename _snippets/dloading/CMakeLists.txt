
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

include_directories( "${CMAKE_SOURCE_DIR}/include" )

add_library( shape_unreal SHARED
             src/ShapeUnreal.cpp
             src/Shape.cpp )

add_library( shape_ogre SHARED
             src/ShapeOgre.cpp
             src/Shape.cpp )

add_executable( main
                src/main.cpp
                src/Shape.cpp )
target_link_libraries( main 
                       dl )