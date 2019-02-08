
cmake_minimum_required( VERSION 3.0.0 )

set( CMAKE_CXX_STANDARD 11 )

set( CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}" )

add_definitions( -DJSON_TEST_RESOURCES="${CMAKE_SOURCE_DIR}/../../res/templates/rlsim/" )

include_directories( "${CMAKE_SOURCE_DIR}/include" )

add_executable( parsing
                parsing.cpp )