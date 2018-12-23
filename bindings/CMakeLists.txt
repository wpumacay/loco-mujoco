

# add all dependencies to include path
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include/" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/cat1/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/tysocCore/include" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui" )
include_directories( "${CMAKE_SOURCE_DIR}/ext/imgui/examples" )
include_directories( "${CMAKE_SOURCE_DIR}/tysocCustomViz/include" )
include_directories( "${CMAKE_SOURCE_DIR}/tysocMujocoViz/include" )

# grab all the sources for compilation
file( GLOB_RECURSE PYTYSOC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

pybind11_add_module( pytysoc
                     ${PYTYSOC_SOURCES} )

target_link_libraries( pytysoc PRIVATE
                       tysocMjcCustomViz )