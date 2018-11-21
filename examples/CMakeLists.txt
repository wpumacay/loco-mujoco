
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

# test_tysoc example
add_executable( test_tysoc
                test_tysoc.cpp )
target_link_libraries( test_tysoc
                       tysocMjc )