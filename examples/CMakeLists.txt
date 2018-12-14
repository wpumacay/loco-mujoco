
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

# test_kintree example
add_executable( test_kintree
                test_kintree.cpp )
target_link_libraries( test_kintree
                       tysocMjc )