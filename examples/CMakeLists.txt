
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

# test_helloworld example
add_executable( test_helloworld
                test_helloworld.cpp )
target_link_libraries( test_helloworld
                       mujoco200 glfw
                       ${OPENGL_LIBRARIES} 
                       ${GLEW_LIBRARIES} )

# test_mjcf_generic example
add_executable( test_mjcf_generic
                test_mjcf_generic.cpp )
target_link_libraries( test_mjcf_generic
                       mjcpp )

# test_mjcf_api example
add_executable( test_mjcf_api
                test_mjcf_api.cpp )
target_link_libraries( test_mjcf_api
                       mjcpp )

# test_mjcf_renderer integration example
add_executable( test_mjcf_renderer
                test_mjcf_renderer.cpp )
target_link_libraries( test_mjcf_renderer
                       cat1 
                       mjcppnogl )

# test_mjterrain example
add_executable( test_mjterrain
                test_mjterrain.cpp )
target_link_libraries( test_mjterrain
                       mjcpp
                       tysocMjc )