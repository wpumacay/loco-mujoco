
message( "Configuring examples" )

set( CMAKE_CXX_STANDARD 11 )

set( CMAKE_CXX_FLAGS "-mavx -pthread -g ${CMAKE_CXX_FLAGS}" )

include_directories( "~/.mujoco/mujoco200_linux/include" )
link_directories( "~/.mujoco/mujoco200_linux/bin" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

# helloworld example
add_executable( helloworld
                helloworld.cpp )
target_link_libraries( helloworld
                       mujoco200 glfw
                       ${OPENGL_LIBRARIES} 
                       ${GLEW_LIBRARIES} )

# basicIntegration example
include_directories( "${CMAKE_SOURCE_DIR}/glengine/include" )

add_executable( basicIntegration
                basicIntegration.cpp )
target_link_libraries( basicIntegration
                       mujoco200nogl
                       gengine )


# tinyxml2 example
include_directories( "${CMAKE_SOURCE_DIR}/tinyxml2" )

add_executable( test_tinyxml2
                test_tinyxml2.cpp )
target_link_libraries( test_tinyxml2
                       tinyxml2 )

# mjcf generic example
add_executable( test_mjcf_generic
                test_mjcf_generic.cpp
                mjcf/mjcf.cpp
                mjcf/mjcf_elements.cpp
                mjcf/mjcf_common.cpp
                mjcf/mjcf_utils.cpp )
target_link_libraries( test_mjcf_generic
                       tinyxml2 )

# mjcf api example
add_executable( test_mjcf_api
                test_mjcf_api.cpp
                pendulum.cpp
                mjcint/mjcint_api.cpp
                mjcf/mjcf_api.cpp
                mjcf/mjcf.cpp
                mjcf/mjcf_elements.cpp
                mjcf/mjcf_common.cpp
                mjcf/mjcf_utils.cpp )
target_link_libraries( test_mjcf_api
                       tinyxml2
                       mujoco200
                       glfw
                       ${OPENGL_LIBRARIES} 
                       ${GLEW_LIBRARIES} )

# mjcf renderer integration example
add_executable( test_mjcf_renderer
                test_mjcf_renderer.cpp
                pendulum.cpp
                mjcint/mjcint_api.cpp
                mjcf/mjcf_api.cpp
                mjcf/mjcf.cpp
                mjcf/mjcf_elements.cpp
                mjcf/mjcf_common.cpp
                mjcf/mjcf_utils.cpp )
target_link_libraries( test_mjcf_renderer
                       tinyxml2
                       mujoco200nogl
                       gengine glfw
                       ${OPENGL_LIBRARIES} 
                       ${GLEW_LIBRARIES} )