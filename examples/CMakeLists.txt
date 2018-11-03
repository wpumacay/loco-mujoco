
message( "Configuring examples" )

set( CMAKE_CXX_STANDARD 11 )

set( CMAKE_CXX_FLAGS "-mavx -pthread ${CMAKE_CXX_FLAGS}" )

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