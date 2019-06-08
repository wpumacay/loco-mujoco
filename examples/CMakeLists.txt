
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

add_executable( test_runtime
                test_runtime.cpp )
target_link_libraries( test_runtime
                       tysocCore )

add_executable( test_com_forces
                test_com_forces.cpp )
target_link_libraries( test_com_forces
                       tysocCore )

add_executable( test_terrain_static
                test_terrain_static.cpp )
target_link_libraries( test_terrain_static
                       tysocCore )

add_executable( test_sandbox_bodies
                test_sandbox_bodies.cpp )
target_link_libraries( test_sandbox_bodies
                       tysocCore )

add_executable( test_sandbox_joints
                test_sandbox_joints.cpp )
target_link_libraries( test_sandbox_joints
                       tysocCore )

add_executable( test_sandbox_stack_bodies
                test_sandbox_stack_bodies.cpp )
target_link_libraries( test_sandbox_stack_bodies
                       tysocCore )

add_executable( test_sandbox_joints_compound
                test_sandbox_joints_compound.cpp )
target_link_libraries( test_sandbox_joints_compound
                       tysocCore )

add_subdirectory( cpp )