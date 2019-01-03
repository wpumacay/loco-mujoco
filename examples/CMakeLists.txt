
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

## # test_kintree example
## add_executable( test_kintree
##                 test_kintree.cpp )
## target_link_libraries( test_kintree
##                        tysocMjc )

# test_kintree example
add_executable( test_kintree
                test_kintree.cpp )
target_link_libraries( test_kintree
                       tysocMjcCustomViz )

# test custom visualizer example
add_executable( test_visualizer_custom
                test_visualizer_custom.cpp )
target_link_libraries( test_visualizer_custom
                       tysocMjcCustomViz )

# test mujoco visualizer example
add_executable( test_visualizer_mujoco
                test_visualizer_mujoco.cpp )
target_link_libraries( test_visualizer_mujoco
                       tysocMjcMujocoViz )

# test kintree parsed from mjcf
add_executable( test_kintree_mjcf
                test_kintree_mjcf.cpp )
target_link_libraries( test_kintree_mjcf
                       tysocMujoco )