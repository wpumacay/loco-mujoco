
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

## # test_kintree example
## add_executable( test_kintree
##                 test_kintree.cpp )
## target_link_libraries( test_kintree
##                        tysocMjc )

# kintree example
add_executable( test_kintree
                test_kintree.cpp )
target_link_libraries( test_kintree
                       tysocMjcCustomViz )

# custom visualizer example
add_executable( test_visualizer_custom
                test_visualizer_custom.cpp )
target_link_libraries( test_visualizer_custom
                       tysocMjcCustomViz )

# mujoco visualizer example
add_executable( test_visualizer_mujoco
                test_visualizer_mujoco.cpp )
target_link_libraries( test_visualizer_mujoco
                       tysocMjcMujocoViz )

# kintree parsed from mjcf example
add_executable( test_kintree_mjcf
                test_kintree_mjcf.cpp )
target_link_libraries( test_kintree_mjcf
                       tysocMujoco )

# kintree parsed from urdf example
add_executable( test_kintree_urdf
                test_kintree_urdf.cpp )
target_link_libraries( test_kintree_urdf
                       tysocMujoco )

# kintree with visualizer and urdf example
add_executable( test_kintree_urdf_viz
                test_kintree_urdf_viz.cpp )
target_link_libraries( test_kintree_urdf_viz
                       tysocMjcCustomViz )

# kintree with mjc-visualizer and urdf example
add_executable( test_kintree_urdf_mjcviz
                test_kintree_urdf_mjcviz.cpp )
target_link_libraries( test_kintree_urdf_mjcviz
                       tysocMjcMujocoViz )