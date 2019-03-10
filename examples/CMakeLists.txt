
message( "Configuring examples" )

find_package( OpenGL REQUIRED )
find_package( GLEW REQUIRED )

include_directories( ${GLEW_INCLUDE_DIRS} )

## # test_kintree example
## add_executable( test_kintree
##                 test_kintree.cpp )
## target_link_libraries( test_kintree
##                        tysocMjc )

## # kintree example
## add_executable( test_kintree
##                 test_kintree.cpp )
## target_link_libraries( test_kintree
##                        tysocMjcCustomViz )
## 
## # custom visualizer example
## add_executable( test_visualizer_custom
##                 test_visualizer_custom.cpp )
## target_link_libraries( test_visualizer_custom
##                        tysocMjcCustomViz )
## 
## # mujoco visualizer example
## add_executable( test_visualizer_mujoco
##                 test_visualizer_mujoco.cpp )
## target_link_libraries( test_visualizer_mujoco
##                        tysocMjcMujocoViz )
## 
## # kintree parsed from mjcf example
## add_executable( test_kintree_mjcf
##                 test_kintree_mjcf.cpp )
## target_link_libraries( test_kintree_mjcf
##                        tysocMujoco )
## 
## # kintree parsed from urdf example
## add_executable( test_kintree_urdf
##                 test_kintree_urdf.cpp )
## target_link_libraries( test_kintree_urdf
##                        tysocMujoco )
## 
## # kintree parsed from rlsim example
## add_executable( test_kintree_rlsim
##                 test_kintree_rlsim.cpp )
## target_link_libraries( test_kintree_rlsim
##                        tysocMujoco )
## 
## # kintree with visualizer and urdf example
## add_executable( test_kintree_urdf_viz
##                 test_kintree_urdf_viz.cpp )
## target_link_libraries( test_kintree_urdf_viz
##                        tysocMjcCustomViz )
## 
## # kintree with visualizer and urdf example
## add_executable( test_kintree_rlsim_viz
##                 test_kintree_rlsim_viz.cpp )
## target_link_libraries( test_kintree_rlsim_viz
##                        tysocMjcCustomViz )
## 
## # kintree with mjc-visualizer and urdf example
## add_executable( test_kintree_urdf_mjcviz
##                 test_kintree_urdf_mjcviz.cpp )
## target_link_libraries( test_kintree_urdf_mjcviz
##                        tysocMjcMujocoViz )
## 

## # visualizer
## add_executable( test_visualizer_only
##                 test_visualizer_only.cpp )
## target_link_libraries( test_visualizer_only
##                        tysocCore )

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