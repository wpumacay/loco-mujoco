#!/usr/bin/env python

import loco
import tinymath as tm
import numpy as np
import gc

def test_runtime_mujoco_backend() :
    vis_data = loco.sim.VisualData()
    vis_data.type = loco.sim.ShapeType.CAPSULE
    vis_data.size = [ 0.1, 0.2, 0.1 ]
    col_data = loco.sim.CollisionData()
    col_data.type = loco.sim.ShapeType.CAPSULE
    col_data.size = [ 0.1, 0.2, 0.1 ]

    body_data = loco.sim.BodyData()
    body_data.dyntype = loco.sim.DynamicsType.DYNAMIC
    body_data.collision = col_data
    body_data.visual = vis_data

    body_obj = loco.sim.SingleBody( 'body_0', body_data, [ 1.0, 1.0, 1.0 ], np.identity( 3 ) )
    scenario = loco.sim.Scenario()
    scenario.AddSingleBody( body_obj )

    runtime = loco.sim.Runtime( loco.sim.PHYSICS_MUJOCO, loco.sim.RENDERING_NONE )

    simulation = runtime.CreateSimulation( scenario )
    simulation.Step()
    simulation.Reset()
    simulation.Pause()
    simulation.Resume()
    assert ( simulation.backendId == 'MUJOCO' )

    visualizer = runtime.CreateVisualizer( scenario )
    camera = visualizer.CreateCamera( 'cam_orbit_0',
                                      loco.sim.VizCameraType.ORBIT,
                                      [ 3.0, 3.0, 3.0 ],
                                      [ 0.0, 0.0, 0.0 ] )
    light = visualizer.CreateLight( 'light_point_0',
                                    loco.sim.VizLightType.POINT,
                                    [ 0.4, 0.4, 0.4 ],
                                    [ 0.8, 0.8, 0.8 ],
                                    [ 0.8, 0.8, 0.8 ] )
    visualizer.Update()
    visualizer.Reset()
    assert ( visualizer.backendId == "null" )
    assert ( visualizer.HasCameraNamed( "cam_orbit_0" ) == True )
    assert ( visualizer.HasLightNamed( "light_point_0" ) == True )
    assert ( visualizer.GetCameraByName( "cam_orbit_0" ) != None )
    assert ( visualizer.GetLightByName( "light_point_0" ) != None )
    assert ( np.allclose( visualizer.GetCameraByName( 'cam_orbit_0' ).position, camera.position ) )
    assert ( np.allclose( visualizer.GetLightByName( 'light_point_0' ).ambient, light.ambient ) )

    runtime.DestroySimulation()
    runtime.DestroyVisualizer()

if __name__ == '__main__' :
    _ = input( 'Press ENTER to start test : test_runtime_mujoco_backend' )
    test_runtime_mujoco_backend()

    _ = input( 'Press ENTER to continue ...' )