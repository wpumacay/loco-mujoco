
import numpy as np
import tysoc_bindings as tysoc
import pytysoc
import time

def fromPositionAndRotation( position, rotation ) :
    transform = np.identity( 4 )
    transform[:3,:3] = rotation
    transform[:3,3] = position
    return transform

def createDoorVersion1( name, position ) :
    _compound = tysoc.PyCompound( name, position, [0.1,0.2,0.3], tysoc.eDynamicsType.DYNAMIC )

    ## create compound bodies separately, compose them and add to the compound afterwards

    _frame_0_coldata = tysoc.PyCollisionData()
    _frame_0_coldata.type = tysoc.eShapeType.BOX
    _frame_0_coldata.size = [ 0.2, 0.2, 2.0 ]
    _frame_0_visdata = tysoc.PyVisualData()
    _frame_0_visdata.type = tysoc.eShapeType.BOX
    _frame_0_visdata.size = [ 0.2, 0.2, 2.0 ]
    _frame_0_visdata.ambient = [ 0.4, 0.3, 0.2 ]
    _frame_0_visdata.diffuse = [ 0.4, 0.3, 0.2 ]
    _frame_0_visdata.specular = [ 0.4, 0.3, 0.2 ]
    _frame_0_visdata.shininess = 50.0
    _frame_0_body_data = tysoc.PyBodyData()
    _frame_0_body_data.collision = _frame_0_coldata
    _frame_0_body_data.visual = _frame_0_visdata
    _frame_0_local_tf = fromPositionAndRotation( [ 0.0, -0.5, 1.0 ], np.identity(3) )
    _frame_0_body = _compound.createRootBody( "frame_0", 
                                              _frame_0_body_data,
                                              _frame_0_local_tf )

    _frame_1_coldata = tysoc.PyCollisionData()
    _frame_1_coldata.type = tysoc.eShapeType.BOX
    _frame_1_coldata.size = [ 0.2, 0.8, 0.2 ]
    _frame_1_visdata = tysoc.PyVisualData()
    _frame_1_visdata.type = tysoc.eShapeType.BOX
    _frame_1_visdata.size = [ 0.2, 0.8, 0.2 ]
    _frame_1_visdata.ambient = [ 0.4, 0.3, 0.2 ]
    _frame_1_visdata.diffuse = [ 0.4, 0.3, 0.2 ]
    _frame_1_visdata.specular = [ 0.4, 0.3, 0.2 ]
    _frame_1_visdata.shininess = 50.0
    _frame_1_body_data = tysoc.PyBodyData()
    _frame_1_body_data.collision = _frame_1_coldata
    _frame_1_body_data.visual = _frame_1_visdata
    _frame_1_joint_data = tysoc.PyJointData()
    _frame_1_joint_data.type = tysoc.eJointType.FIXED
    _frame_1_joint_data.localTransform = np.identity( 4 ) ## identity (same ref-frame as body's frame)
    _frame_1_local_tf = fromPositionAndRotation( [ 0.0, 0.5, 0.9 ], np.identity(3) )
    _frame_1_body = tysoc.PyCompoundBody( "frame_1",
                                          _frame_1_body_data,
                                          _frame_1_joint_data,
                                          _frame_0_body,
                                          _frame_1_local_tf[:3,3],
                                          _frame_1_local_tf[:3,:3] )

    _frame_2_coldata = tysoc.PyCollisionData()
    _frame_2_coldata.type = tysoc.eShapeType.BOX
    _frame_2_coldata.size = [ 0.2, 0.2, 2.0 ]
    _frame_2_visdata = tysoc.PyVisualData()
    _frame_2_visdata.type = tysoc.eShapeType.BOX
    _frame_2_visdata.size = [ 0.2, 0.2, 2.0 ]
    _frame_2_visdata.ambient = [ 0.4, 0.3, 0.2 ]
    _frame_2_visdata.diffuse = [ 0.4, 0.3, 0.2 ]
    _frame_2_visdata.specular = [ 0.4, 0.3, 0.2 ]
    _frame_2_visdata.shininess = 50.0
    _frame_2_body_data = tysoc.PyBodyData()
    _frame_2_body_data.collision = _frame_2_coldata
    _frame_2_body_data.visual = _frame_2_visdata
    _frame_2_joint_data = tysoc.PyJointData()
    _frame_2_joint_data.type = tysoc.eJointType.FIXED
    _frame_2_joint_data.localTransform = np.identity( 4 ) ## identity (same ref-frame as body's frame)
    _frame_2_local_tf = fromPositionAndRotation( [ 0.0, 0.5, -0.9 ], np.identity(3) )
    _frame_2_body = tysoc.PyCompoundBody( "frame_2",
                                          _frame_2_body_data,
                                          _frame_2_joint_data,
                                          _frame_1_body,
                                          _frame_2_local_tf[:3,3],
                                          _frame_2_local_tf[:3,:3] )

    _panel_coldata = tysoc.PyCollisionData()
    _panel_coldata.type = tysoc.eShapeType.BOX
    _panel_coldata.size = [ 1.2, 0.1, 2.0 ]
    _panel_visdata = tysoc.PyVisualData()
    _panel_visdata.type = tysoc.eShapeType.BOX
    _panel_visdata.size = [ 1.2, 0.1, 2.0 ]
    _panel_visdata.ambient = [ 0.4, 0.3, 0.5 ]
    _panel_visdata.diffuse = [ 0.4, 0.3, 0.5 ]
    _panel_visdata.specular = [ 0.4, 0.3, 0.5 ]
    _panel_visdata.shininess = 50.0
    _panel_body_data = tysoc.PyBodyData()
    _panel_body_data.collision = _panel_coldata
    _panel_body_data.visual = _panel_visdata
    _panel_joint_data = tysoc.PyJointData()
    _panel_joint_data.type = tysoc.eJointType.REVOLUTE
    _panel_joint_data.axis = [ 0.0, 0.0, 1.0 ]
    _panel_joint_data.limits = [ -0.5 * np.pi, 0.5 * np.pi ]
    _panel_joint_data.localTransform = fromPositionAndRotation( [ -0.6, -0.05, 0.0 ], np.identity(3) )
    _panel_local_tf = fromPositionAndRotation( [ 0.7, -0.15, 0.0 ], np.identity(3) )
    _panel_body = tysoc.PyCompoundBody( "panel",
                                        _panel_body_data,
                                        _panel_joint_data,
                                        _frame_0_body,
                                        _panel_local_tf[:3,3],
                                        _panel_local_tf[:3,:3] )

    ## add all bodies (the compound is just a handy container)
    _compound.addCompoundBody( _frame_1_body )
    _compound.addCompoundBody( _frame_2_body )
    _compound.addCompoundBody( _panel_body )

    ##**********************************************************************************************

    return _compound

if __name__ == '__main__' :
    _terrainGen = tysoc.PyStaticTerrainGen( 'terrainGen0' ) 
    _terrainGen.createPrimitive( 'plane',
                                 [ 10.0, 10.0, 0.2 ],
                                 [ 0.0, 0.0, 0.0 ],
                                 [ 0.0, 0.0, 0.0 ],
                                 [ 0.2, 0.3, 0.4 ],
                                 'built_in_chessboard' )

    _scenario = tysoc.PyScenario()
    _scenario.addTerrainGen( _terrainGen )

    _door = createDoorVersion1( 'door_0', [0.0, 0.0, 1.0] )
    _scenario.addCompound( _door )

    _runtime = pytysoc.createRuntime( physicsBackend = pytysoc.BACKENDS.PHYSICS.MUJOCO,
                                      renderingBackend = pytysoc.BACKENDS.RENDERING.GLVIZ )

    _simulation = _runtime.createSimulation( _scenario )
    _visualizer = _runtime.createVisualizer( _scenario )

    _simulation.initialize()
    _visualizer.initialize()

    _simulation.step()
    _visualizer.render()

    _running = False

    while _visualizer.isActive() :

        if _visualizer.checkSingleKeyPress( tysoc.KEY_P ) :
            _running = not _running
        elif _visualizer.checkSingleKeyPress( tysoc.KEY_R ) :
            _simulation.reset()
        elif _visualizer.checkSingleKeyPress( tysoc.KEY_ESCAPE ) :
            break

        start = time.time()
        if _running :
            _simulation.step()

        _visualizer.render()

        duration = time.time() - start
        print( 'step-time: {} ||| fps: {}'.format( duration, 1.0 / duration ) )