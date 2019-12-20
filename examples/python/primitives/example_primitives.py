
import numpy as np
import tysoc_bindings as tysoc
import pytysoc
import time

NUM_BOXES = 5
NUM_SPHERES = 5
NUM_CYLINDERS = 5
NUM_CAPSULES = 5
NUM_MESHES = 0

def createSingleBody( name, shape ) :
    _collisionData = tysoc.PyCollisionData()
    _visualData = tysoc.PyVisualData()
    _bodyData = tysoc.PyBodyData()

    if shape == 'box' :
        _collisionData.type = tysoc.eShapeType.BOX
        _collisionData.size = [ 0.2, 0.2, 0.2 ]

        _visualData.type = tysoc.eShapeType.BOX
        _visualData.size = [ 0.2, 0.2, 0.2 ]

    elif shape == 'sphere' :
        _collisionData.type = tysoc.eShapeType.SPHERE
        _collisionData.size = [ 0.1, 0.1, 0.1 ]

        _visualData.type = tysoc.eShapeType.SPHERE
        _visualData.size = [ 0.1, 0.1, 0.1 ]

    elif shape == 'cylinder' :
        _collisionData.type = tysoc.eShapeType.CYLINDER
        _collisionData.size = [ 0.1, 0.2, 0.1 ]

        _visualData.type = tysoc.eShapeType.CYLINDER
        _visualData.size = [ 0.1, 0.2, 0.1 ]

    elif shape == 'capsule' :
        _collisionData.type = tysoc.eShapeType.CAPSULE
        _collisionData.size = [ 0.1, 0.2, 0.1 ]

        _visualData.type = tysoc.eShapeType.CAPSULE
        _visualData.size = [ 0.1, 0.2, 0.1 ]

    elif shape == 'mesh' :
        _collisionData.type = tysoc.eShapeType.MESH
        _collisionData.size = [ 0.2, 0.2, 0.2 ]
        _collisionData.filename = pytysoc.PATHS.RESOURCES_DIR + "meshes/monkey.stl"

        _visualData.type = tysoc.eShapeType.MESH
        _visualData.size = [ 0.2, 0.2, 0.2 ]
        _visualData.filename = pytysoc.PATHS.RESOURCES_DIR + "meshes/monkey.stl"

    _visualData.ambient = [ 0.7, 0.5, 0.3 ]
    _visualData.diffuse = [ 0.7, 0.5, 0.3 ]
    _visualData.specular = [ 0.7, 0.5, 0.3 ]
    _visualData.shininess = 50.0;

    _bodyData = tysoc.PyBodyData()
    _bodyData.dyntype = tysoc.eDynamicsType.DYNAMIC
    _bodyData.collision = _collisionData
    _bodyData.visual = _visualData
    
    _position = 4.0 * ( np.random.rand( 3 ) - 0.5 )
    _position[2] = 3.0
    _rotation = np.pi * ( np.random.rand( 3 ) - 0.5 )

    _body = tysoc.PySingleBody( name,
                                _bodyData,
                                _position,
                                _rotation )

    return _body

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
    
    for i in range( NUM_BOXES ) :
        _scenario.addSingleBody( createSingleBody( 'box_' + str( i ), 'box' ) )

    for i in range( NUM_SPHERES ) :
        _scenario.addSingleBody( createSingleBody( 'sphere_' + str( i ), 'sphere' ) )

    for i in range( NUM_CYLINDERS ) :
        _scenario.addSingleBody( createSingleBody( 'cylinder_' + str( i ), 'cylinder' ) )

    for i in range( NUM_CAPSULES ) :
        _scenario.addSingleBody( createSingleBody( 'capsule_' + str( i ), 'capsule' ) )

    for i in range( NUM_MESHES ) :
        _scenario.addSingleBody( createSingleBody( 'mesh_' + str( i ), 'mesh' ) )
    
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
        print( "step-time: {} ||| fps: {}".format( duration, 1.0 / duration ) )