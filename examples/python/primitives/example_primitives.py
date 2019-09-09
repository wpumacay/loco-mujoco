
import numpy as np
import tysoc_bindings
import pytysoc

def createSingleBody( name, shape ) :
    _collisionData = tysoc_bindings.PyCollisionData()
    _visualData = tysoc_bindings.PyVisualData()
    _bodyData = tysoc_bindings.PyBodyData()

    if shape == 'box' :
        _collisionData.type = tysoc_bindings.eShapeType.BOX
        _collisionData.size = [ 0.2, 0.2, 0.2 ]

        _visualData.type = tysoc_bindings.eShapeType.BOX
        _visualData.size = [ 0.2, 0.2, 0.2 ]

    elif shape == 'sphere' :
        _collisionData.type = tysoc_bindings.eShapeType.SPHERE
        _collisionData.size = [ 0.1, 0.1, 0.1 ]

        _visualData.type = tysoc_bindings.eShapeType.SPHERE
        _visualData.size = [ 0.1, 0.1, 0.1 ]

    elif shape == 'cylinder' :
        _collisionData.type = tysoc_bindings.eShapeType.CYLINDER
        _collisionData.size = [ 0.1, 0.2, 0.1 ]

        _visualData.type = tysoc_bindings.eShapeType.CYLINDER
        _visualData.size = [ 0.1, 0.2, 0.1 ]

    elif shape == 'capsule' :
        _collisionData.type = tysoc_bindings.eShapeType.CAPSULE
        _collisionData.size = [ 0.1, 0.2, 0.1 ]

        _visualData.type = tysoc_bindings.eShapeType.CAPSULE
        _visualData.size = [ 0.1, 0.2, 0.1 ]

    _visualData.ambient = [ 0.7, 0.5, 0.3 ]
    _visualData.diffuse = [ 0.7, 0.5, 0.3 ]
    _visualData.specular = [ 0.7, 0.5, 0.3 ]
    _visualData.shininess = 50.0;

    _bodyData = tysoc_bindings.PyBodyData()
    _bodyData.dyntype = tysoc_bindings.eDynamicsType.DYNAMIC
    _bodyData.hasInertia = False
    _bodyData.addCollision( _collisionData )
    _bodyData.addVisual( _visualData )
    
    _position = 4.0 * ( np.random.rand( 3 ) - 0.5 )
    _position[2] = 3.0
    _rotation = np.pi * ( np.random.rand( 3 ) - 0.5 )

    _body = tysoc_bindings.PyBody( name,
                                   _bodyData,
                                   _position,
                                   _rotation )

    return _body

if __name__ == '__main__' :
    _terrainGen = tysoc_bindings.PyStaticTerrainGen( 'terrainGen0' ) 
    _terrainGen.createPrimitive( 'box',
                                 [ 10.0, 10.0, 0.2 ],
                                 [ 0.0, 0.0, 0.0 ],
                                 [ 0.0, 0.0, 0.0 ],
                                 [ 0.2, 0.3, 0.4 ],
                                 'chessboard' )
    
    _scenario = tysoc_bindings.PyScenario()
    _scenario.addTerrainGen( _terrainGen )
    
    for i in range( 5 ) :
        _scenario.addBody( createSingleBody( 'box_' + str( i ), 'box' ) )

    for i in range( 5 ) :
        _scenario.addBody( createSingleBody( 'sphere_' + str( i ), 'sphere' ) )

    for i in range( 5 ) :
        _scenario.addBody( createSingleBody( 'cylinder_' + str( i ), 'cylinder' ) )

    for i in range( 5 ) :
        _scenario.addBody( createSingleBody( 'capsule_' + str( i ), 'capsule' ) )
    
    _runtime = pytysoc.createRuntime( physicsBackend = pytysoc.BACKENDS.PHYSICS.MUJOCO,
                                      renderingBackend = pytysoc.BACKENDS.RENDERING.GLVIZ,
                                      workingDir = pytysoc.PATHS.WORKING_DIR )
    
    _simulation = _runtime.createSimulation( _scenario )
    _visualizer = _runtime.createVisualizer( _scenario )
    
    _simulation.initialize()
    _visualizer.initialize()
    
    _simulation.step()
    _visualizer.render()

    _running = False
    
    while _visualizer.isActive() :
    
        if _visualizer.checkSingleKeyPress( 15 ) :
            _running = not _running

        if _visualizer.checkSingleKeyPress( 26 ) :
            break

        if _visualizer.checkSingleKeyPress( 17 ) :
            _simulation.reset()

        if _running :
            _simulation.step()
    
        _visualizer.render()