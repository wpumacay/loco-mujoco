
import numpy as np
from scipy import ndimage
import tysoc_bindings
import pytysoc

NUM_BOXES       = 1
NUM_SPHERES     = 1
NUM_CYLINDERS   = 1
NUM_CAPSULES    = 1
NUM_MESHES      = 1

def createHfield( name, position ) :
    ## nxSamples = 50
    ## nySamples = 50
    ## xExtent = 5.0
    ## yExtent = 5.0

    ## _x = xExtent * np.linspace( -0.5, 0.5, nxSamples )
    ## _y = yExtent * np.linspace( -0.5, 0.5, nySamples )
    ## _xx, _yy = np.meshgrid( _x, _y )
    ## _zz = 10.0 * ( _xx ** 2 + _yy ** 2 ) / ( xExtent ** 2 + yExtent ** 2 )

    # Adapted from dm_control quadruped.py
    #  https://github.com/deepmind/dm_control/blob/master/dm_control/suite/quadruped.py

    nxSamples = 50
    nySamples = 50
    xExtent = 10.0
    yExtent = 10.0

    _TERRAIN_SMOOTHNESS = 0.15  # 0.0: maximally bumpy; 1.0: completely smooth.
    _TERRAIN_BUMP_SCALE = 2  # Spatial scale of terrain bumps (in meters).

    # Sinusoidal bowl shape.
    _xx, _yy = np.ogrid[-1:1:nxSamples*1j, -1:1:nySamples*1j]
    radius = np.clip( np.sqrt( _xx ** 2 + _yy ** 2 ), .04, 1 )
    bowl_shape = .5 - np.cos( 2 * np.pi * radius ) / 2
    # Random smooth bumps.
    terrain_size = 2 * xExtent
    bump_res = int( terrain_size / _TERRAIN_BUMP_SCALE )
    bumps = np.random.uniform( _TERRAIN_SMOOTHNESS, 1, ( bump_res, bump_res ) )
    smooth_bumps = ndimage.zoom( bumps, nxSamples / float( bump_res ) )
    # Terrain is elementwise product.
    _zz = bowl_shape * smooth_bumps

    _maxHeight = np.max( _zz )
    _heightData = ( _zz / _maxHeight ).ravel()

    _heightFieldData = tysoc_bindings.PyHeightFieldData()
    _heightFieldData.nWidthSamples = nxSamples
    _heightFieldData.nDepthSamples = nySamples
    _heightFieldData.heightData = _heightData

    _collisionData = tysoc_bindings.PyCollisionData()
    _collisionData.type = tysoc_bindings.eShapeType.HFIELD
    _collisionData.size = [ xExtent, yExtent, _maxHeight ]
    _collisionData.hdata = _heightFieldData

    _visualData = tysoc_bindings.PyVisualData()
    _visualData.type = tysoc_bindings.eShapeType.HFIELD
    _visualData.size = [ xExtent, yExtent, _maxHeight ]
    _visualData.hdata = _heightFieldData

    _visualData.ambient     = [ 0.2, 0.3, 0.4 ]
    _visualData.diffuse     = [ 0.2, 0.3, 0.4 ]
    _visualData.specular    = [ 0.2, 0.3, 0.4 ]
    _visualData.shininess   = 50.0

    _bodyData = tysoc_bindings.PyBodyData()
    _bodyData.dyntype = tysoc_bindings.eDynamicsType.STATIC
    _bodyData.hasInertia = False
    _bodyData.addCollision( _collisionData )
    _bodyData.addVisual( _visualData )

    _body = tysoc_bindings.PyBody( name, 
                                   _bodyData, 
                                   position, 
                                   [ 0.0, 0.0, 0.0 ] )

    return _body

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

    elif shape == 'mesh' :
        _collisionData.type = tysoc_bindings.eShapeType.MESH
        _collisionData.size = [ 0.2, 0.2, 0.2 ]
        _collisionData.filename = '/home/gregor/Documents/repos/tysoc_mujoco_workspace/tysoc_mujoco/core/res/meshes/monkey.stl'

        _visualData.type = tysoc_bindings.eShapeType.MESH
        _visualData.size = [ 0.2, 0.2, 0.2 ]
        _visualData.filename = '/home/gregor/Documents/repos/tysoc_mujoco_workspace/tysoc_mujoco/core/res/meshes/monkey.stl'

    _visualData.ambient = [ 0.7, 0.5, 0.3 ]
    _visualData.diffuse = [ 0.7, 0.5, 0.3 ]
    _visualData.specular = [ 0.7, 0.5, 0.3 ]
    _visualData.shininess = 50.0

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
    _scenario = tysoc_bindings.PyScenario()
    
    _hfield = createHfield( "terrain_0", [ 0.0, 0.0, 0.0 ] )
    _scenario.addBody( _hfield )
    
    for i in range( NUM_BOXES ) :
        _scenario.addBody( createSingleBody( 'box_' + str( i ), 'box' ) )

    for i in range( NUM_SPHERES ) :
        _scenario.addBody( createSingleBody( 'sphere_' + str( i ), 'sphere' ) )

    for i in range( NUM_CYLINDERS ) :
        _scenario.addBody( createSingleBody( 'cylinder_' + str( i ), 'cylinder' ) )

    for i in range( NUM_CAPSULES ) :
        _scenario.addBody( createSingleBody( 'capsule_' + str( i ), 'capsule' ) )

    for i in range( NUM_MESHES ) :
        _scenario.addBody( createSingleBody( 'mesh_' + str( i ), 'mesh' ) )
    
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
    
        if _visualizer.checkSingleKeyPress( 80 ) :
            _running = not _running

        if _visualizer.checkSingleKeyPress( 256 ) :
            break

        if _visualizer.checkSingleKeyPress( 82 ) :
            _simulation.reset()

        if _running :
            _simulation.step()
    
        _visualizer.render()