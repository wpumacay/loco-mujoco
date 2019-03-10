
import os
import tysoc_bindings
import pytysoc

import numpy as np

_agent = tysoc_bindings.PyCoreAgent( 'agent0', [0,0,0.75], 'mjcf', 'ant' )
_terrainGen = tysoc_bindings.PyStaticTerrainGen( 'terrainGen0' )
_terrainGen.createPrimitive( 'plane',
                             [10,10,0.1],
                             [0,0,0],
                             [0,0,0],
                             [.2,.3,.4],
                             'chessboard' )

_scenario = tysoc_bindings.PyScenario()
_scenario.addAgent( _agent )
_scenario.addTerrainGen( _terrainGen )

_runtime = pytysoc.createRuntime( physicsBackend = pytysoc.BACKENDS.PHYSICS.MUJOCO,
                                  renderingBackend = pytysoc.BACKENDS.RENDERING.GLVIZ,
                                  workingDir = pytysoc.PATHS.WORKING_DIR )

_simulation = _runtime.createSimulation( _scenario )
_simulation.initialize()

_visualizer = _runtime.createVisualizer( _scenario )
_visualizer.initialize()

_actionDim = _agent.getActionDim()
print( 'actionSpaceDim: ', _actionDim )

while _visualizer.isActive() :

    ## _agent.setActions( -1.0 + 2.0 * np.random.random( _actionDim ) )
    _obsMap = _simulation.getDictOfVectorizedSimData()
    
    _state = np.concatenate( [ _obsMap['qpos'].flat[2:],
                               _obsMap['qvel'].flat,
                               np.clip( _obsMap['comForcesExt'], -1, 1 ).flat ] )
    ## print( 'state.shape: ', _state.shape )
    ## print( 'state: ', _state )

    _simulation.step()
    _visualizer.render()