
import os
import tysoc_bindings
import pytysoc

import numpy as np

_agent1 = tysoc_bindings.PyCoreAgent( 'agent1', [0,0,1.25], 'mjcf', 'cheetah' )
_agent2 = tysoc_bindings.PyCoreAgent( 'agent2', [-2,0,1.25], 'mjcf', 'walker' )
_agent3 = tysoc_bindings.PyCoreAgent( 'agent3', [0,2,1.25], 'mjcf', 'ant' )
_agent4 = tysoc_bindings.PyCoreAgent( 'agent4', [2,0,1.25], 'mjcf', 'hopper' )
_agent5 = tysoc_bindings.PyCoreAgent( 'agent5', [0,-2,1.25], 'mjcf', 'humanoid' )
_terrainGen = tysoc_bindings.PyStaticTerrainGen( 'terrainGen0' )
_terrainGen.createPrimitive( 'box',
                             [20,20,0.1],
                             [0,0,-0.05],
                             [0,0,0],
                             [.2,.3,.4],
                             'chessboard' )

_scenario = tysoc_bindings.PyScenario()
_scenario.addAgent( _agent1 )
_scenario.addAgent( _agent2 )
_scenario.addAgent( _agent3 )
_scenario.addAgent( _agent4 )
_scenario.addAgent( _agent5 )
_scenario.addTerrainGen( _terrainGen )

_runtime = pytysoc.createRuntime( physicsBackend = pytysoc.BACKENDS.PHYSICS.MUJOCO,
                                  renderingBackend = pytysoc.BACKENDS.RENDERING.GLVIZ,
                                  workingDir = pytysoc.PATHS.WORKING_DIR )

_simulation = _runtime.createSimulation( _scenario )
_simulation.initialize()

_visualizer = _runtime.createVisualizer( _scenario )
_visualizer.initialize()

## _actionDim = _agent.getActionDim()
## print( 'actionSpaceDim: ', _actionDim )

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
