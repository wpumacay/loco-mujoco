
import os
import tysoc_bindings
import pytysoc

import numpy as np

_agent = tysoc_bindings.PyCoreAgent( 'agent0', [0,0,2], 'mjcf', 'ant' )
_terrainGen = tysoc_bindings.PyStaticTerrainGen( 'terrainGen0' )
_terrainGen.createPrimitive( 'box',
                             [10,10,0.1],
                             [0,0,-0.05],
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

while True :

    _simulation.step()
    _visualizer.render()

    _agent.setActions( -1.0 + 2.0 * np.random.random( _actionDim ) )