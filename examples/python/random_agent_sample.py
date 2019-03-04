
import os
import tysoc_bindings
import pytysoc

import numpy as np

_agent = tysoc_bindings.PyCoreAgent( 'agent0', [0,0,-2], 'mjcf', 'ant' )

_scenario = tysoc_bindings.PyScenario()
_scenario.addAgent( _agent )

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

    _agent.setActions( -1.0 + 2.0 * np.random.random( _actionDim ) )

    _simulation.step()
    _visualizer.render()