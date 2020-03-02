
import os
import pytysoc
import tysoc_bindings

_humanoid   = tysoc_bindings.PyCoreAgent( 'humanoid1', [-2,-2,3], 'mjcf', 'humanoid' )
_walker     = tysoc_bindings.PyCoreAgent( 'walker1', [0,-2,3], 'mjcf', 'walker' )
_ant        = tysoc_bindings.PyCoreAgent( 'ant1', [2,-2,3], 'mjcf', 'ant' )

_laikago    = tysoc_bindings.PyCoreAgent( 'laikago1', [-1,0,3], 'urdf', 'laikago' )
_dogbot     = tysoc_bindings.PyCoreAgent( 'dogbot1', [1,0,3], 'urdf', 'dogbot' )

## _dog3d      = tysoc_bindings.PyCoreAgent( 'dog1', [-3,2,3], 'rlsim', 'dog3d' )
## _raptor3d   = tysoc_bindings.PyCoreAgent( 'raptor1', [-1,2,3], 'rlsim', 'raptor3d' )
## _goat3d     = tysoc_bindings.PyCoreAgent( 'goat1', [1,2,3], 'rlsim', 'goat3d' )
_biped3d    = tysoc_bindings.PyCoreAgent( 'biped1', [3,2,3], 'rlsim', 'biped3d' )
_humanoid3d = tysoc_bindings.PyCoreAgent( 'biped1', [3,2,3], 'rlsim', 'humanoid3d' )

_scenario = tysoc_bindings.PyScenario()
_scenario.addAgent( _humanoid )
_scenario.addAgent( _walker )
_scenario.addAgent( _ant )
_scenario.addAgent( _laikago )
_scenario.addAgent( _dogbot )
## _scenario.addAgent( _dog3d )
## _scenario.addAgent( _raptor3d )
## _scenario.addAgent( _goat3d )
_scenario.addAgent( _biped3d )
_scenario.addAgent( _humanoid3d )

_runtime = pytysoc.createRuntime( physicsBackend = pytysoc.BACKENDS.PHYSICS.MUJOCO,
                                  renderingBackend = pytysoc.BACKENDS.RENDERING.GLVIZ )

_simulation = _runtime.createSimulation( _scenario )
_simulation.initialize()

_visualizer = _runtime.createVisualizer( _scenario )
_visualizer.initialize()

while _visualizer.isActive() :

    if _visualizer.checkSingleKeyPress( tysoc_bindings.KEY_P ) :
        _running = not _running
    elif _visualizer.checkSingleKeyPress( tysoc_bindings.KEY_R ) :
        _simulation.reset()
    elif _visualizer.checkSingleKeyPress( tysoc_bindings.KEY_ESCAPE ) :
        break

    _simulation.step()
    _visualizer.render()