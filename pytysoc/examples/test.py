
import os
import pytysoc

_humanoid   = pytysoc.PyCoreAgent( 'humanoid1', [-2,-2,1], 'mjcf', 'humanoid' )
_walker     = pytysoc.PyCoreAgent( 'walker1', [0,-2,1], 'mjcf', 'walker' )
_baxter     = pytysoc.PyCoreAgent( 'baxter1', [2,-2,1], 'mjcf', 'baxter' )

_laikago    = pytysoc.PyCoreAgent( 'laikago1', [-1,0,1], 'urdf', 'laikago' )
_doublePend = pytysoc.PyCoreAgent( 'doublePend1', [1,0,1], 'urdf', 'double_pendulum' )

_dog        = pytysoc.PyCoreAgent( 'dog1', [-3,2,1], 'rlsim', 'dog3d' )
_raptor     = pytysoc.PyCoreAgent( 'raptor1', [-1,2,1], 'rlsim', 'raptor3d' )
_goat       = pytysoc.PyCoreAgent( 'goat1', [1,2,1], 'rlsim', 'goat3d' )
_biped      = pytysoc.PyCoreAgent( 'biped1', [3,2,1], 'rlsim', 'biped3d' )

_scenario = pytysoc.PyScenario()
_scenario.addAgent( _humanoid )
_scenario.addAgent( _walker )
_scenario.addAgent( _baxter )
_scenario.addAgent( _laikago )
_scenario.addAgent( _doublePend )
_scenario.addAgent( _dog )
_scenario.addAgent( _raptor )
_scenario.addAgent( _goat )
_scenario.addAgent( _biped )

PATH = '/home/gregor/Documents/wilbert/repos/tysocMjc/build/'
_runtime = pytysoc.PyRuntime( PATH + 'libtysocMujoco.so',
                              PATH + 'tysocCustomViz/libtysocCustomViz.so' )

_simulation = _runtime.createSimulation( _scenario )
_simulation.initialize()

_visualizer = _runtime.createVisualizer( _scenario )
_visualizer.initialize()

while _visualizer.isActive() :

    _simulation.step()
    _visualizer.render()