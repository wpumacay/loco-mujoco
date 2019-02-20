
import os
import pytysoc

#_humanoid = pytysoc.PyCoreAgent( 'humanoid1', [0,0,1], 'mjcf', 'humanoid' )
_walker   = pytysoc.PyCoreAgent( 'walker1', [-2,0,1], 'mjcf', 'walker' )
#_dog      = pytysoc.PyCoreAgent( 'dog1', [2,0,1], 'rlsim', 'dog3d' )

_scenario = pytysoc.PyScenario()
#_scenario.addAgent( _humanoid )
_scenario.addAgent( _walker )
#_scenario.addAgent( _dog )

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