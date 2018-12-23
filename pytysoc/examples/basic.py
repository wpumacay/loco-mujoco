
"""
    Basic sample: We just create every component as needed and ...
    assemble a simple scenario programatically
"""

import time
import numpy as np

import pytysoc

## @WIP
## _runtime = pytysoc.createRuntime( physicsBackend = 'bullet', 
##                                   renderingBackend = 'custom',
##                                   renderingMode = 'interactive' )

## @WIP
## _runtime = pytysoc.createRuntime( physicsBackend = 'bullet', 
##                                   renderingBackend = 'custom',
##                                   renderingMode = 'headless' )

## @WIP
## _runtime = pytysoc.createRuntime( physicsBackend = 'bullet', 
##                                   renderingBackend = 'ogre',
##                                   renderingMode = 'interactive' )

## @WIP
## _runtime = pytysoc.createRuntime( physicsBackend = 'flex', 
##                                   renderingBackend = 'custom',
##                                   renderingMode = 'interactive' )

## @AVAILABLE
## _runtime = pytysoc.createRuntime( physicsBackend = 'mujoco', 
##                                   renderingBackend = 'mujocoviz',
##                                   renderingMode = 'interactive' )

## @WIP
## _runtime = pytysoc.createRuntime( physicsBackend = 'mujoco', 
##                                   renderingBackend = 'custom',
##                                   renderingMode = 'headless' )

## . . .

_runtime = pytysoc.createRuntime( physicsBackend = 'mujoco', 
                                  renderingBackend = 'custom',
                                  renderingMode = 'interactive' )

_humanoid           = _runtime.createAgent( pytysoc.agents.HUMANOID )
_terrainGenerator   = _runtime.createTerrainGenerator( pytysoc.terrain.PROCEDURAL_BLOCKY,
                                                       configParams = { 'sectionDepth' : 1.0 } )
## _terrainGenerator   = _runtime.createTerrainGenerator( pytysoc.terrain.PROCEDURAL_BLOCKY,
##                                                        configFile = 'procedural_1.json' )

_scenario = _runtime.createScenario( pytysoc.scenario.SIMPLE )
_scenario.addAgent( _humanoid )
_scenario.addTerrainGenerator( _terrainGenerator )

_runtime.setScenario( _scenario )

_visualizer = _runtime.createVisualizer()

_time           = 0.0
_timeDelta      = 0.0
_timeStampBef   = time.time()
_timeStampNow   = time.time()

while ( _timeNow < 60.0 ) :

    # random policy
    _actions = np.random.random( _humanoid.getActionShape() )
    _humanoid.setActions( _actions )

    # update the simulation
    _runtime.step()

    # and the visualizer
    _visualizer.update()

    _timeStampNow   = time.time()
    _timeDelta      = _timeStampNow - _timeStampBef
    _timeStampBef   = _timeStampNow

    _time += _timeDelta

