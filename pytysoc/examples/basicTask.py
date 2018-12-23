
"""
    Basic task sample: Here we create a task which contains a group of ...
    agent(s), terrain generator(s) and sensor(s). 
"""

import time
import numpy as np

import pytysoc

_runtime = pytysoc.createRuntime( physicsBackend = 'mujoco', 
                                  renderingBackend = 'custom',
                                  renderingMode = 'interactive',
                                  renderingConfigParams = {} )

_task = _runtime.createTask( taskType = pytysoc.tasks.SINGLE_WALK_FORWARD,
                             agentType = pytysoc.agents.HUMANOID,
                             terrainType = pytysoc.terrain.SIMPLE_PROCEDURAL_FORWARD,
                             terrainConfigParams = { 'sectionDepth' : 1.0 },
                             agentSensors = [ pytysoc.sensors.INTRINSIC, 
                                              pytysoc.sensors.TERRAIN_PROFILE,
                                              pytysoc.sensors.FRONT_CAMERA ] )

## _task = _runtime.createTask( taskType = pytysoc.tasks.SINGLE_WALK_FORWARD,
##                              agentType = pytysoc.agents.HUMANOID,
##                              terrainType = pytysoc.terrain.SIMPLE_PROCEDURAL_FORWARD,
##                              terrainConfigFile = 'procedural_forward.json',
##                              agentSensors = [ pytysoc.sensors.INTRINSIC, 
##                                               pytysoc.sensors.TERRAIN_PROFILE,
##                                               pytysoc.sensors.FRONT_CAMERA ] )

_visualizer = _runtime.createVisualizer()

_time           = 0.0
_timeDelta      = 0.0
_timeStampBef   = time.time()
_timeStampNow   = time.time()

_agents = _task.getAgents()

while ( _timeNow < 60.0 ) :

    # we have just one agent here, but could be multiagent
    _actions = np.random.random( _agents[0].getActionShape() )
    _agents[0].setActions( _actions ) 

    # update the simulation
    _runtime.step()

    # and the visualizer
    _visualizer.update()

    _timeStampNow   = time.time()
    _timeDelta      = _timeStampNow - _timeStampBef
    _timeStampBef   = _timeStampNow

    _time += _timeDelta

