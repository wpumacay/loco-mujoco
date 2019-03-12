
import os
import tysoc_bindings
import pytysoc

import numpy as np
import tensorflow as tf
import tf_util
import load_policy

_policyFn = load_policy.load_policy( 'experts/Ant-v2.pkl' )

_agent = tysoc_bindings.PyCoreAgent( 'walker1', [0,0,0.75], 'mjcf', 'ant' )
_terrainGen = tysoc_bindings.PyStaticTerrainGen( 'terrainGen0' ) 
_terrainGen.createPrimitive( 'plane',
                             [200,10,0.1],
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

_running = False

with tf.Session() :
    tf_util.initialize()

    while _visualizer.isActive() :

        # press key P to start simulation (sorry, forgot to map keys in python)
        if _visualizer.checkSingleKeyPress( 15 ) :
            _running = not _running

        if _running :
            _obsMap = _simulation.getDictOfVectorizedSimData()
            _observation = np.concatenate( [ _obsMap['qpos'].flat[2:],
                                             _obsMap['qvel'].flat,
                                             np.clip( _obsMap['comForcesExt'], -1, 1 ).flat ] )
    
            _action = _policyFn( _observation[None,:] )
            _agent.setActions( _action )
    
            _simulation.step()

        _visualizer.render()
