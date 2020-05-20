#!/usr/bin/env python

import sys
import loco
import tinymath as tm
import numpy as np

class Environment(object):

    def __init__(self,
                 physics_backend=loco.sim.PHYSICS_MUJOCO,
                 graphics_backend=loco.sim.RENDERING_GLVIZ_GLFW,
                 time_step=1./60.):
        self._physics_backend = physics_backend
        self._graphics_backend = graphics_backend
        self._scenario = loco.sim.Scenario()
        self._runtime = None
        self._visualizer = None
        self._simulation = None
        self._time_step = time_step

    def build(self):
        self._runtime = loco.sim.Runtime(self._physics_backend, self._graphics_backend)
        self._simulation = self._runtime.CreateSimulation(self._scenario)
        self._visualizer = self._runtime.CreateVisualizer(self._scenario)

    def reset(self):
        assert self._simulation != None, 'Environment::reset >>> simulation is required, but got None'
        assert self._visualizer != None, 'Environment::reset >>> simulation is required, but got None'
        self._simulation.Reset()
        self._visualizer.Reset()

    def step(self):
        assert self._simulation != None, 'Environment::step >>> simulation is required, but got None'
        self._simulation.Step(self._time_step)

    def render(self):
        assert self._visualizer != None, 'Environment::render >>> simulation is required, but got None'
        self._visualizer.Render(loco.sim.RenderMode.NORMAL)

class PushBlockEnvironment(Environment):

    def __init__(self):
        super(PushBlockEnvironment, self).__init__()
        wwidth, wdepth, wheight = 4.0, 0.2, 0.4
        self._walls_positions = [(-2.0, 0.0, 0.5*wheight),
                                 ( 0.0, 2.0, 0.5*wheight),
                                 ( 2.0, 0.0, 0.5*wheight),
                                 ( 0.0,-2.0, 0.5*wheight)]
        self._walls_rotations = [tm.rotationZf(-np.pi / 2.),
                                 np.identity(3),
                                 tm.rotationZf(np.pi / 2.),
                                 tm.rotationZf(np.pi)]
        self._walls = []
        for i, (pos, rot) in enumerate(zip(self._walls_positions, self._walls_rotations)) :
            self._walls.append(loco.sim.Box('wall_{}'.format(i), [wwidth, wdepth, wheight], pos, rot, loco.sim.DynamicsType.STATIC))
        self._floor = loco.sim.Plane('floor', 5.0, 5.0, np.zeros(3), np.identity(3))
        self._ball = loco.sim.Sphere('ball', 0.1, (1.0, 1.0, 1.0), np.identity(3))
        self._player = loco.sim.Box('player', (0.2, 0.2, 0.4), (0.0, 0.0, 0.2), np.identity(3))
        self._player.constraint = loco.sim.SingleBodyTranslational3dConstraint( 'player_constraint' )
        self._goal = loco.sim.Cylinder('goal', 0.4, 0.2, (-1.0, -1.0, 0.1), np.identity(3), loco.sim.DynamicsType.STATIC)

    def build(self):
        for wall in self._walls:
            self._scenario.AddSingleBody(wall)
        self._scenario.AddSingleBody(self._floor)
        self._scenario.AddSingleBody(self._ball)
        self._scenario.AddSingleBody(self._player)
        self._scenario.AddSingleBody(self._goal)
        super(PushBlockEnvironment, self).build()

        self._floor.drawable.texture = 'built_in_chessboard'
        self._floor.drawable.ambient = (0.3, 0.5, 0.7)
        self._floor.drawable.diffuse = (0.3, 0.5, 0.7)
        self._floor.drawable.specular = (0.3, 0.5, 0.7)

        for wall in self._walls:
            wall.drawable.ambient = (0.3, 0.3, 0.3)
            wall.drawable.diffuse = (0.3, 0.3, 0.3)
            wall.drawable.specular = (0.3, 0.3, 0.3)

        self._ball.drawable.texture = 'built_in_chessboard'
        self._ball.drawable.ambient = (1.0, 1.0, 1.0)
        self._ball.drawable.diffuse = (1.0, 1.0, 1.0)
        self._ball.drawable.specular = (1.0, 1.0, 1.0)

        self._goal.drawable.ambient = (0.8, 0.1, 0.1)
        self._goal.drawable.diffuse = (0.8, 0.1, 0.1)
        self._goal.drawable.specular = (0.8, 0.1, 0.1)

    def step(self, action):
        self._player.AddForceCOM(action)
        super(PushBlockEnvironment, self).step()
        done, reward=False, 0. # Check for the ball-goal collision
        dist_to_goal = np.clip(1.0 - np.sqrt((self._goal.pos - self._player.pos)**2), 0.0, 1.0)
        reward += 0.1 * dist_to_goal
        for contact in self._goal.collider.contacts:
            if contact.name == self._ball.collider.name:
                print('Successfully pushed ball to the goal!!! :D')
                done, reward=True, reward + 10.0
                break
        observation=np.hstack((self._player.pos, self._goal.pos, self._ball.pos))
        return observation, reward, done, {}

if __name__ == '__main__' :
    env = PushBlockEnvironment()
    env.build()
    env.reset()
    while True :
        env.step(100.0*np.random.randn(3))
        env.render()