import pybullet as p
import time
p.connect(p.GUI)
offset = [0,0,0]

turtle = p.loadURDF("anymal.urdf",offset)
p.setRealTimeSimulation(1)

while (1):
	p.setGravity(0,0,-10)
	time.sleep(1./240.)

