import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.loadURDF("plane.urdf")

robot = p.loadURDF("urdf/UR3_with_gripper.urdf", useFixedBase=True)

# делаем бесконечный цикл, чтобы окно не закрывалось
while True:
    p.stepSimulation()
    time.sleep(1./240.)
