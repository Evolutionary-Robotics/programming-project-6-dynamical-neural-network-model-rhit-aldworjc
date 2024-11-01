import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

def run_worm(robotId, duration):
    ps.Prepare_To_Simulate(robotId)
    x = np.linspace(0,10*np.pi, duration)
    y = np.sin(x)*np.pi/2
    
    for i in range(duration):
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'Foot_Torso',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = y[i],
                               maxForce = 500)
    
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'Torso_Tail',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = y[i],
                               maxForce = 500)
        p.stepSimulation()
        time.sleep(1/500)
    plt.figure()
    plt.plot(x,y)
    plt.xlabel("Time")
    plt.ylabel("Target Angle")
    plt.title("Motor Signal")
    plt.show()

def run_stumble(robotId, duration):
    ps.Prepare_To_Simulate(robotId)
    x = np.linspace(0,10*np.pi, duration)
    Y = np.zeros(duration)
    for i in range(duration):
        y = x[i] % 2*np.pi
        Y[i] = y
        #ps.Set_Motor_For_Joint(bodyIndex = robotId, 
        #                       jointName = b'Left_Foot_Torso',
        #                       controlMode = p.POSITION_CONTROL,
        #                       targetPosition = y,
        #                       maxForce = 500)
    
        #ps.Set_Motor_For_Joint(bodyIndex = robotId, 
        #                       jointName = b'Right_Foot_Torso',
        #                       controlMode = p.POSITION_CONTROL,
        #                       targetPosition = -y,
        #                       maxForce = 500)
        p.stepSimulation()
        time.sleep(1/500)
    plt.figure()
    plt.plot(x,Y)
    plt.xlabel("Time")
    plt.ylabel("Target Angle")
    plt.title("Motor Signal")
    plt.show()

def run_walker(robotId, duration):
    ps.Prepare_To_Simulate(robotId)
    
    x = np.linspace(0,10*np.pi, duration)
    y = np.sin(x)*np.pi/2
    
    for i in range(duration):
        p.stepSimulation()
        pos, _ = (p.getBasePositionAndOrientation(robotId))
        print(pos)
        time.sleep(1/500)
        
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
rId = p.loadURDF("walker.urdf")


run_walker(rId,10000)



p.disconnect()
