import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time
import matplotlib.pyplot as plt
import ctrnn

def run_walker(duration):
    #setup Physics Simulation
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadURDF("walker.urdf")
    ps.Prepare_To_Simulate(robotId)

    #setup neural network
    size = 4
    stepsize = 0.01
    
    nn = ctrnn.CTRNN(4)
    nn.randomizeParameters(2*np.pi)
    
    t = np.arange(0.0,duration,stepsize)
    outputs = np.zeros((len(t),size))
    states = np.zeros((len(t),size))
    step = 0
    for i in t:
        p.stepSimulation()
        nn.step(stepsize)
        sig = nn.Outputs
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'LF_Motor',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = sig[0],
                               maxForce = 500)
    
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'RF_Motor',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = sig[1],
                               maxForce = 500)
        
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'LB_Motor',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = sig[2],
                               maxForce = 500)
    
        ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                               jointName = b'RB_Motor',
                               controlMode = p.POSITION_CONTROL,
                               targetPosition = sig[3],
                               maxForce = 500)
        pos, _ = (p.getBasePositionAndOrientation(robotId))
        nn.Inputs = [pos[0],pos[1],pos[2],10*i/duration]
        states[step] = nn.States
        outputs[step] = nn.Outputs
        step += 1
        time.sleep(1/500)

    p.disconnect()
    # How much is the neural activity changing over time
    activity = np.sum(np.abs(np.diff(outputs,axis=0)))/(duration*size*stepsize)
    print("Overall activity: ",activity)

    # Plot activity
    plt.plot(t,outputs)
    plt.xlabel("Time")
    plt.ylabel("Outputs")
    plt.title("Neural output activity")
    plt.show()

    # Plot activity
    plt.plot(t,states)
    plt.xlabel("Time")
    plt.ylabel("States")
    plt.title("Neural state activity")
    plt.show()
        

#p.loadSDF("box.sdf")



run_walker(25)




