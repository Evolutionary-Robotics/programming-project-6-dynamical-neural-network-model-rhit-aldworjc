import ctrnn
import matplotlib.pyplot as plt
import numpy as np

# Parameters
size = 10
duration = 100
stepsize = 0.01

# Data
time = np.arange(0.0,duration,stepsize)
outputs = np.zeros((len(time),size))
states = np.zeros((len(time),size))

# Initialization
nn = ctrnn.CTRNN(size)

# Neural parameters written by hand
# w = [[5.422,-0.24,0.535],[-0.018,4.59,-2.25],[2.75,1.21,3.885]]
# b = [-4.108,-2.787,-1.114]
# t = [1,2.5,1]
# nn.setParameters(w,b,t)
# nn.initializeState(np.array([4.0,2.0,1.0]))

# Neural parameters at random
nn.randomizeParameters()

# Initialization at zeros or random
nn.initializeState(np.zeros(size))
#nn.initializeState(np.random.random(size=size)*20-10)

# Run simulation
step = 0
for t in time:
    nn.step(stepsize)
    states[step] = nn.States
    outputs[step] = nn.Outputs
    step += 1

# How much is the neural activity changing over time
activity = np.sum(np.abs(np.diff(outputs,axis=0)))/(duration*size*stepsize)
print("Overall activity: ",activity)

# Plot activity
plt.plot(time,outputs)
plt.xlabel("Time")
plt.ylabel("Outputs")
plt.title("Neural output activity")
plt.show()

# Plot activity
plt.plot(time,states)
plt.xlabel("Time")
plt.ylabel("States")
plt.title("Neural state activity")
plt.show()

# Save CTRNN parameters for later
nn.save("ctrnn")
