import ctrnn
import matplotlib.pyplot as plt
import numpy as np

# EXPERIMENT PARAMETERS
size = 10
duration = 100
stepsize = 0.01
time = np.arange(0.0,duration,stepsize)

# DEFINITION OF THE OBSERVABLE
def activity(er):
    nn = ctrnn.CTRNN(size)
    nn.randomizeParameters()

    # Set some correct proportion of excitatory/inhibitory connections
    for i in range(size):
        for j in range(size):
            if np.random.random() < er:
                nn.Weights[i,j] = np.random.random()*10.0 - 0.5
            else:
                nn.Weights[i,j] = 0.0 #np.random.random()*(-10.0)

    nn.initializeState(np.zeros(size))
    outputs = np.zeros((len(time),size))

    # Run transient
    for t in time:
        nn.step(stepsize)

    # Run simulation
    step = 0
    for t in time:
        nn.step(stepsize)
        outputs[step] = nn.Outputs
        step += 1

    # Sum the absolute rate of change of all neurons across time as a proxy for "active"
    activity = np.sum(np.abs(np.diff(outputs,axis=0)))/(duration*size*stepsize)
    return activity

# ITERATE THROUGH DIFFERENT PROPORTIONS
reps = 100
steps = 11
errange = np.linspace(0.0,1.0,steps)

data = np.zeros((steps,reps))
k = 0
for er in errange:
    for r in range(reps):
        data[k][r] = activity(er)
    k += 1


# visualize the results
plt.plot(errange,np.mean(data,axis=1),'ko')
plt.plot(errange,np.mean(data,axis=1)+np.std(data,axis=1)/np.sqrt(reps),'k.')
plt.plot(errange,np.mean(data,axis=1)-np.std(data,axis=1)/np.sqrt(reps),'k.')
plt.xlabel("Proportion of excitatory connections")
plt.ylabel("Amount of activity in the circuit")
plt.show()
