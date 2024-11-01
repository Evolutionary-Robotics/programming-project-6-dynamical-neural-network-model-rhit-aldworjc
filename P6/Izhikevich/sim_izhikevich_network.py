import izhikevich as iz
import matplotlib.pyplot as plt
import numpy as np

# Global Parameters
size = 10
duration = 100 #1000
stepsize = 0.001

time = np.arange(0.0,duration,stepsize)

nn = iz.IzhikevichNetwork(size)
nn.pcnnParams(8)

outputs = np.zeros((len(time),size))
firings = np.zeros((len(time),size))

# Run simulation
step = 0
inputs = np.zeros(size)
for t in time:
    if t > 10:
        inputs = 10*np.ones(size)
    nn.step(stepsize, inputs)
    outputs[step] = nn.Voltages
    firings[step] = nn.Firing
    step += 1

# Plot activity
plt.plot(time,outputs)
plt.xlabel("Time")
plt.ylabel("Voltage")
plt.title("Neural activity")
plt.show()

# Plot activity
plt.imshow(firings.T, cmap='Greys', interpolation='nearest', aspect='auto')
plt.xlabel("Time")
plt.ylabel("Voltage")
plt.title("Neural activity")
plt.show()
