import numpy as np
import matplotlib.pyplot as plt

# Parameters
duration = 4.0         # in some unit of time, let's say seconds
dt = 0.5                # stepsize of integration (one tenth of a second, decisecond)
y = 1                   # starting state 

# Data
time = np.arange(dt,duration+dt,dt)     # time in increments of stepsize until duration
y_hist =  np.zeros(len(time))           # place to record the state over time

# Simulation
k = 0                   # index 
for t in time:          # for each integration step
    dydt = y                # rate of change
    y = y + dt * dydt       # update state using Euler method
    y_hist[k] = y           # record state into history
    k += 1

# Visualization
plt.plot(time,y_hist,'-o',fillstyle='none',label="dt=0.5")
plt.plot()
plt.legend()
plt.xlabel("time (sec)")
plt.ylabel("y (level of water)")
plt.show()
