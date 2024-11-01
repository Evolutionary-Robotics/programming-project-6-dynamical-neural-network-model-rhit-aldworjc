import lotkavolterra as lv
import numpy as np
import matplotlib.pyplot as plt

a = 1.0     # alpha = reproduction rate of prey
b = 0.1     # beta = mortality rate of predator per prey
c = 0.4     # gamma = mortality rate of predator
d = 0.02    # delta = reproduction rate of predator per prey

x = 10      # Starting value of prey (rabbits)
y = 10      # Starting value of predator (foxes)

a = lv.LotkaVolterra(a,b,c,d,x,y)

Duration = 100.0
StepSize = 0.00001
Steps = int(Duration/StepSize)

yhist = np.zeros(Steps)
xhist = np.zeros(Steps)
thist = np.zeros(Steps)

t=0.0
for i in range(Steps):
    a.step(StepSize)
    t += StepSize
    yhist[i]=a.y
    xhist[i]=a.x
    thist[i]=t

plt.plot(thist,yhist,'.',label="Foxes")
plt.plot(thist,xhist,'.',label="Rabbits")
plt.xlabel("time")
plt.ylabel("population size")
plt.legend(loc="upper right")
plt.show()

m = np.max([np.max(xhist),np.max(yhist)]) + 1
plt.plot(xhist,yhist)
plt.plot(xhist[0],yhist[0],"ro")
plt.xlabel("Rabbits")
plt.ylabel("Foxes")
plt.xlim(0,m)
plt.ylim(0,m)
plt.show()
