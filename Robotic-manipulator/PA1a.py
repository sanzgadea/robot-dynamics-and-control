"""
Robot Dynamics and Control Assignment 1a: PID controller and step response
-------------------------------------------------------------------------------
DESCRIPTION:
1-DoF mass-damper plant controlled by a PID controller.

Important variables:
xr -> plant reference position
x -> plant actual position
dx -> plant velocity
ddx -> plant acceleration
F -> actuator force

TASK:
Create and tune PID controllers for three different cases to achieve step
responses of a mass-damper plant that include:
1) overshoot
2) no overshoot
3) instability
-------------------------------------------------------------------------------


INSTRUCTOR: Luka Peternel
e-mail: l.peternel@tudelft.nl

"""



import numpy as np
import matplotlib.pyplot as plt



'''PARAMETERS'''

m = 1.0 # mass
b = 1.0 # damping (e.g., air/water resistance)
g = 9.81 # gravity

dt = 0.001 # sample time
T = 10.0 # simulation time

xr = 1.0 # reference position (set-point)

state1 = [] # case 1 state vector
state2 = [] # case 2 state vector
state3 = [] # case 3 state vector








'''SIMULATION'''

# OVERSHOOT
'''*********** Student should fill in ***********'''
Kp = 10 # proportional gain
Ki = 1 # integral gain
Kd = 1 # derivative gain
'''*********** Student should fill in ***********'''
se = 0.0 # integrated error
pe = 0.0 # previous error (used for derivation)
t = 0.0 # time
x = 0.0 # position
dx = 0.0 # velocity
ddx = 0.0 # acceleration
for i in range(int(T/dt)):
    state1.append([t,x,dx,ddx])
    
    # PID controller
    '''*********** Student should fill in ***********'''
    e = xr-x 
    se += e * dt
    F = Kp*(e) + Ki*se + Kd*(e-pe)/dt
    pe = e
    '''*********** Student should fill in ***********'''
    
    # dynamics & numerical integration
    ddx = F/m - dx*b/m
    dx = dx + ddx*dt
    x = x + dx*dt
    t += dt



# NO OVERSHOOT
'''*********** Student should fill in ***********'''
Kp = 0.3 # proportional gain
Ki = 0.1 # integral gain
Kd = 10 # derivative gain
'''*********** Student should fill in ***********'''
se = 0.0 # integrated error
pe = 0.0 # previous error (used for derivation)
t = 0.0 # time
x = 0.0 # position
dx = 0.0 # velocity
ddx = 0.0 # acceleration
for i in range(int(T/dt)):
    state2.append([t,x,dx,ddx])
    
    # PID controller
    '''*********** Student should fill in ***********'''
    e = xr-x 
    se += e * dt
    F = Kp*(e) + Ki*se + Kd*(e-pe)/dt
    pe = e
    '''*********** Student should fill in ***********'''
    
    # dynamics & numerical integration
    ddx = F/m - dx*b/m
    dx = dx + ddx*dt
    x = x + dx*dt
    t += dt



# INSTABILITY
'''*********** Student should fill in ***********'''
Kp = 2 # proportional gain
Ki = 5 # integral gain
Kd = 1 # derivative gain
'''*********** Student should fill in ***********'''
se = 0.0 # integrated error
pe = 0.0 # previous error (used for derivation)
t = 0.0 # time
x = 0.0 # position
dx = 0.0 # velocity
ddx = 0.0 # acceleration
for i in range(int(T/dt)):
    state3.append([t,x,dx,ddx])
    
    # PID controller
    '''*********** Student should fill in ***********'''
    e = xr-x 
    se += e * dt
    F = Kp*(e) + Ki*se + Kd*(e-pe)/dt
    pe = e
    '''*********** Student should fill in ***********'''
    
    # dynamics & numerical integration
    ddx = F/m - dx*b/m
    dx = dx + ddx*dt
    x = x + dx*dt
    t += dt












'''ANALYSIS'''

state1 = np.array(state1)
state2 = np.array(state2)
state3 = np.array(state3)


plt.figure(1,figsize=(5, 20))
plt.subplot(311)
plt.title("STEP RESPONSE CASES", fontsize=10)
plt.plot(state1[:,0],state1[:,1]*0+xr,"--g")
plt.step(state1[:,0],state1[:,1],"b", label="overshoot")
plt.ylabel("position [m]", fontsize=12)
plt.xlim((0,10))
plt.ylim((-0.1,2.1))
plt.legend()

plt.subplot(312)
plt.plot(state1[:,0],state2[:,1]*0+xr,"--g")
plt.step(state1[:,0],state2[:,1],"m", label="no overshoot")
plt.ylabel("position [m]", fontsize=12)
plt.xlim((0,10))
plt.ylim((-0.1,2.1))
plt.legend()

plt.subplot(313)
plt.plot(state1[:,0],state3[:,1]*0+xr,"--g")
plt.step(state2[:,0],state3[:,1],"r", label="instability")
plt.ylabel("position [m]", fontsize=12)
plt.xlabel("time [s]")
plt.xlim((0,10))
plt.ylim((-0.1,2.1))
plt.legend(loc=1)

plt.tight_layout()
