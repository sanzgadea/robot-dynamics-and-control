"""
Robot Dynamics and Control Assignment 1e: dynamic control of endpoint
-------------------------------------------------------------------------------
DESCRIPTION:
2-DoF planar robot arm model with shoulder and elbow joints. The code includes
a simulation environment and visualisation of the robot.

The robot is a torque-controlled robot:
- Measured joint angle vector q is provided each sample time.
- Calculate the joint torque tau in each sample time to be sent to the robot.

Important variables:
q[0]/tau[0] -> shoulder joint configuration/torque
q[1]/tau[1] -> elbow joint configuration/torque
p[0] -> endpoint x position
p[1] -> endpoint y position

TASK:
Derive the forward dynamics model of a given 2-DoF planar robot. This model
should be used to simulate the robot and NOT to compensate for its dynamics.
Basically you want to know how your input torque moves the robot so that the
simulation can be updated in each sample time.

Make the robot track a given endpoint reference trajectory by using a dynamic
control (i.e., impedance controller). Bonus: try impedance control without and
with the damping term to analyse the stability.
-------------------------------------------------------------------------------


INSTURCTOR: Luka Peternel
e-mail: l.peternel@tudelft.nl

"""



import numpy as np
import math
import matplotlib.pyplot as plt
import pygame





'''ROBOT MODEL'''

class robot_arm_2dof:
    def __init__(self, l, m, ag):
        self.l = l # link length
        self.m = m # link mass
        self.ag = ag # acceleration of gravity
        
        self.q = np.zeros([2]) # joint position
        self.dq = np.zeros([2]) # joint veloctiy
        self.tau = np.zeros([2]) # joint torque
        
        # joint friction matrix (NOT part of model derivation with the Lagrangian method, but can be added to the model as a separate term)
        self.B = np.array([[0.050, 0.025],
                           [0.025, 0.050]])
    
        # external damping matrix, e.g., when the endpoint is moving inside a fluid (NOT part of model derivation with the Lagrangian method, but can be added to the model as a separate term)
        self.D = np.diag([0.01, 0.01])



    # forward kinematics
    def FK(self):
        p = np.zeros([2]) # endpoint position
        '''*********** Student should fill in ***********'''
        p[0] = self.l[0]*np.cos(self.q[0]) + self.l[1]*np.cos(self.q[0] + self.q[1])
        p[1] = self.l[0]*np.sin(self.q[0]) + self.l[1]*np.sin(self.q[0] + self.q[1])
        '''*********** Student should fill in ***********'''
        return p
    

    
    # Jacobian matrix
    def Jacobian(self):
        '''*********** Student should fill in ***********'''
        J = np.zeros([2,2])
        J[0,0] = -self.l[0]*np.sin(self.q[0]) - self.l[1]*np.sin(self.q[0] + self.q[1])
        J[0,1] = -self.l[1]*np.sin(self.q[0] + self.q[1])
        J[1,0] = self.l[0]*np.cos(self.q[0]) + self.l[1]*np.cos(self.q[0] + self.q[1])
        J[1,1] = self.l[1]*np.cos(self.q[0] + self.q[1])
        '''*********** Student should fill in ***********'''
        return J

    
    
    # forward dynamics
    def FD(self):
        ddq = np.zeros([2]) # joint acceleration
        '''*********** Student should fill in ***********'''
        M = np.zeros([2,2])
        M[0,0] = self.m[0]*self.l[0]**2 + self.m[1]*(self.l[0]**2 + 2*self.l[0]*self.l[1]*np.cos(self.q[1]) + self.l[1]**2)
        M[0,1] = self.m[1]*(self.l[0]*self.l[1]*np.cos(self.q[1])+self.l[1]**2)
        M[1,0] = self.m[1]*(self.l[0]*self.l[1]*np.cos(self.q[1])+self.l[1]**2)
        M[1,1] = self.m[1]*self.l[1]**2
        
        C = np.zeros([2])
        C[0] = - self.m[1]*self.l[0]*self.l[1]*np.sin(self.q[1])*(2*self.dq[0]*self.dq[1]+self.dq[1]**2)
        C[1] = self.m[1]*self.l[0]*self.l[1]*np.sin(self.q[1])*self.dq[0]**2
        
        G = np.zeros([2])
        G[0] = (self.m[0] + self.m[1])*self.ag*self.l[0]*np.cos(self.q[0]) + self.m[1]*self.ag*self.l[1]*np.cos(self.q[0]+self.q[1])
        G[1] = self.m[1]*self.ag*self.l[1]*np.cos(self.q[0]+self.q[1])
        
        M_inverse = np.linalg.inv(M)
        ddq = np.matmul(M_inverse, self.tau - C - G)
        '''*********** Student should fill in ***********'''
        return ddq
    
    
    
    # inverse kinematics
    def IK(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0]**2+p[1]**2)
        q[1] = np.pi - math.acos((self.l[0]**2+self.l[1]**2-r**2)/(2*self.l[0]*self.l[1]))
        q[0] = math.atan2(p[1],p[0]) - math.acos((self.l[0]**2-self.l[1]**2+r**2)/(2*self.l[0]*r))
        
        return q
    
    
    
    # state change
    def state(self, q, dq, tau):
        self.q = q
        self.dq = dq
        self.tau = tau






'''SIMULATION'''

# SIMULATION PARAMETERS
dt = 0.01 # intergration step time dt = 0.01 # integration step time
dts = dt*1 # desired simulation step time (NOTE: it may not be achieved)
T = 3 # total simulation time



# ROBOT PARAMETERS
x0 = 0.0 # base x position
y0 = 0.0 # base y position
l1 = 0.3 # link 1 length
l2 = 0.3 # link 2 length
l = [l1, l2] # link length
m = [1.0, 1.0] # link mass
ag = 9.81 # acceleration of gravity



# REFERENCE TRAJECTORY
ts = T/dt # trajectory size
xt = np.linspace(-2,2,int(ts))
yt1 = np.sqrt(1-(abs(xt)-1)**2)
yt2 = -3*np.sqrt(1-(abs(xt)/2)**0.5)

x = np.concatenate((xt, np.flip(xt,0)), axis=0)
y = np.concatenate((yt1, np.flip(yt2,0)), axis=0)

pr = np.array((x / 10 + 0.0, y / 10 + 0.45)) # reference endpoint trajectory



'''*********** Student should fill in ***********'''
# IMPEDANCE CONTROLLER PARAMETERS
K =  10000 # stiffness matrix N/m
D =  2 * 0.7 * np.sqrt(K) # damping matrix Ns/m
'''*********** Student should fill in ***********'''



# SIMULATOR
# initialise robot model class
model = robot_arm_2dof(l, m, ag)

# initialise real-time plot with pygame
pygame.init() # start pygame
window = pygame.display.set_mode((800, 600)) # create a window (size in pixels)
window.fill((255,255,255)) # white background
xc, yc = window.get_rect().center # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255)) # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dts) # refresh rate

# initial conditions
t = 0.0 # time
q = model.IK(pr[:,0]) # joint position
dq = np.array([0., 0.]) # joint velocity
tau = np.array([0., 0.]) # joint torque
model.state(q, dq, tau) # update initial state
p_prev = pr[:,0] # previous endpoint position
i = 0 # loop counter
state = [] # state vector

# scaling
window_scale = 400 # conversion from meters to pixels



# wait until the start button is pressed
run = True
while run:
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.KEYUP:
            if event.key == ord('e'): # enter the main loop after 'e' is pressed
                run = False



# MAIN LOOP
run = True
for i in range(len(x)):
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.QUIT: # force quit with closing the window
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('q'): # force quit with q button
                run = False
    
    # update individual link position
    x1 = l1*np.cos(q[0])
    y1 = l1*np.sin(q[0])
    x2 = x1+l2*np.cos(q[0]+q[1])
    y2 = y1+l2*np.sin(q[0]+q[1])
    
    # real-time plotting
    window.fill((255,255,255)) # clear window
    pygame.draw.circle(window, (0, 255, 0), (int(window_scale*pr[0,i])+xc,int(-window_scale*pr[1,i])+yc), 3) # draw reference position
    pygame.draw.lines(window, (0, 0, 255), False, [(window_scale*x0+xc,-window_scale*y0+yc), (window_scale*x1+xc,-window_scale*y1+yc), (window_scale*x2+xc,-window_scale*y2+yc)], 3) # draw links
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale*x0)+xc,int(-window_scale*y0)+yc), 7) # draw shoulder / base
    pygame.draw.circle(window, (0, 0, 0), (int(window_scale*x1)+xc,int(-window_scale*y1)+yc), 7) # draw elbow
    pygame.draw.circle(window, (255, 0, 0), (int(window_scale*x2)+xc,int(-window_scale*y2)+yc), 3) # draw hand / endpoint
    
    text = font.render("FPS = " + str( round( clock.get_fps() ) ), True, (0, 0, 0), (255, 255, 255))
    window.blit(text, textRect)
    
    pygame.display.flip() # update display
    
    
    
    '''*********** Student should fill in ***********'''
    
    # DYNAMIC CONTROL
    
    # Obtain position
    p = model.FK()
    
    # Compute error
    e = pr[:,i] - p
    
    # Compute x_dot
    x_dot = (p - p_prev) / dt
    
    # Compute F
    F = K*e - D*x_dot 
    
    # Jacobian
    J = model.Jacobian()
    
    # Compute tau
    tau = np.dot(J.T, F)
    
    # Update q, dq and tau in model state
    model.state(q, dq, tau)
    
    # Obtain ddq
    ddq = model.FD()
    
    '''*********** Student should fill in ***********'''



    # log states for analysis
    state.append([t, q[0], q[1], dq[0], dq[1], ddq[0], ddq[1], tau[0], tau[1], p[0], p[1]])    
        
    # integration
    dq += ddq*dt
    q += dq*dt
    t += dt
    
    # previous endpoint position for velocity calculation
    p_prev = p
    
    # increase loop counter
    i = i + 1
    
    # try to keep it real time with the desired step time
    clock.tick(FPS)
    
    if i >= len(x):
        run = False
    if run == False:
        break

pygame.quit() # stop pygame












'''ANALYSIS'''

state = np.array(state)


plt.figure(1)
plt.subplot(411)
plt.title("JOINT SPACE BEHAVIOUR")
plt.plot(state[:,0],state[:,7],"b",label="shoulder")
plt.plot(state[:,0],state[:,8],"r",label="elbow")
plt.legend()
plt.ylabel("tau [Nm]")

plt.subplot(412)
plt.plot(state[:,0],state[:,5],"b")
plt.plot(state[:,0],state[:,6],"r")
plt.ylabel("ddq [rad/s2]")

plt.subplot(413)
plt.plot(state[:,0],state[:,3],"b")
plt.plot(state[:,0],state[:,4],"r")
plt.ylabel("dq [rad/s]")

plt.subplot(414)
plt.plot(state[:,0],state[:,1],"b")
plt.plot(state[:,0],state[:,2],"r")
plt.ylabel("q [rad]")
plt.xlabel("t [s]")

plt.tight_layout()




plt.figure(2)
plt.title("ENDPOINT SPACE BEHAVIOUR")
plt.plot(0,0,"ok",label="shoulder")
plt.plot(state[:,9],state[:,10],label="trajectory")
plt.plot(state[0,9],state[0,10],"xg",label="start point")
plt.plot(state[-1,9],state[-1,10],"+r",label="end point")
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.tight_layout()




