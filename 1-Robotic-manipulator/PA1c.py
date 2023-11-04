"""
Robot Dynamics and Control Assignment 1c: kinematic control of endpoint
-------------------------------------------------------------------------------
DESCRIPTION:
2-DoF planar robot arm model with shoulder and elbow joints. The code includes
a simulation environment and visualisation of the robot.

The robot is a classic position-controlled robot:
- Measured joint angle vector q is provided each sample time.
- Calculate the joint velocity dq in each sample time to be sent to the robot.

Important variables:
q[0] -> shoulder joint configuration
q[1] -> elbow joint configuration
p[0] -> endpoint x position
p[1] -> endpoint y position

TASK:
Make the robot track a given endpoint reference trajectory by using a kinematic
control (i.e., PID controller). Experimentally tune the PID parameters to
achieve a good performance. Make sure the controller is robust to singularities.
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
    def __init__(self, l):
        self.l = l # link length
        
        self.q = np.array([0.0, 0.0]) # joint position
        self.dq = np.array([0.0, 0.0]) # joint veloctiy
        self.tau = np.array([0.0, 0.0]) # joint torque


    
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
    
    
    
    # inverse kinematics
    def IK(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0]**2+p[1]**2)
        q[1] = np.pi - math.acos((self.l[0]**2+self.l[1]**2-r**2)/(2*self.l[0]*self.l[1]))
        q[0] = math.atan2(p[1],p[0]) - math.acos((self.l[0]**2-self.l[1]**2+r**2)/(2*self.l[0]*r))
        
        return q
    
    
    
    # state change
    def state(self, q, dq):
        self.q = q
        self.dq = dq






'''SIMULATION'''

# SIMULATION PARAMETERS
dt = 0.01 # integration step time
dts = dt*1 # desired simulation step time (NOTE: it may not be achieved)
T = 3 # total simulation time



# ROBOT PARAMETERS
x0 = 0.0 # base x position
y0 = 0.0 # base y position
l1 = 0.3 # link 1 length
l2 = 0.3 # link 2 length
l = [l1, l2] # link length



# REFERENCE TRAJECTORY
ts = T/dt # trajectory size
xt = np.linspace(-2,2,int(ts))
yt1 = np.sqrt(1-(abs(xt)-1)**2)
yt2 = -3*np.sqrt(1-(abs(xt)/2)**0.5)

x = np.concatenate((xt, np.flip(xt,0)), axis=0)
y = np.concatenate((yt1, np.flip(yt2,0)), axis=0)

pr = np.array((x / 5 + 0.0, y / 5 + 0.45)) # reference endpoint trajectory



'''*********** Student should fill in ***********'''
# PID CONTROLLER PARAMETERS
Kp =  100 # proportional gain
Ki =  0.1 # integral gain
Kd =  0.1 # derivative gain
'''*********** Student should fill in ***********'''
se = np.zeros([2]) # integrated error
pe = np.zeros([2]) # previous error



# SIMULATOR
# initialise robot model class
model = robot_arm_2dof(l)

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
q = model.IK([-0.38, 0.43]) # initial configuration
dq = np.array([0., 0.]) # joint velocity
model.state(q, dq) # update initial state
i = 0 # loop counter
state = [] # state vector

# scaling
window_scale = 400 # conversion from meters to pixels

print(pr.shape)



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
    
    # KINEMATIC CONTROL
    
    # Update q and dq in model state
    model.state(q, dq)
    
    # Obtain position
    p = model.FK()
    
    # Compute error
    e = pr[:,i] - p
    
    # Jacobian
    J = model.Jacobian()
    
    # Damping coefficient for damped least-squares method
    lambda_coeff = 0.008
    
    # Compute Jacobian singularity-robust pseudo-inverse
    lambda_I = lambda_coeff * np.identity(J.shape[0])
    paren_inverted = np.linalg.inv(np.dot(J, np.transpose(J)) + lambda_I)
    J_plus = np.dot(np.transpose(J), paren_inverted)
    
    # Compute joint velocity
    dq = np.dot(J_plus, Kp*e + Ki*se + Kd*(e-pe)/dt) 

    # Update integrated error and previous error 
    se += e*dt 
    pe = e
    

    '''*********** Student should fill in ***********'''



    # log states for analysis
    state.append([t, q[0], q[1], dq[0], dq[1], p[0], p[1]])
    
    # get joint angles by integration
    q += dq*dt
    t += dt
    
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
plt.subplot(211)
plt.title("JOINT SPACE BEHAVIOUR")
plt.plot(state[:,0],state[:,3],"b",label="shoulder")
plt.plot(state[:,0],state[:,4],"r",label="elbow")
plt.legend()
plt.ylabel("dq [rad/s]")

plt.subplot(212)
plt.plot(state[:,0],state[:,1],"b",label="shoulder")
plt.plot(state[:,0],state[:,2],"r",label="elbow")
plt.legend()
plt.ylabel("q [rad]")
plt.xlabel("t [s]")

plt.tight_layout()




plt.figure(2)
plt.title("ENDPOINT SPACE BEHAVIOUR")
plt.plot(0,0,"ok",label="shoulder")
plt.plot(state[:,5],state[:,6],label="trajectory")
plt.plot(state[0,5],state[0,6],"xg",label="start point")
plt.plot(state[-1,5],state[-1,6],"+r",label="end point")
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.tight_layout()




