import numpy as np
import matplotlib.pyplot as plt


def cubicTraj(T, pos_s, pos_f, vel_s, vel_f):
# This function should return the coefficients for a cubic polynomial that will be used
# to define a trajectory between two points/angles (single degree of freedom) for some time duration T.
# The user should provide start and ending positions and start and ending velocities
# as well as the desired time duration for the trajectory.

# First define a matrix A as we did in class
    A = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [1, T, T**2, T**3],
                 [0, 1, 2*T, 3*T**2]]) #fill in the matrix here

#Next define a vector b
    b = np.array([pos_s, vel_s, pos_f, vel_f]) #fill in the vector

#Calculate the coefficients for the trajectory here
    coeffs = np.linalg.pinv(A)@b 

    return coeffs

def quinticTraj(T, pos_s, pos_f, vel_s, vel_f, acc_s, acc_f):
    # This function should return the coefficients for a quintic polynomial that will be used
    # to define a trajectory between two points/angles (single degree of freedom) for some time duration T.
    # The user should provide start/ending positions, start/ending velocities and start/ending accelerations
    # as well as the desired time duration for the trajectory.

    #Use a similar strategy as with part (a) and as we did in class.

    A = np.array([[1, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 2, 0, 0, 0],
                  [1, T, T**2, T**3, T**4, T**5],
                  [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                  [0, 0, 2, 6*T, 12*T**2, 20*T**3]])
    
    b = np.array([pos_s, vel_s, acc_s, pos_f, vel_f, acc_f]) 
    coeffs = np.linalg.pinv(A)@b 

    return coeffs

def trajPoints(coeffs, T, num, type="cubic"):
    # This function should return vectors of position, velocity and accelerations for a trajectory at each timestep
    # The user should provide the coefficients of the polynomial that define the trajectory as well as the duration
    # T of the trajectory AND the desired number of timesteps.

    # create a time vector with the correct number of time steps over the full duration
    t = np.linspace(0, T, num)

    if type == 'cubic':
        
        a0 = coeffs[0]
        a1 = coeffs[1]
        a2 = coeffs[2]
        a3 = coeffs[3]

        pos = a3*t**3 + a2*t**2 + a1*t + a0
        vel = 3*a3*t**2 + 2*a2*t + a1
        acc = 6*a3*t + 2*a2

        return t, pos, vel, acc
    
    if type == 'quintic':
        
        a0 = coeffs[0]
        a1 = coeffs[1]
        a2 = coeffs[2]
        a3 = coeffs[3]
        a4 = coeffs[4]
        a5 = coeffs[5]

        pos = a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0
        vel = 5*a5*t**4 + 4*a4*t**3 + 3*a3*t**2 + 2*a2*t + a1
        acc = 20*a5*t**3 + 12*a4*t**2 + 6*a3*t + 2*a2

        return t, pos, vel, acc
    
    else:
        print("not a valid trajectory type")
        return None

def traj3D(start, end, numpoints, type="quintic"):

    #This function should return a 3 degree of freedom trajectory (angular or cartesian) for a robotic manipulator
    #The user will provide a start/end points or angles, start/end velocities and start/end accelerations, as well
    #as the number of points that should be in the trajectory and the type of trajectory desired.

    pos = np.zeros(shape=(numpoints, 3))
    vel = np.zeros(shape=(numpoints, 3))
    acc = np.zeros(shape=(numpoints, 3))

    if type == 'quintic':
        for i in range(3):
    #generate the trajectory for each degree of freedom (joint or cartesian) for a quintic
            coeffs = quinticTraj(T, start[i], end[i], start[i+3], end[i+3], start[i+6], end[i+6])
            t, pos[:,i], vel[:,i], acc[:,i] = trajPoints(coeffs, T, num=numpoints, type="quintic")

    elif type == 'cubic':
        for i in range(3):
    # generate the trajectory for each degree of freedom (joint or cartesian) for a cubic
            coeffs = cubicTraj(T, start[i], end[i], start[i+3], end[i+3])
            t, pos[:,i], vel[:,i], acc[:,i] = trajPoints(coeffs, T, num=numpoints, type="cubic")

    else:
        print("not a valid trajectory type")

    return t, pos, vel, acc

if __name__ == "__main__":

 # PART (4c): Fill in what you need to run part (4c) and produce a plot showing position, velocity and
 # acceleration for the specified cubic trajectory

 #input variables
    T = 5.          #time
    stp = np.pi/2   #start point
    edp = np.pi     #end point
    #add others as needed...
    stv = 0         #start velocity
    edv = 0         #end velocity

    coeffs = cubicTraj(T, stp, edp, stv, edv)  #you will definitely call this, but it will need some inputs
    t, pos, vel, acc = trajPoints(coeffs, T, num=20, type="cubic") #then you will need this

 #do some plotting!

    plt.figure()
    plt.title('Cubic Trajectory')
    plt.plot(t, pos, label='position')
    plt.plot(t, vel, label='velocity')
    plt.plot(t, acc, label='acceleration')
    plt.xlabel('time [s]')
    plt.ylabel('value')
    plt.legend()
    plt.grid()
    plt.show()


 # PART (4f): Fill in what you need to run part (4f) and produce a plot showing position, velocity and
 # acceleration for the specified quintic trajectory (should be very similar to the above)

    T = 5.          #time
    stp = np.pi/2   #start point
    edp = np.pi     #end point
    stv = 0         #start velocity
    edv = 0         #end velocity
    sta = 0         #start acceleration
    eda = 0         #end acceleration

    coeffs = quinticTraj(T, stp, edp, stv, edv, sta, eda) 
    t, pos, vel, acc = trajPoints(coeffs, T, num=20, type="quintic") 

    plt.figure()
    plt.title('Quintic Trajectory')
    plt.plot(t, pos, label='position')
    plt.plot(t, vel, label='velocity')
    plt.plot(t, acc, label='acceleration')
    plt.xlabel('time [s]')
    plt.ylabel('value')
    plt.legend()
    plt.grid()
    plt.show()

 # PART (5b): Fill in what you need to run part (5b) and produce a 3D plot showing the requested trajectory

 # The start and end vectors have the following format:

# For the cubic case:
# start = [x_start, y_start, z_start, x_dot_start, y_dot_start, z_dot_start]
# end = [x_end, y_end, z_end, x_dot_end, y_dot_end, z_dot_end]

 # For the quintic case:
 # start = [x_start, y_start, z_start, x_dot_start, y_dot_start, z_dot_start, x_ddot_start, y_ddot_start, z_ddot_start]
 # end = [x_end, y_end, z_end, x_dot_end, y_dot_end, z_dot_end, x_ddot_end, y_ddot_end, z_ddot_end]

# Start and positions, velocities and accelerations
start_positions = [0, 0, 0]
start_velocities = [0, 0, 0]
start_accelerations = [0, 0, 0]
end_positions = [5, 3, 7]
end_velocities = [0, 0, 0]
end_accelerations = [0, 0, 0]

# Build the start and end vectors
start = start_positions + start_velocities + start_accelerations
end = end_positions + end_velocities + end_accelerations

# Generate the trajectory, in this case quintic is used
T = 5
num_points = 50
t, pos, vel, acc = traj3D(start, end, num_points, type="quintic")

# Plot the 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(pos[:,0], pos[:,1], pos[:,2], c='r', marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
# Set title
plt.title('3D Trajectory from [0,0,0] to [5,3,7]')
plt.show()


