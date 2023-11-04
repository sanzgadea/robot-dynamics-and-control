
import numpy as np
import matplotlib.pyplot as plt

#%matplotlib qt

def dh(alpha, a, d, theta):

    # Take the DH-Parameters as inputs (one row of the DH-Table)
    # Output the homogenous transformation matrix T for any input joint position theta.

    # Row 1 of the matrix
    r_1_1 = np.cos(theta)
    r_1_2 = -1*np.sin(theta)*np.cos(alpha)
    r_1_3 = np.sin(theta)*np.sin(alpha)
    r_1_4 = np.cos(theta)*a

    # Row 2 of the matrix
    r_2_1 = np.sin(theta)
    r_2_2 = np.cos(theta)*np.cos(alpha)
    r_2_3 = -1*np.cos(theta)*np.sin(alpha)
    r_2_4 = np.sin(theta)*a

    # Row 3 of the matrix
    r_3_1 = 0
    r_3_2 = np.sin(alpha)
    r_3_3 = np.cos(alpha)
    r_3_4 = d

    # Row 4 of the matrix
    r_4_1 = 0
    r_4_2 = 0
    r_4_3 = 0
    r_4_4 = 1

    # Build the complete transformation matrix in an np.array
    T = np.array([[r_1_1, r_1_2, r_1_3, r_1_4],
                  [r_2_1, r_2_2, r_2_3, r_2_4],
                  [r_3_1, r_3_2, r_3_3, r_3_4],
                  [r_4_1, r_4_2, r_4_3, r_4_4]])
    return T

def fk_calc(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    T_all = []  # Initialize a list of the transforms we will need

    T_0_0 = np.array([[1, 0, 0, 0],  # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)  # Put our first transform in our list

    # In this loop we want to calculate the transform from the base frame to each joint
    # for example, if we have 3 rows in our DH-table we will have 4 transforms
    # since we include the identity base frame as initialized above. Once we calculate each
    # transform we will append it to the list.

    for i in range(numjoints):
        alpha, a, d, theta = dh_para[i, :]
        theta = theta + q[i]             
        T = dh(alpha, a, d, theta)       # Get each transformation matrix
        T_0_i = np.matmul(T_all[i],T)    # Do some calculations here
        T_all.append(T_0_i)              # put them all in a list

    return T_all

def jacobian_fromVP(q: np.ndarray, dh_para: np.ndarray):

    # this function should calculate the robot jacobian using the velocity propagation approach
    # you should use your dh_to_T function to calculate your homogenous transforms
    # follow the method from class to calculate you z and t vectors
    
    Jacobian = np.zeros((6, 3))
    
    T_0_0 = np.array([[1, 0, 0, 0],         # create T_0_0
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    
    # Calculate each of the homogenous transforms for the robot at the prescribed joint position
    T_0_1 = dh(dh_para[0, 0], dh_para[0, 1], dh_para[0, 2], q[0])
    T_1_2 = dh(dh_para[1, 0], dh_para[1, 1], dh_para[1, 2], q[1])
    T_2_3 = dh(dh_para[2, 0], dh_para[2, 1], dh_para[2, 2], q[2])
    
    T_0_2 = fk_calc(q, dh_para, 3)[-2]
    T_0_3 = fk_calc(q, dh_para, 3)[-1]
    
    # Use the transformation matrices to calculate the z and t vectors needed for the velocity
    # propagation method as demonstrated in class.
    z_0 = T_0_0[0:3, 2]
    t_0 = T_0_0[0:3, 3]
    
    z_1 = T_0_1[0:3, 2]
    t_1 = T_0_1[0:3, 3]
    
    z_2 = T_0_2[0:3, 2]
    t_2 = T_0_2[0:3, 3]
    
    z_3 = T_0_3[0:3, 2]
    t_3 = T_0_3[0:3, 3]
    
    # Build the Jacobian
    
    # Column 1
    Jacobian[0:3, 0] = np.round(np.cross(z_0, (t_3-t_0)), 3)
    Jacobian[3:6, 0] = np.round(z_0, 3)
    
    # Column 2
    Jacobian[0:3, 1] = np.round(np.cross(z_1, (t_3-t_1)), 3)
    Jacobian[3:6, 1] = np.round(z_1, 3)
    
    # Column 3
    Jacobian[0:3, 2] = np.round(np.cross(z_2, (t_3-t_2)), 3)
    Jacobian[3:6, 2] = np.round(z_2, 3)
    
    
    return Jacobian


def jacobianMoreJoints(q: np.ndarray, dh_para: np.ndarray, numjoints=3):
    # This function should calculate the jacobian for a robot with 1 or more joints
    # you should use a loop to calculate the homogenous transform matrices and then
    # extract your z and t vectors from those matrices before using them to build
    # the complete jacobian.


    Jacobian = np.zeros((6,numjoints))  # Initialize the jacobian size correctly

    T_all = []                          # Initialize an empty list of the transforms we will need

    T_0_0 = np.array([[1, 0, 0, 0],     # create T_0_0
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

    T_all.append(T_0_0)                 #Put our first transform in our list


    # In this loop we want to calculate the transform from the base frame to each joint
    # for example, if we have 3 rows in our DH-table we will have 4 transforms
    # since we include the identity base frame as initialized above. Once we calculate our
    # transform we will append it to the list.

    for i in range(numjoints):
        alpha, a, d, theta = dh_para[i, :]
        theta = theta + q[i]                #
        T = dh(alpha, a, d, theta)          # find the homogenous transform to the next joint
        T_0_i = np.matmul(T_all[i],T)       # find the homogenous transform from the baseframe to each joint
        T_all.append(T_0_i)                 #append each transform to the list

    # To calculate our Jacobian we need the final transform to the endpoint so lets get that:
    T_0_end = T_all[-1]              # get the last homogenous transform in our list
    t_end = T_0_end[:3,-1]           # get the t vector from that transform


    # In this loop we will get our z and t vectors for each transform
    # once we have our z and t vectors we build the jacobian 1 column at a time.
    for i in range(numjoints):
        T_0_i = T_all[i]       # get each transform
        z_i =  T_0_i[:3,-2]    # get the z vector for the current transform
        t_i =  T_0_i[:3,-1]    # get the t vector for the current transform

        # within the same loop, calculate each column of the jacobian
        # using velocity propagation
        Jacobian[0:3, i] = np.round(np.cross(z_i, (t_end-t_i)), 3)  # linear components
        Jacobian[3:6, i] = np.round(z_i,3)  # angular components


    return Jacobian 


if __name__ == "__main__":\

 # PART (2b): Fill in what you need to run part (2b) and return the final transform from the base frame
 # to the end-point frame. I have added some code here to get you started.
    h1 = 0.5
    l2 = 1.4
    l3 = 1.2
    alpha1 = np.pi/2

    a = [0 , l2, l3]
    d = [h1, 0, 0]
    alpha = [alpha1, 0, 0]
    theta = [0,0,0]

    dh_paras = np.array([alpha, a, d, theta]).T
    q0 = [0.0, 0.0, 0.0]
    T0 = fk_calc(q0, dh_paras, 3)

 # PART (2c): Fill in what you need to run part (2c) and plot the end-point position as all three joints
 # move from 0 to pi/4.
    positions = []
    
    step_size = 0.01
    
    for dq in np.arange(0, np.pi/4, step_size):
        q = [dq, dq, dq]
        T = fk_calc(q, dh_paras, 3)[-1]
        position = T[:3, -1]
        positions.append(position)
    
    positions = np.array(positions)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(positions[:,0], positions[:,1], positions[:,2], c='r', marker='o')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

 # Part (3c): Fill in what you need to run part (3c) and compare the two Jacobian functions you have created.
 # I recommend testing them at the 0 position first and then trying others positions. Testing 3-5 positions is sufficient.
 # Remember to use your dh() and fk_calc() functions as needed.

qs = [[0.0, 0.0, 0.0],
     [0.4, 0.2, 1.0],
     [0.1, -0.3, 1.3],
     [2, 0.3, -0.2],
     [0.3, 0.6, 0.7]]   

for q in qs:
    jacobian1 = jacobian_fromVP(q, dh_paras)
    jacobian2 = jacobianMoreJoints(q, dh_paras, 3)
    assert np.array_equal(jacobian1, jacobian2)

 # Part (3d): Fill in what you need to run part (3d) and produce the end-point position and Jacobian for the
 # UR robot at the two requested positions. Remember to use your dh() and fk_calc() functions as needed.
l1 = 0.1519
l2 = 0.24365
l3 = 0.21325
l4 = 0.11235
l5 = 0.08535
l6 = 0.0819

q0 = [0,0,0,0,0,0]
q1 = [0, np.pi/4, np.pi/4, 0, np.pi/3, 0]

a = [0 , -l2, -l3, 0, 0, 0]
d = [l1, 0, 0, l4, l5, l6]
alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
theta = [0,0,0,0,0,0]

dh_paras = np.array([alpha, a, d, theta]).T

print("q0:", q0)
print("q1:", q1)

# Final position
T_0 = fk_calc(q0, dh_paras, 6)[-1]
position_0 = T_0[0:3, -1]
print("\nFinal position for q0:", position_0)

T_1 = fk_calc(q1, dh_paras, 6)[-1]
position_1 = T_1[0:3, -1]
print("Final position for q1:", position_1)

# Jacobian
jacobian_0 = jacobianMoreJoints(q0, dh_paras, 6)
jacobian_1 = jacobianMoreJoints(q1, dh_paras, 6)

print("\nJacobian for q0:")
print(jacobian_0)
print("\nJacobian for q1:")
print(jacobian_1)



