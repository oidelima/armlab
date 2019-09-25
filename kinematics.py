import numpy as np
import math
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

#link lengths in mm
L1 = 41.38
L2 = 99.2
L3 = 66.9
L4 = 41.9
L5 = 55

def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    dhtable = np.array([[0, math.pi/2, L1, math.pi/2+joint_angles[0]], 
          [L2, 0, 0, math.pi/2+joint_angles[1]],
          [0, math.pi/2, 0, math.pi/2+joint_angles[2]],
          [0, math.pi/2, L3+L4, math.pi + joint_angles[3]]])

    #link thetas
    th1 = dhtable[0,3];
    th2 = dhtable[1,3];
    th3 = dhtable[2,3];
    th4 = dhtable[3,3];

    #link alphas
    al1 = dhtable[0,1];
    al2 = dhtable[1,1];
    al3 = dhtable[2,1];
    al4 = dhtable[3,1];

    A1 = np.array([[math.cos(th1), -math.sin(th1)*math.cos(al1), math.sin(th1)*math.sin(al1), 0],
        [math.sin(th1), math.cos(th1)*math.cos(al1), -math.cos(th1)*math.sin(al1), 0],
        [0, math.sin(al1), math.cos(al1), dhtable[0, 2]],
        [0, 0, 0, 1]])
    
    A2 = np.array([[math.cos(th2), -math.sin(th2)*math.cos(al2), math.sin(th2)*math.sin(al2), L2*math.cos(th2)],
        [math.sin(th2), math.cos(th2)*math.cos(al2), -math.cos(th2)*math.sin(al2), L2*math.sin(th2)],
        [0, math.sin(al2), math.cos(al2), dhtable[1, 2]],
        [0, 0, 0, 1]])
    
    A3 = np.array([[math.cos(th3), -math.sin(th3)*math.cos(al3), math.sin(th3)*math.sin(al3), 0],
        [math.sin(th3), math.cos(th3)*math.cos(al3), -math.cos(th3)*math.sin(al3), 0],
        [0, math.sin(al3), math.cos(al3), dhtable[2, 2]],
        [0, 0, 0, 1]])
    
    A4 = np.array([[math.cos(th4), -math.sin(th4)*math.cos(al4), math.sin(th4)*math.sin(al4), 0],
        [math.sin(th4), math.cos(th4)*math.cos(al4), -math.cos(th4)*math.sin(al4), 0],
        [0, math.sin(al4), math.cos(al4), dhtable[3, 2]],
        [0, 0, 0, 1]])
        
    H1 = A1
    H2 = np.matmul(H1, A2)
    H3 = np.matmul(H2, A3)
    H4 = np.matmul(H3, A4)

    if link == 1: return H1
    elif link == 2: return H2
    elif link == 3: return H3
    elif link == 4: return H4
    


def kf_test():
	pass

    #print("Elbow at 90 deg gives: ", np.matmul(FK_dh([0,0,math.pi/2,0,0], 4), np.array([0,0,0,1]))) # elbow at 90
    #print("Elbow at -90 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/2,0,0], 4), np.array([0,0,0,1]))) # elbow at -90
    #print("Elbow at 45 deg gives: ", np.matmul(FK_dh([0,0,math.pi/4,0,0], 4), np.array([0,0,0,1]))) # elbow at 45
    #print("Elbow at -45 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/4,0,0], 4), np.array([0,0,0,1]))) # elbow at -45
    #print("Shoulder at 90 deg gives: ", np.matmul(FK_dh([0,math.pi/2,0, 0,0], 4), np.array([0,0,0,1]))) # shoulder at 90
    #print("Shoulder at -90 deg gives: ", np.matmul(FK_dh([0,-math.pi/2,0,0,0], 4), np.array([0,0,0,1]))) # shoulder at -90
    #print("Shoulder at 45 deg gives: ", np.matmul(FK_dh([0,math.pi/4,0,0,0], 4), np.array([0,0,0,1]))) # shoulder at 45
    #print("Shoulder at -45 deg gives: ", np.matmul(FK_dh([0,-math.pi/4,0,0,0], 4), np.array([0,0,0,1]))) # shoulder at -45
    #print("Shoulder at -45 deg, base at 90, elboow at -45 gives: ", np.matmul(FK_dh([math.pi/2,-math.pi/4,-math.pi/4,0,0], 4), np.array([0,0,0,1]))) # shoulder at -45

#kf_test()

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    umath.sing product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(pose):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass