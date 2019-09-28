import numpy as np
import math
from math import *
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

#link lengths in mm
GROUND_OFFSET = 0.0
BASE = 42.8
SHOULDER = 99.29
ELBOW = 67.2
WRIST_Z_1 = 42.3
WRIST_X = 47.6
WRIST_Z_2 = 59.8
links = [GROUND_OFFSET, BASE, SHOULDER, ELBOW, WRIST_Z_1, WRIST_X, WRIST_Z_2]

def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention
    return a transformation matrix representing the pose of the 
    desired link
    note: phi is the euler angle about the y-axis in the base frame

    """
    dh_t = np.array([[     0.0, pi/2,            links[1], pi/2 + joint_angles[0]], 
          			 [links[2],  0.0,                 0.0, pi/2 + joint_angles[1]],
          			 [     0.0, pi/2,                 0.0, pi/2 + joint_angles[2]],
          			 [     0.0, pi/2, links[3] + links[4], pi/1 + joint_angles[3]],
          			 [     0.0, pi/2,                 0.0, pi/1 + joint_angles[4]],
          			 [     0.0,  0.0, links[5] + links[6],        joint_angles[5]]])

    #link θs
    #θs = [ 0.0, dh_t[0,3], dh_t[1,3], dh_t[2,3], dh_t[3,3], dh_t[4,3], dh_t[5,3]]
    
    # th1 = dh_t[0,3]
    # th2 = dh_t[1,3]
    # th3 = dh_t[2,3]
    # th4 = dh_t[3,3]
    # th5 = dh_t[4,3]
    # th6 = dh_t[5,3]

    #link alphas
    #alphas = [ 0.0, dh_t[0,1], dh_t[1,1], dh_t[2,1], dh_t[3,1], dh_t[4,1], dh_t[5,1]]
    # al1 = dh_t[0,1]
    # al2 = dh_t[1,1]
    # al3 = dh_t[2,1]
    # al4 = dh_t[3,1]
    # al5 = dh_t[4,1]
    # al6 = dh_t[5,1]

    θ = [ 0.0, dh_t[0,3], dh_t[1,3], dh_t[2,3], dh_t[3,3], dh_t[4,3], dh_t[5,3]]
    
    α = [ 0.0, dh_t[0,1], dh_t[1,1], dh_t[2,1], dh_t[3,1], dh_t[4,1], dh_t[5,1]]

    A1 = np.array([[ cos(θ[1]), - sin(θ[1]) * cos(α[1]),   sin(θ[1]) * sin(α[1]),          0],
        		   [ sin(θ[1]),   cos(θ[1]) * cos(α[1]), - cos(θ[1]) * sin(α[1]),          0],
        		   [ 		 0,               sin(α[1]),               cos(α[1]), dh_t[0, 2]],
        		   [ 		 0,                       0,                       0,          1]])
    
    A2 = np.array([[ cos(θ[2]), - sin(θ[2]) * cos(α[2]),   sin(θ[2]) * sin(α[2]), links[2] * cos(θ[2])],
        		   [ sin(θ[2]),   cos(θ[2]) * cos(α[2]), - cos(θ[2]) * sin(α[2]), links[2] * sin(θ[2])],
        		   [ 		 0,   			  sin(α[2]),   			   cos(α[2]),  	        dh_t[1, 2]],
        		   [ 		 0, 					  0, 					   0, 		       	     1]])
    
    A3 = np.array([[ cos(θ[3]), - sin(θ[3]) * cos(α[3]),   sin(θ[3]) * sin(α[3]), 			   0],
   			       [ sin(θ[3]),   cos(θ[3]) * cos(α[3]), - cos(θ[3]) * sin(α[3]), 			   0],
        		   [		 0,  			  sin(α[3]),   			   cos(α[3]), 	  dh_t[2, 2]],
    		       [		 0, 					  0, 					   0,			   1]])
    
    A4 = np.array([[ cos(θ[4]), - sin(θ[4]) * cos(α[4]),   sin(θ[4]) * sin(α[4]), 			   0],
        		   [ sin(θ[4]),   cos(θ[4]) * cos(α[4]), - cos(θ[4]) * sin(α[4]), 			   0],
        		   [		 0,  			  sin(α[4]),  			   cos(α[4]), 	  dh_t[3, 2]],
        		   [		 0, 					  0, 					   0, 			   1]])

    A5 = np.array([[ cos(θ[5]), - sin(θ[5]) * cos(α[5]),   sin(θ[5]) * sin(α[5]), 			   0],
        		   [ sin(θ[5]),   cos(θ[5]) * cos(α[5]), - cos(θ[5]) * sin(α[5]), 			   0],
        		   [		 0,  			  sin(α[5]),   			   cos(α[5]), 	  dh_t[4, 2]],
        		   [		 0, 					  0, 					   0, 			   1]])

    A6 = np.array([[ cos(θ[6]), - sin(θ[6]) * cos(α[6]),   sin(θ[6]) * sin(α[6]), 			   0],
        		   [ sin(θ[6]),   cos(θ[6]) * cos(α[6]), - cos(θ[6]) * sin(α[6]), 			   0],
        		   [		 0,  			  sin(α[6]),  			   cos(α[6]), 	  dh_t[5, 2]],
        		   [		 0, 					  0, 					   0, 			   1]])
        
    H1 = A1
    H2 = np.matmul(H1, A2)
    H3 = np.matmul(H2, A3)
    H4 = np.matmul(H3, A4)
    H5 = np.matmul(H4, A5)
    H6 = np.matmul(H5, A6)

    if   link == 1: return H1
    elif link == 2: return H2
    elif link == 3: return H3
    elif link == 4: return H4
    elif link == 5: return H5
    elif link == 6: return H6
    


def kf_test():
    #print("Elbow at 90 deg gives: ", np.matmul(FK_dh([0,0,math.pi/2,0,0,0], 6), np.array([0,0,0,1]))) # elbow at 90
    #print("Elbow at -90 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/2,0,0,0], 6), np.array([0,0,0,1]))) # elbow at -90
    #print("Elbow at 45 deg gives: ", np.matmul(FK_dh([0,0,math.pi/4,0,0,0], 6), np.array([0,0,0,1]))) # elbow at 45
    #print("Elbow at -45 deg gives: ", np.matmul(FK_dh([0,0,-math.pi/4,0,0,0], 6), np.array([0,0,0,1]))) # elbow at -45
    #print("Shoulder at 90 deg gives: ", np.matmul(FK_dh([0,math.pi/2,0, 0,0,0], 6), np.array([0,0,0,1]))) # shoulder at 90
    #print("Shoulder at -90 deg gives: ", np.matmul(FK_dh([0,-math.pi/2,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at -90
    #print("Shoulder at 45 deg gives: ", np.matmul(FK_dh([0,math.pi/4,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at 45
    #print("Shoulder at -45 deg gives: ", np.matmul(FK_dh([0,-math.pi/4,0,0,0,0], 6), np.array([0,0,0,1]))) # shoulder at -45
    #print("Shoulder at -45 deg, base at 90, elboow at -45 gives: ", np.matmul(FK_dh([math.pi/2,-math.pi/4,-math.pi/4,0,0,0], 4), np.array([0,0,0,1]))) # shoulder at -45
    pass

kf_test()

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    use math.sing product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(o , R):
    """ TODO: Calculate inverse kinematics for rexarm return the required joint angles """
    
    oc = np.array([[o[0] - (links[6]+links[5])*R[0][2]], [o[1] - (links[6] + links[5])*R[1][2]], [o[2] - (links[6] + links[5])*R[2][2]]])
    oc.round(4)
    
    θ1 = atan2(oc[0], oc[1]) + pi #two possibilities
    
    θ3 =  -acos((oc[0]**2 + oc[1]**2 + (oc[2]-L1)**2 - (L3+L4)**2 - L2**2)/(2*(L3+L4)*L2)) #two possibilities
    
    θ2 = pi/2 - (atan2(oc[2]-L1, sqrt(oc[0]**2 + oc[1]**2)) - atan2((L3+L4)*sin(θ3), L2 + (L3+L4)*cos(θ3))) #two possibilities because of θ 3
    
    θ1 = round(θ1, 6)


    R03 = np.array([[ cos(θ1)*(cos(θ2+θ3)), -cos(θ1)*(sin(θ2+θ3)),  sin(θ1)],
                    [ sin(θ1)*(cos(θ2+θ3)), -sin(θ1)*(sin(θ2+θ3)), -cos(θ1)],
                    [           sin(θ2+θ3),            cos(θ2+θ3),        0]])

    

    R36 = np.matmul(R03.transpose(),R)

    θ4 = atan2(R36[1][2], R36[0][2])
    
    θ5 = atan2(sqrt(1 - R36[2][2]**2), R36[2][2])
    
    θ6 = atan2(R36[2][1], - R[2][0])

    return [θ1, θ2, θ3, θ4, θ5, θ6]

def ik_test():

    #testing elbow at 90 deg
    Rfull = FK_dh([0,0,pi/2,0,0,0], 6)
    R = Rfull[0:3, 0:3]
    o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
    #print("Elbow at 90 gives: ",IK(o,R))
    """
    #testing elbow at -90
    Rfull = FK_dh([0,0,-math.pi/2,0,0,0], 6)
    R = Rfull[0:3, 0:3]
    o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
    print("Elbow at -90 gives: ",IK(o,R))

    #testing elbow at 45
    Rfull = FK_dh([0,0,math.pi/4,0,0,0], 6)
    R = Rfull[0:3, 0:3]
    o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
    print("Elbow at 45 gives: ",IK(o,R))

    #testing shoulder at 90
    Rfull = FK_dh([0,math.pi/2,0, 0,0,0], 6)
    R = Rfull[0:3, 0:3]
    o = np.array([Rfull[0][3], Rfull[1][3], Rfull[2][3]]) 
    print("Shoulder at 90 gives: ",IK(o,R))
"""



ik_test()

def get_euler_angles_from_T(T):
    """ TODO: implement this function return the Euler angles from a T matrix """
    θ = 0.0
    φ = 0.0
    ψ = 0.0

    if T[1,3] or T[2,3]:
        θ = atan2(T[3,3], pow((1 - pow(T[3,3],2), 0.5))
        if sin(θ) > 0:
            φ = atan2(T[1,3], T[2,3])
            ψ = atan2(-T[3,1], T[3,2])
        else:
            φ = atan2(-T[1,3], T[2,3])
            ψ = atan2(-T[3,1], -T[3,2])

    else:
        #Determine θ
        if T[3,3] == 0:
            θ = 0  
        #Need to determine φ + ψ assuming φ = 0 by convention: Determine ψ 
            ψ =  atan2(T[1,1], -T[1,2])

        else:
            θ = pi
            #Need to determine φ + ψ assuming φ = 0 by convention: Determine ψ 
            ψ =  atan2(-T[1,1], -T[1,2])

    return (θ, φ, ψ) 

def get_pose_from_T(T):
    """ TODO: return the joint pose from a T matrix of the form (x,y,z,phi) where phi is rotation about base frame y-axis """
    x = T[0]
    y = T[1]
    z = T[2]
    φ = T[3]

    matrix = np.array([[ cos(φ), 0, sin(φ), x],
                       [      0, 1,      0, y],
                       [-sin(φ), 0, cos(φ), z],
                       [      0, 0,      0, 1]])
    pass




def to_s_matrix(w,v):
    """ TODO: implement this function Find the [s] matrix for the POX method e^([s]*θ) """
    pass